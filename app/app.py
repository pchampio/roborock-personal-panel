"""Feature-complete web control panel for a Roborock vacuum via a LOCAL stack.

Targets parity with the Roborock mobile app for controlling a single device,
using the pinned, patched python-roborock client pointed at your own stack (base_url) — no cloud.

Feature areas (tabs in the UI):
  Clean       start/pause/stop/dock/find, room (segment) clean, zone clean,
              go-to point, cycles, per-clean fan/water/mop
  Modes       default fan power / water flow / mop route / repeat
  Map         multi-map switch, rename rooms, reset map
  Dock        auto-empty mode, mop-wash mode, smart-wash params, trigger
              empty / wash / wash-then-charge
  Maintenance consumable life + reset, network info, lifetime stats, history
  Settings    Do-Not-Disturb schedule, volume, child lock, status LED,
              indicator (flow) LED, off-peak (valley) timer
  Schedule    list + run routines/scenes
  Info        full live status, device capabilities

Includes optional WebRTC video and audio. OTA firmware and voice packs are out of scope.

    python -m venv .venv && ./.venv/bin/pip install -r requirements.txt
    cp .env.example .env        # RR_EMAIL / RR_PIN
    set -a; source .env; set +a
    ./.venv/bin/python app.py   # open http://127.0.0.1:8099
"""

from __future__ import annotations

import asyncio
import base64
import enum
import hashlib
import json
import logging
import os
import unicodedata
import sys
import time
import typing
from pathlib import Path

from aiohttp import WSMsgType, web

from roborock import (
    FILTER_REPLACE_TIME,
    MAIN_BRUSH_REPLACE_TIME,
    RoborockCommand,
    RoborockDockDustCollectionModeCode,
    RoborockException,
    RoborockStateCode,
    SENSOR_DIRTY_REPLACE_TIME,
    SIDE_BRUSH_REPLACE_TIME,
)
from roborock.web_api import RoborockApiClient
from roborock.devices.device import RoborockDevice
from roborock.devices.device_manager import DeviceManager, UserParams, create_device_manager
from roborock.devices.traits.v1.consumeable import ConsumableAttribute
from roborock.data.v1.v1_containers import DnDTimer
# 6.x models the app's own semantics: high-level cleaning modes plus the exact
# fan/water/route options a given device supports. Prefer these over guessing.
from roborock.data.v1.v1_clean_modes import (
    WATER_SLIDE_MODE_MAPPING,
    CleanRoutes,
    CleaningMode,
    VacuumModes,
    WashTowelModes,
    WaterModes,
    get_clean_modes,
    get_clean_routes,
    get_cleaning_mode_options,
    get_water_mode_mapping,
    get_water_modes,
)

import jobs as jobs_mod
import patrol as patrol_mod

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
_LOGGER = logging.getLogger("rr-demo")

BASE_URL = os.environ.get("RR_BASE_URL", "http://localhost:555")
EMAIL = os.environ.get("RR_EMAIL", "")
PIN = os.environ.get("RR_PIN", "")
HOST = os.environ.get("RR_UI_HOST", "127.0.0.1")
PORT = int(os.environ.get("RR_UI_PORT", "8099"))
REGION = os.environ.get("RR_REGION", "eu")  # only affects the cn-specific mop route
# If barriers/zones/go-to land vertically mirrored on hardware, set RR_COORD_FLIP_Y=1.
FLIP_Y = os.environ.get("RR_COORD_FLIP_Y", "0").strip() in ("1", "true", "yes")

# Room floor materials (set_segment_ground_material) and carpet modes (set_carpet_clean_mode).
GROUND_MATERIALS = {"generic": 0, "wood": 3, "tile": 4}
CARPET_CLEAN_MODES = {"avoid": 0, "lift": 1, "off (cross over)": 2, "lift (camera)": 3}

HERE = Path(__file__).parent

STATE_LABELS = {e.value: e.name for e in RoborockStateCode}

# RoborockStateCode is verbatim the 2016 Xiaomi Mi Robot Vacuum v1 table, and no
# model-specific override exists for newer hardware (Saros10RStatus overrides
# fan_power / water_box_mode / mop_mode only, and inherits `state` unchanged).
# So a handful of its names are actively misleading here: code 2 renders as
# "charger_disconnected" while the robot is mid-clean and halted on a brush jam.
# Never show a bare state name where the code is unverified for this model --
# _status_dict pairs it with `state_trusted` and the UI falls back to the error
# or the motion flags. See jobs.describe_error for the same policy on errors.
UNVERIFIED_STATE_CODES = {2}

# What a code has actually been SEEN to mean on this model, as opposed to what the
# v1 enum calls it. Separate table on purpose: these are observations, they do not
# make the code trusted, and `state_trusted` stays false for anything in
# UNVERIFIED_STATE_CODES so the error and the motion flags still win where they
# matter. This only gives the chip something better to say than "state 2".
#
# 2 -- the enum says charger_disconnected. Read off the official app on
#      2026-08-22: it shows "En veille" (standby) for this code, on an idle robot
#      parked on the dock, which is consistent with everything else the panel sees
#      at the time (running false, in_cleaning 0, battery full).
#      NOT the whole story: the note above records code 2 also appearing mid-clean
#      while halted on a brush jam. So it is a state the firmware reuses, and the
#      wording is only honest while nothing else contradicts it -- which is exactly
#      what the trusted/untrusted split already arranges.
MODEL_STATE_LABELS = {2: "en veille"}

# How long the tracker's background sampler waits between polls while the robot
# is doing something. Errors get timestamped from these samples, so this also
# bounds how stale "in this state for N" can be.
TRACK_INTERVAL_ACTIVE = 30.0
TRACK_INTERVAL_IDLE = 60.0
# With a browser attached. The point of these is NOT to poll faster than before
# but to poll at all only where the device volunteers nothing: progress numbers
# and the map. Everything a datapoint reports arrives by push regardless.
# 1s is affordable here because the broker and the local API are containers on
# this same box (RR_MQTT_URL=mqtt://@mosquitto:1883, RR_BASE_URL=http://roborock:555):
# the panel-to-broker leg is loopback, and only the vacuum's own WiFi hop is real.
# A deployment talking to the Roborock cloud instead should raise both.
WATCH_INTERVAL_ACTIVE = float(os.environ.get("RR_WATCH_ACTIVE", "1"))
# Watched and idle. Not just a safety net: the fields that report PHYSICAL state --
# the dust bin (rss), the mop plate, the water tank -- carry no `dps` metadata, so
# the vacuum never pushes them. They change only when the status is re-read, and at
# 30s taking the bin out took twenty seconds to show. The device answers promptly
# when asked; the delay was entirely ours.
# One small command every 5s while a browser is open and nothing is running -- still
# five times lighter than the per-second poll every tab used to make on its own.
WATCH_INTERVAL_IDLE = float(os.environ.get("RR_WATCH_IDLE", "5"))
WATCH_MAP_ACTIVE = float(os.environ.get("RR_WATCH_MAP", "6"))
WATCH_TL_GAP = 5.0
# The consumable counters. Pushed datapoints keep the brushes and the filter
# current once they arrive, but nothing populates the trait to begin with -- and
# the ones with no datapoint (the sensors, the mop) are only ever read. So they
# are refreshed on a slow timer: they move by an hour of use at a time, so half an
# hour of staleness is nothing, and it is one command.
WEAR_REFRESH = 1800.0
MAP_IDLE_GAP = 300.0
TRACKER = jobs_mod.Tracker()
# Small JSON log of what each run actually did. The device keeps no record of
# the settings a run used, so the sampler writes them down as it sees them.
RUNS_PATH = os.environ.get("RR_RUNS_DB", str(HERE / "runs.json"))
RUNS = jobs_mod.RunLog(RUNS_PATH)
# Named jobs -- rooms or zones plus the settings to clean them with. The vacuum
# cannot hold these: per-room settings ride along with the job and are gone when
# it ends, so a "clean the office the way I like it" button has to live here.
PRESETS_PATH = os.environ.get("RR_PRESETS_DB", str(HERE / "presets.json"))
PRESETS = jobs_mod.PresetStore(PRESETS_PATH)
# What the vacuum drove around, kept because the vacuum does not keep it: see
# jobs.ObstacleArchive. Photos live in a sibling <stem>_photos/ directory.
OBSTACLES_PATH = os.environ.get("RR_OBSTACLES_DB", str(HERE / "obstacles.json"))
ARCHIVE = jobs_mod.ObstacleArchive(OBSTACLES_PATH)

# States that mean "a cleaning run is under way" (as opposed to docked, washing
# or idle). Used to decide when to open and close a run-log entry.
CLEANING_STATES = {
    RoborockStateCode.cleaning.value,
    RoborockStateCode.spot_cleaning.value,
    RoborockStateCode.zoned_cleaning.value,
    RoborockStateCode.segment_cleaning.value,
    RoborockStateCode.returning_home.value,
    RoborockStateCode.going_to_target.value,
}

# Whole-device actions that take no params.
SIMPLE_ACTIONS: dict[str, RoborockCommand] = {
    "start": RoborockCommand.APP_START,
    "pause": RoborockCommand.APP_PAUSE,
    "stop": RoborockCommand.APP_STOP,
    "dock": RoborockCommand.APP_CHARGE,
    "find": RoborockCommand.FIND_ME,
    "resume_segment": RoborockCommand.RESUME_SEGMENT_CLEAN,
    "resume_zone": RoborockCommand.RESUME_ZONED_CLEAN,
    "collect_dust": RoborockCommand.APP_START_COLLECT_DUST,
    "wash_mop": RoborockCommand.APP_START_WASH,
    "stop_wash": RoborockCommand.APP_STOP_WASH,
    "wash_then_charge": RoborockCommand.START_WASH_THEN_CHARGE,
}

# Consumables that can be reset, with their lifetime limit (seconds).
CONSUMABLES = {
    "main_brush": (ConsumableAttribute.MAIN_BRUSH_WORK_TIME, "main_brush_work_time", MAIN_BRUSH_REPLACE_TIME),
    "side_brush": (ConsumableAttribute.SIDE_BRUSH_WORK_TIME, "side_brush_work_time", SIDE_BRUSH_REPLACE_TIME),
    "filter": (ConsumableAttribute.FILTER_WORK_TIME, "filter_work_time", FILTER_REPLACE_TIME),
    "sensor": (ConsumableAttribute.SENSOR_DIRTY_TIME, "sensor_dirty_time", SENSOR_DIRTY_REPLACE_TIME),
}


# --------------------------------------------------------------------------- #
# App state
# --------------------------------------------------------------------------- #
class AppState:
    def __init__(self) -> None:
        self.manager: DeviceManager | None = None
        self.devices: list[RoborockDevice] = []
        self.selected_duid: str | None = None
        self._lock = asyncio.Lock()

    async def login(self) -> None:
        if not EMAIL or not PIN:
            sys.exit("Set RR_EMAIL and RR_PIN (see .env.example).")
        api = RoborockApiClient(username=EMAIL, base_url=BASE_URL)
        try:
            await api.request_code()
        except Exception as exc:  # noqa: BLE001
            _LOGGER.info("request_code skipped: %s", exc)
        user_data = await api.code_login(PIN)
        _LOGGER.info("Logged in; MQTT endpoint -> %s", user_data.rriot.r.m)
        params = UserParams(username=EMAIL, user_data=user_data, base_url=BASE_URL)
        self.manager = await create_device_manager(params)
        self.devices = await self.manager.get_devices()
        if self.devices:
            self.selected_duid = self.devices[0].duid
            _LOGGER.info("Devices: %s", [d.name for d in self.devices])
        else:
            _LOGGER.warning("No devices in the account snapshot.")

    _connected: set[str] = set()

    def device(self) -> RoborockDevice | None:
        if self.selected_duid is None:
            return None
        return next((d for d in self.devices if d.duid == self.selected_duid), None)

    async def props(self) -> typing.Any:
        device = self.device()
        if device is None or device.v1_properties is None:
            raise web.HTTPBadRequest(reason="no device / not a v1 vacuum")
        # 6.x: connect() subscribes once and the library reconnects in the
        # background. is_connected tracks the CHANNEL, which can read false
        # while the subscription is already held -- calling connect() again
        # then raises "Only one subscription allowed at a time". So connect at
        # most once per device and let the library manage the link.
        if device.duid not in self._connected:
            self._connected.add(device.duid)
            try:
                await device.connect()
            except ValueError as exc:  # already subscribed
                _LOGGER.debug("connect skipped for %s: %s", device.duid, exc)
            except Exception:
                self._connected.discard(device.duid)  # let a real failure retry
                raise
            # Feature-gated traits (obstacle photos, and the obstacle type /
            # description / confidence that come with them) do not exist until
            # the device's features have been discovered.
            try:
                await asyncio.wait_for(device.v1_properties.discover_features(), timeout=120)
            except Exception:  # noqa: BLE001
                _LOGGER.warning("discover_features failed; obstacle details will be missing", exc_info=True)
            self._listen(device)
            _journal_commands(device.v1_properties)
        return device.v1_properties

    _listening: set[str] = set()

    def _listen(self, device: RoborockDevice) -> None:
        """React to the device instead of asking it.

        The vacuum pushes its own changes over MQTT as DPS datapoints, and the
        library decodes them straight into the status trait -- so by the time this
        callback runs, `props.status` is already the new truth and there is nothing
        to fetch. add_update_listener is the library's own hook for it; the trait
        fires it only when a datapoint actually changed a field, so an idle robot
        is silent rather than chatty.
        """
        if device.duid in self._listening or device.v1_properties is None:
            return
        try:
            device.v1_properties.status.add_update_listener(_mark_status_dirty)
        except Exception:  # noqa: BLE001 -- polling still works; do not fail the request
            _LOGGER.warning("could not subscribe to status updates", exc_info=True)
            return
        self._listening.add(device.duid)
        _LOGGER.info("subscribed to pushed status updates for %s", device.duid)

    async def close(self) -> None:
        if self.manager is not None:
            await self.manager.close()


STATE = AppState()


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
# Which device we have already read the feature flags off. They describe the
# firmware -- which suction levels, water levels and mop routes exist -- not the
# moment, so they cannot change while the process runs. Re-reading them was two
# MQTT round trips on every single /api/state (_mode_options and _cleaning_modes
# each asked), the 30s poll included.
_FEATURES_READ: set[str] = set()


async def _device_features(props: typing.Any) -> typing.Any:
    """The refreshed DeviceFeatures trait, or None.

    DeviceFeaturesTrait subclasses DeviceFeatures, but every flag reads
    False until it has been refreshed -- which silently produced empty
    mode tables and removed controls from the UI. Refreshed ONCE per device: the
    trait keeps its flags, and they are a property of the firmware.
    """
    feats = getattr(props, "device_features", None)
    if feats is None:
        return None
    duid = STATE.selected_duid or ""
    if duid in _FEATURES_READ:
        return feats
    try:
        await feats.refresh()
    except Exception:  # noqa: BLE001
        _LOGGER.warning("device_features refresh failed", exc_info=True)
        return None
    _FEATURES_READ.add(duid)   # only on success: a failed read must be retried
    return feats


async def _mode_options(props: typing.Any) -> dict[str, typing.Any]:
    """{fan|water|mop: {name: code}} for THIS device, from its own features.

    Replaces 4.20's annotation scraping. Values are the wire codes; names are
    the library's canonical mode names.
    """
    out: dict[str, typing.Any] = {"fan": {}, "water": {}, "mop": {}}
    feats = await _device_features(props)
    if feats is None:
        return out
    try:
        out["fan"] = {m.value: m.code for m in get_clean_modes(feats)}
        # WaterModes aliases several codes onto one name (low is 201 AND 225), and
        # `m.code` hands back the primary -- so on a slide-mode device the table
        # came out holding 201/202/203 rather than the slide codes 225/235/245 the
        # device actually drives. get_water_mode_mapping is the library's own
        # answer to that; it returns code -> name, which inverts to what the UI
        # wants. Later codes win on a name clash, which keeps the slide codes.
        out["water"] = {name: code for code, name in get_water_mode_mapping(feats).items()}
        # get_clean_routes also takes a region: one China-only "deep plus"
        # variant differs. Default to eu; override with RR_REGION if needed.
        out["mop"] = {m.value: m.code for m in get_clean_routes(feats, REGION)}
        if feats.is_water_slide_mode_supported:
            from roborock.data.v1.v1_clean_modes import (
                WATER_LEVEL_BANDS,
                WATER_LEVEL_BASE,
                WATER_LEVEL_MAX,
                WATER_LEVEL_MIN,
            )

            out["water_level"] = {
                "base": WATER_LEVEL_BASE,
                "min": WATER_LEVEL_MIN,
                "max": WATER_LEVEL_MAX,
                "bands": [[lo, hi, name] for lo, hi, name in WATER_LEVEL_BANDS],
            }
    except Exception:  # noqa: BLE001
        # Log loudly: an empty table silently removes controls from the UI.
        _LOGGER.warning("mode options unavailable", exc_info=True)
    return out


async def _cleaning_modes(props: typing.Any) -> list[str]:
    """High-level modes (vacuum / vac_and_mop / mop / ...) this device supports."""
    feats = await _device_features(props)
    if feats is None:
        return []
    try:
        return [m.value for m in get_cleaning_mode_options(feats)]
    except Exception:  # noqa: BLE001
        _LOGGER.warning("cleaning mode options unavailable", exc_info=True)
        return []


def _enum_options(status_cls: type | None, field: str) -> dict[str, int]:
    if status_cls is None:
        return {}
    try:
        annotation = typing.get_type_hints(status_cls).get(field)
    except Exception:  # noqa: BLE001
        return {}
    if annotation is None:
        return {}
    for arg in (typing.get_args(annotation) or [annotation]):
        if isinstance(arg, type) and issubclass(arg, enum.Enum):
            # Drop sentinel/unusable entries; keep everything else.
            return {e.name: int(e.value) for e in arg if e.name not in ("unknown",)}
    return {}


def _status_class(device: RoborockDevice) -> type | None:
    """Retired in 6.x.

    4.20 kept a model -> Status subclass registry (ModelStatus) and the mode
    options were read off that class's type annotations. 6.x drops it: the
    supported modes come from the device's own DeviceFeatures instead, via
    _mode_options(). Kept as a no-op so the call sites stay readable.
    """
    return None


async def _apply_setting(cmd: typing.Any, command: RoborockCommand, key: str, value: typing.Any) -> bool:
    """Send a setting, trying bare-list then dict encoding (models disagree)."""
    for params in ([value], [{key: value}]):
        try:
            await cmd.send(command, params)
            return True
        except RoborockException as exc:
            _LOGGER.info("%s %r rejected: %s", command, params, exc)
    return False


def _map_projector(map_data: typing.Any):
    """Return (to_pct(x,y)->(left%,top%), img_w, img_h) for the rendered PNG, or None.

    Converts a point in the parser's map coordinate space to a percentage
    position over the rendered image (so the browser can scale freely). Shared
    by room markers and barrier overlays.
    """
    image = getattr(map_data, "image", None) if map_data is not None else None
    if image is None or getattr(image, "is_empty", False):
        return None
    dims = getattr(image, "dimensions", None)  # ImageData keeps geometry on .dimensions
    if dims is None:
        return None
    from vacuum_map_parser_base.map_data import Point

    try:
        base_w, base_h = dims.width * dims.scale, dims.height * dims.scale
        rot = int(getattr(dims, "rotation", 0)) % 360
        img_w, img_h = (base_h, base_w) if rot in (90, 270) else (base_w, base_h)
        if not img_w or not img_h:
            return None

        def to_pct(x: float, y: float):
            # to_img().rotated() = exactly how the PNG (and calibration) maps points.
            p = Point(x, y).to_img(dims).rotated(dims)
            return (100.0 * p.x / img_w, 100.0 * p.y / img_h)

        to_pct(dims.left, dims.top)  # validate the transform runs
    except Exception:  # noqa: BLE001
        return None
    return to_pct, img_w, img_h


def _point_pct(to_pct: typing.Callable, pt: typing.Any) -> list[float] | None:
    """One map point as [left%, top%] over the rendered PNG, or None."""
    if pt is None:
        return None
    x, y = getattr(pt, "x", None), getattr(pt, "y", None)
    if x is None or y is None:
        return None
    try:
        left, top = to_pct(float(x), float(y))
    except Exception:  # noqa: BLE001
        return None
    return [round(left, 3), round(top, 3)]


# The map cannot tell us the robot is parked: it only rewrites `vacuum_position`
# while the robot is moving, so once it docks the last frame keeps showing it
# wherever it stopped. The status enum is the only thing that knows.
_DOCKED_STATES = {"charging", "charging_complete"}


# Where the robot was the last time we asked JUST that, as a map percentage.
# The badge and the picture want very different update rates: the badge is the
# answer to "where is it now" and wants to move continuously, while the trace is
# a picture that is perfectly readable a few seconds old. The blob carries both,
# so the split is not in the fetching -- it is in the parsing and the rendering,
# which is where the cost actually is. See _read_position.
_LIVE_POS: dict[str, typing.Any] = {"pct": None, "at": 0.0}
LIVE_POS_TTL = 30.0     # older than this and the map's own copy is no worse


def _vacuum_position(props: typing.Any, state_label: str) -> dict | None:
    """Where the robot actually is, as a percentage over the rendered map.

    Read together with the state, because neither half is enough on its own.
    The map has the coordinates but goes stale the moment the robot stops
    moving -- after an interrupted run it will still be pointing at the doorway
    the robot got stuck in, hours after it took itself back to the dock. The
    state knows it is charging but not where the dock is. The map does: it
    carries the charger's position, and "charging" means that is the answer.
    """
    try:
        md = getattr(getattr(props, "map_content", None), "map_data", None)
    except Exception:  # noqa: BLE001
        return None
    if md is None:
        return None
    proj = _map_projector(md)
    if proj is None:
        return None
    to_pct = proj[0]
    charger = _point_pct(to_pct, getattr(md, "charger", None))
    at_dock = state_label in _DOCKED_STATES
    if at_dock and charger:
        return {"pct": charger, "at_dock": True, "source": "charger"}
    # A position read since the last full map parse wins: same coordinates, same
    # projection, just newer. The picture underneath is allowed to lag.
    if _LIVE_POS["pct"] and (time.time() - _LIVE_POS["at"]) < LIVE_POS_TTL:
        return {"pct": _LIVE_POS["pct"], "at_dock": at_dock, "source": "live"}
    live = getattr(md, "vacuum_position", None) or getattr(md, "robot_position", None)
    pct = _point_pct(to_pct, live)
    if pct is None:
        # No trace yet on a freshly built map: the dock is where it starts from.
        return {"pct": charger, "at_dock": at_dock, "source": "charger"} if charger else None
    return {"pct": pct, "at_dock": at_dock, "source": "map"}


# A second parser, for position only. The trait's own parser renders the whole
# picture on every parse -- rooms recoloured, path drawn, PNG encoded -- and that
# render is the expensive half of a map refresh. This one is told to draw nothing,
# at scale 1, with no background, walls or room recolouring, so parsing a blob
# costs about what it takes to read the header and the position block. The blob
# still has to be fetched; there is no "where are you" RPC in the v1 protocol,
# the coordinates only exist inside the map.
_POS_PARSER = [None]


def _position_parser() -> typing.Any:
    if _POS_PARSER[0] is None:
        from roborock.map.map_parser import MapParser, MapParserConfig

        _POS_PARSER[0] = MapParser(MapParserConfig(
            drawables=[], show_background=False, show_walls=False,
            show_rooms=False, map_scale=1))
    return _POS_PARSER[0]


# What the device can actually keep up with. A map request is a real round trip
# over the vacuum's own WiFi, and asking for one every second when it takes longer
# than that to answer does not give a faster badge -- it queues commands until they
# hit the 10s timeout, and every other request in the panel waits behind them on
# STATE._lock. So the interval is the configured one OR what the last read cost,
# whichever is slower, and the request itself is given a short deadline so a slow
# one fails fast instead of holding the lock for ten seconds.
_POS_COST = [0.0]
POS_TIMEOUT = 5.0
POS_PACE = 1.6           # leave room for the rest of the tick's work


def _position_gap() -> float:
    return max(WATCH_INTERVAL_ACTIVE, _POS_COST[0] * POS_PACE)


async def _read_position(props: typing.Any) -> bool:
    """Read the robot's position without re-rendering the map. True if it moved.

    Caller holds STATE._lock. Deliberately does not touch map_content: the picture
    the browsers are showing stays exactly as it was, and only the badge moves.
    """
    started = time.monotonic()
    try:
        raw = await props.map_content.rpc_channel.send_command(
            RoborockCommand.GET_MAP_V1, timeout=POS_TIMEOUT)
        _POS_COST[0] = time.monotonic() - started
        if not isinstance(raw, (bytes, bytearray)):
            return False
        parsed = _position_parser().parse(bytes(raw))
        md = getattr(parsed, "map_data", None)
        if md is None:
            return False
        # The same blob carries the route: the browsers get its new points now,
        # rather than at the next full render.
        _note_live_map(md, raw, full=False)
        proj = _map_projector(md)
        if proj is None:
            return False
        live = getattr(md, "vacuum_position", None) or getattr(md, "robot_position", None)
        pct = _point_pct(proj[0], live)
    except Exception as exc:  # noqa: BLE001 -- the slow map refresh is still the floor
        # A failure counts as expensive: whatever is wrong, hammering it at 1s is
        # not going to help, and backing off is what lets the rest of the panel
        # through.
        _POS_COST[0] = max(_POS_COST[0], POS_TIMEOUT)
        _LOGGER.info("position read failed (%s); backing off to %.1fs",
                     exc.__class__.__name__, _position_gap())
        return False
    if pct is None:
        return False
    moved = pct != _LIVE_POS["pct"]
    _LIVE_POS["pct"] = pct
    _LIVE_POS["at"] = time.time()
    return moved


_ROOM_PIXELS_CACHE: dict[int, dict] = {}
_ROOM_MASK_RES = 460  # downsample width for the room pixel analysis / shipped index
# A room is assigned by nearest palette colour (see _room_pixels), so anything on
# the render whose colour happens to sit nearer a room's than to white is tagged
# with that room: anti-aliased edges, the path line, the dock glyph, obstacle
# icons, furniture shading. Those land as a scatter of stray pixels all over the
# floor -- and this index is not cosmetic, the browser resolves every tap and cuts
# every selection highlight from it. So tapping a speckle selected a room on the
# other side of the flat, and selecting a room lit up dots across the whole map.
# Each room is therefore reduced to its real regions: 4-connected components
# below a floor are dropped. The floor is relative to the room's own largest
# region as well as absolute, so a genuine second lobe (a room split by a
# doorway) survives while dust does not.
_SPECKLE_MIN_PX = 10
_SPECKLE_MIN_SHARE = 0.06


def _close_dither(buf: bytearray, sw: int, sh: int) -> int:
    """Fill the gaps a dithered fill leaves in the room assignment.

    Carpet is not drawn as a flat colour: the renderer chequers it, so roughly every
    other pixel over a rug carries the room's own colour and the rest carry the
    carpet's. Nearest-colour assignment therefore produces a CHECKERBOARD over every
    carpeted area -- room id, nothing, room id, nothing.

    That was quietly destructive rather than merely untidy. _despeckle_rooms walks
    4-connected components, and on a checkerboard the room-coloured pixels touch
    only diagonally: each one is a component of ONE pixel, every one falls under the
    floor, and the room is erased wherever it has a rug. A tap on the carpet then
    resolved to no room, the selection highlight had a hole in it, and the room's
    centroid and area were computed from a floorplan with its rugs cut out.

    So the gaps are closed first. A pixel with no id is given one when its
    orthogonal neighbours agree: at least three of them carry an id and all of the
    ones that do are the same room. On a checkerboard that is every gap, in one
    pass. It cannot bleed through a wall -- a pixel between two rooms has neighbours
    that disagree -- and it cannot grow a room outward, because a pixel on the
    room's edge has too few neighbours carrying an id.

    Returns how many pixels were filled.
    """
    n = sw * sh
    src = bytes(buf)                     # decide from the ORIGINAL, so one pass
    filled = 0                           # cannot cascade outward from itself
    for i in range(n):
        if src[i]:
            continue
        x = i % sw
        agree = 0
        rid = 0
        for j, ok in ((i - 1, x > 0), (i + 1, x + 1 < sw),
                      (i - sw, i >= sw), (i + sw, i + sw < n)):
            if not ok:
                continue
            v = src[j]
            if not v:
                continue
            if rid and v != rid:         # two different rooms: this is a boundary
                agree = -1
                break
            rid = v
            agree += 1
        if agree >= 3 and rid:
            buf[i] = rid
            filled += 1
    return filled


def _despeckle_rooms(buf: bytearray, sw: int, sh: int) -> dict[int, list[tuple[int, int, int]]]:
    """Drop each room's stray regions from a flat room-id buffer, in place.

    Returns the surviving RUNS per room id as (y, x0, x1) with x1 exclusive.

    Runs, not pixels, because this is on the cold /api/state path and has to be
    cheap: a room's region is made of horizontal runs of one id, and an index has
    a few thousand of those against a few hundred thousand pixels. Labelling the
    runs and unioning the ones that touch between rows is the same 4-connected
    component analysis at a fraction of the cost -- and the caller's centroid comes
    out of the runs arithmetically, instead of materialising a list of every pixel
    in the flat.
    """
    parent: list[int] = []
    rows: list[tuple[int, int, int, int]] = []      # (rid, y, x0, x1) per label

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]           # halve the path as we go
            a = parent[a]
        return a

    prev: list[tuple[int, int, int, int]] = []      # previous row: (x0, x1, rid, label)
    for y in range(sh):
        base = y * sw
        cur: list[tuple[int, int, int, int]] = []
        x = 0
        while x < sw:
            rid = buf[base + x]
            if not rid:
                x += 1
                continue
            x0 = x
            while x < sw and buf[base + x] == rid:
                x += 1
            label = len(rows)
            parent.append(label)
            rows.append((rid, y, x0, x))
            # Overlapping run of the same room directly above: same component.
            for px0, px1, prid, plabel in prev:
                if prid == rid and px0 < x and px1 > x0:
                    ra, rb = find(plabel), find(label)
                    if ra != rb:
                        parent[rb] = ra
            cur.append((x0, x, rid, label))
        prev = cur

    sizes: dict[int, int] = {}
    for label, (rid, y, x0, x1) in enumerate(rows):
        root = find(label)
        sizes[root] = sizes.get(root, 0) + (x1 - x0)
    biggest: dict[int, int] = {}
    for root, size in sizes.items():
        rid = rows[root][0]
        if size > biggest.get(rid, 0):
            biggest[rid] = size

    kept: dict[int, list[tuple[int, int, int]]] = {}
    for label, (rid, y, x0, x1) in enumerate(rows):
        root = find(label)
        floor = max(_SPECKLE_MIN_PX, int(biggest[rid] * _SPECKLE_MIN_SHARE))
        if sizes[root] >= floor:
            kept.setdefault(rid, []).append((y, x0, x1))
        else:
            base = y * sw
            for x in range(x0, x1):
                buf[base + x] = 0
    return kept


_CLEAN_RENDER_CACHE: dict[int, bytes | None] = {}
_DISPLAY_RENDER_CACHE: dict[int, bytes | None] = {}


# The wake behind the robot, in map cells. A Roborock is ~35cm across and the map
# grid is 5cm a cell, so the chassis is about seven cells wide; the wake is five.
# Deliberately narrower than the footprint: it is claiming the strip the robot
# actually covered, not the full width of the shell, and an over-wide wake spills
# through doorways and over the furniture it drove around -- which would make the
# picture say the floor is clean where it is not.
# Four cells, not five, and a tight blur: what has to stay under the footprint is
# the wake you can SEE, and the blur spreads it about 1.4 sigma either side. Four
# cells (~20cm) plus a 0.5-cell sigma lands around 22px at the default scale 4,
# against a ~28px chassis.
_WAKE_CELLS = 4
_WAKE_BLUR_CELLS = 0.5
_WAKE_ALPHA = 0.42       # a hint under the trace, not a second floor colour
_TRACE_CELLS = 0.8       # the line itself, about what the library draws
# Travel, in map cells: on/off long enough to read as dotted at the map's own scale
# rather than as a slightly broken solid line.
_DASH_ON_CELLS = 0.9
_DASH_OFF_CELLS = 1.1


# While a job runs, the live map carries THAT job's path and that is what belongs
# on screen. Once it ends the live map stops being about it -- the firmware clears
# the trace before the next run and keeps drawing whatever it has -- so the path on
# screen was not the last clean's. The last run's own saved map has it, so it is
# taken from there once and pinned, and drawn over the live floorplan: the same map
# means the same coordinate space, so the live projector places it correctly.
#
# `pts` is the route in the order it was driven, each point (x, y, kind) in map
# coordinates; kind is a bit field: TRACE_TRAVEL (driving, not cleaning) and
# TRACE_BREAK (first point of a new polyline). Ordered, because the browser is fed
# the route incrementally and only ever appends.
_LAST_TRACE: dict[str, typing.Any] = {"begin": None, "pts": [], "goto": [], "stale": False}
TRACE_TRAVEL = 1
TRACE_BREAK = 2
TRACE_HOME = 4      # heading back to the station (to wash the mop, or at the end)

# Per-point flags of the robot's path: map block 18, one byte per path point. The
# parser calls it a mop mask and keeps every non-zero point as "mopped", but on
# this firmware it is a bit field (decoded 2026-10-25 against run maps whose route
# is known): 0x01 mopping, 0x02 heading back (wash trip), 0x04 leaving the dock,
# 0x08 DRIVING, NOT CLEANING. Run 1791651577 opens with 40 points of 0x08/0x0c --
# exactly the drive from the dock to the kitchen -- and every hop between its two
# zones is a short 0x08 run.
_PATH_FLAGS_BLOCK = 18
_PATH_FLAG_TRANSIT = 0x08
_PATH_FLAG_HOMEWARD = 0x02


def _map_blocks(raw: bytes | None) -> dict[int, bytes]:
    """The raw map blob's blocks, by type (the last of each type wins)."""
    if not raw:
        return {}
    import gzip
    import struct
    out: dict[int, bytes] = {}
    try:
        data = bytes(raw)
        if data[:2] == b"\x1f\x8b":
            data = gzip.decompress(data)
        pos = struct.unpack("<H", data[2:4])[0]
        while pos + 8 <= len(data):
            kind, head = struct.unpack("<HH", data[pos:pos + 4])
            size = struct.unpack("<I", data[pos + 4:pos + 8])[0]
            out[kind] = data[pos:pos + head + size]
            pos += head + size
    except Exception:  # noqa: BLE001 -- whatever was read before the fault stands
        _LOGGER.debug("map blocks unreadable", exc_info=True)
    return out


def _floor_digest(raw: bytes | None) -> str:
    """Identity of the floor picture: the charger and image blocks only.

    The picture /api/map.png serves no longer carries the path or the robot, so it
    only changes when the floor plan (or the dock) does -- not on every map read
    of a run, which is what made each update a whole new picture.
    """
    blocks = _map_blocks(raw)
    floor = blocks.get(2)
    if floor is None:
        return hashlib.blake2b(bytes(raw), digest_size=8).hexdigest() if raw else ""
    h = hashlib.blake2b(digest_size=8)
    h.update(blocks.get(1, b""))
    h.update(floor)
    return h.hexdigest()


def _trace_of(map_data: typing.Any, raw: bytes | None = None) -> dict:
    """The routes in a parsed map, as plain ordered points (see _LAST_TRACE).

    Travel is split off the path by the per-point flags when the raw blob is at
    hand and they line up with the points; without them everything is cleaning,
    as before. `goto` is the map's planned routes, which are always travel.
    """
    def poly(attr: str) -> list[list[tuple[float, float]]]:
        out = []
        for seg in (getattr(getattr(map_data, attr, None), "path", None) or []):
            pts = [(float(p.x), float(p.y)) for p in seg]
            if len(pts) >= 2:
                out.append(pts)
        return out

    path = poly("path")
    block = _map_blocks(raw).get(_PATH_FLAGS_BLOCK)
    flags = None
    if block:
        import struct
        head = struct.unpack("<H", block[2:4])[0]
        flags = block[head:]
        if len(flags) != sum(len(seg) for seg in path):
            flags = None
    pts: list[tuple[float, float, int]] = []
    i = 0
    for seg in path:
        for j, (x, y) in enumerate(seg):
            kind = TRACE_BREAK if j == 0 else 0
            if flags is not None and flags[i] & _PATH_FLAG_TRANSIT:
                kind |= TRACE_TRAVEL
            if flags is not None and flags[i] & _PATH_FLAG_HOMEWARD:
                kind |= TRACE_HOME
            pts.append((x, y, kind))
            i += 1
    return {"pts": pts, "goto": poly("goto_path") + poly("predicted_path")}


def _pin_trace(begin: int, map_data: typing.Any, raw: bytes | None = None) -> None:
    """Remember a finished run's path as the one to draw while nothing is running."""
    trace = _trace_of(map_data, raw)
    if not trace["pts"]:
        return
    _LAST_TRACE.update(begin=int(begin), stale=False, **trace)
    _hcache_touch()
    _kick_trace()
    _LOGGER.info("map trace pinned to run %s (%d point(s), %d driving)", begin,
                 len(trace["pts"]), sum(1 for p in trace["pts"] if p[2] & TRACE_TRAVEL))


def _render_trace() -> dict | None:
    """The trace the map should show: None means "the live map's own".

    Reads the status trait, which the MQTT push keeps current, so this costs
    nothing and is right the moment a run starts or stops.
    """
    device = STATE.device()
    props = getattr(device, "v1_properties", None) if device is not None else None
    if props is not None:
        st = props.status
        if _is_running(_int_or_none(st.state), st.in_cleaning):
            return None
    # Superseded: a newer run has started since this was pinned, so the pinned one
    # is the run BEFORE the one you just watched. This is the gap that showed the
    # wrong map on the way back to the dock -- the robot stops counting as running
    # the moment it finishes cleaning, but its own map is not read back for another
    # minute, and in between the pinned trace was a run older than the live map's.
    # The live map's path is this run's, so it stays on screen until superseded by
    # its own.
    if _LAST_TRACE.get("stale"):
        return None
    return _LAST_TRACE if _LAST_TRACE["pts"] else None


# --------------------------------------------------------------------------- #
# The route, as vectors for the browser to draw.
#
# It used to be baked into /api/map.png, so every update of a running job was a
# whole new picture (~58 KB) and was only re-rendered every few seconds. Now the
# picture is the floor alone (see _display_render) and the route goes over the
# socket as points in the picture's own pixel space: the full set once (GET
# /api/trace), then only the points added since, every time the sampler reads the
# map -- about once a second while someone watches a job, a few bytes each.
# `md`/`raw`: the newest read of the live map, whichever made it -- the route's
# points come from here. `proj`: the last FULL parse, which is what the floor
# picture was drawn from; the 1s position reads parse at scale 1, so their own
# projection would land the route a quarter of the way across the picture.
_LIVE_MAP: dict[str, typing.Any] = {"md": None, "raw": None, "proj": None, "proj_raw": None}
_TRACE_SENT: dict[str, typing.Any] = {"id": None, "n": 0, "goto": None}


def _note_live_map(map_data: typing.Any, raw: bytes | None, full: bool = True) -> None:
    """Record a fresh read of the live map and tell the browsers what it added."""
    if map_data is None or not raw:
        return
    _LIVE_MAP["md"], _LIVE_MAP["raw"] = map_data, bytes(raw)
    if full:
        _LIVE_MAP["proj"], _LIVE_MAP["proj_raw"] = map_data, bytes(raw)
    _kick_trace()


def _kick_trace() -> None:
    """push_trace from sync code (or with STATE._lock held): it never needs the lock."""
    try:
        asyncio.get_running_loop().create_task(push_trace())
    except RuntimeError:
        pass


def _trace_view() -> dict | None:
    """The route to show now, projected into the floor picture's pixels."""
    md, raw = _LIVE_MAP["md"], _LIVE_MAP["raw"]
    if md is None:
        return None
    pinned = _render_trace()
    src = pinned if pinned is not None else _trace_of(md, raw)
    view = _project_trace(src)
    if view is not None:
        # Changes when the route is a different one: a pinned run, or a live run
        # whose first point moved (the firmware starts a fresh path per job).
        view["id"] = f"{'run' + str(pinned.get('begin')) if pinned else 'live'}:{view['pts'][:2]}"
    return view


def _project_trace(src: dict) -> dict | None:
    """A trace (map coordinates) as the browser draws it: floor-picture pixels.

    Always through the LIVE map's projection, whichever run the route is from:
    map coordinates are the robot's own, so an old run's route lands where it
    happened on today's floor picture -- the picture that is on screen.
    """
    if _LIVE_MAP["proj"] is None:
        return None
    proj = _map_projector(_LIVE_MAP["proj"])
    if proj is None:
        return None
    to_pct, img_w, img_h = proj

    def px(x: float, y: float) -> tuple[int, int]:
        left, top = to_pct(x, y)
        return round(left / 100.0 * img_w), round(top / 100.0 * img_h)

    flat: list[int] = []
    for x, y, kind in src["pts"]:
        flat.extend((*px(x, y), kind))
    goto = [[c for pt in seg for c in px(*pt)] for seg in src["goto"]]
    dims = getattr(getattr(_LIVE_MAP["proj"], "image", None), "dimensions", None)
    scale = max(1, int(getattr(dims, "scale", 1) or 1))
    return {
        "id": None, "n": len(flat) // 3, "pts": flat, "goto": goto,
        "w": img_w, "h": img_h,
        # The grid is 5 cm a cell and the picture `scale` pixels a cell: what one
        # pixel of the floor picture is on the floor (the choreography simulator
        # turns speeds into distances with it).
        "mm_px": 50.0 / scale,
        "style": {"wake": max(3, _WAKE_CELLS * scale), "blur": max(0.6, scale * _WAKE_BLUR_CELLS),
                  "wake_alpha": _WAKE_ALPHA, "line": max(1, round(_TRACE_CELLS * scale)),
                  "dash": [max(2.0, scale * _DASH_ON_CELLS), max(2.0, scale * _DASH_OFF_CELLS)],
                  "color": _path_color_hex()},
    }


def _path_color_hex() -> str:
    try:
        from vacuum_map_parser_base.config.color import ColorsPalette, SupportedColor
        r, g, b = tuple(ColorsPalette().get_color(SupportedColor.PATH))[:3]
        return f"#{r:02x}{g:02x}{b:02x}"
    except Exception:  # noqa: BLE001
        return "#93c2ee"


async def push_trace() -> None:
    """Send the browsers what the route gained since the last frame.

    Append-only while it is the same route: `from` is how many points the browser
    must already hold, and one that holds a different number (it just connected,
    or missed a frame) fetches the whole set from /api/trace instead.
    """
    view = _trace_view()
    if view is None:
        return
    sent = _TRACE_SENT
    same = view["id"] == sent["id"] and view["n"] >= sent["n"]
    start = sent["n"] if same else 0
    # Sent on every read even when nothing was added: `at` is how the page knows
    # the route is CURRENT rather than merely unchanged (a parked robot draws no
    # new points), and it is a few dozen bytes.
    frame = {"id": view["id"], "from": start, "n": view["n"],
             "pts": view["pts"][start * 3:], "goto": view["goto"], "at": round(time.time(), 1)}
    if not same:
        frame["reset"] = True       # the browser re-fetches the whole set
        frame["pts"] = []
    sent.update(id=view["id"], n=view["n"], goto=view["goto"])
    await HUB.send("trace", frame)


async def handle_trace(request: web.Request) -> web.Response:
    """The whole route to draw over the floor picture. ?fresh=1 re-reads the map
    first (shared TTL with the picture, so a burst of tabs costs one read)."""
    if request.query.get("fresh") == "1":
        async with STATE._lock:
            props = await STATE.props()
            if time.time() - _MAP_READ_AT[0] > MAP_TTL_FRESH:
                _drop_map_ttl()
                await _refresh_map(props)
    view = _trace_view() or {"id": None, "n": 0, "pts": [], "goto": []}
    view["at"] = _MAP_READ_AT[0] or None
    resp = web.json_response(view, headers={"Cache-Control": "no-store"})
    resp.enable_compression()
    return resp


# Past runs' routes, for the replay in the history sheet: map coordinates, kept
# for the last few so replaying again costs nothing (a run map is ~460 KB).
_RUN_TRACE_CACHE: dict[int, dict] = {}
_RUN_TRACE_KEEP = 8


async def handle_run_trace(request: web.Request) -> web.Response:
    """One finished run's route, in the order it was driven, for replaying."""
    try:
        begin = int(request.query["begin"])
    except (KeyError, ValueError):
        raise web.HTTPBadRequest(reason="begin required")
    src = _RUN_TRACE_CACHE.get(begin)
    async with STATE._lock:
        props = await STATE.props()
        if _LIVE_MAP["proj"] is None:            # nothing to project onto yet
            await _refresh_map(props)
        if src is None:
            try:
                raw = await props.map_content.rpc_channel.send_command(
                    RoborockCommand.GET_CLEAN_RECORD_MAP, params={"start_time": begin})
                content = props.map_content.converter.parse_map_content(bytes(raw))
            except Exception as exc:  # noqa: BLE001
                raise web.HTTPBadGateway(reason=f"carte du nettoyage indisponible : {exc}") from exc
            src = _trace_of(content.map_data, bytes(raw))
            _RUN_TRACE_CACHE[begin] = src
            for stale in list(_RUN_TRACE_CACHE)[:-_RUN_TRACE_KEEP]:
                del _RUN_TRACE_CACHE[stale]
    view = _project_trace(src)
    if view is None or not view["n"]:
        raise web.HTTPNotFound(reason="aucun trajet pour ce nettoyage")
    view["id"] = f"run{begin}"
    view["bands"] = _route_bands(view)
    resp = web.json_response(view)
    resp.enable_compression()
    return resp


def _route_bands(view: dict) -> dict:
    """What the replay's scrub bar shows behind it, as point ranges.

    rooms: [start, end, room id] -- the room under each point, read off the same
    per-pixel room map the taps use. A point on a wall or a doorway sill has no
    room and keeps the previous one, and runs of a few points are folded into
    their neighbour, so the bar shows rooms, not the jitter along their borders.
    home: [start, end] -- stretches heading back to the station.
    colors: {room id: "#rrggbb"}, the map's own room colours.
    """
    out: dict[str, typing.Any] = {"rooms": [], "home": [], "colors": {}}
    pts, n = view["pts"], view["n"]
    md, raw = _LIVE_MAP["proj"], _LIVE_MAP["proj_raw"]
    rooms = getattr(md, "rooms", None) if md is not None else None
    if rooms:
        try:
            from vacuum_map_parser_base.config.color import ColorsPalette

            room_ids = [int(r) for r in rooms.keys()]
            clean = _clean_room_render(raw)
            rp = _room_pixels(clean, room_ids) if clean else {"assign": None}
            assign, (sw, sh) = rp["assign"], rp.get("size", (0, 0))
            if assign is not None and sw and sh:
                ap = assign.load()
                fx, fy = sw / view["w"], sh / view["h"]
                per: list[int] = []
                last = 0
                for i in range(n):
                    x = min(sw - 1, max(0, int(pts[3 * i] * fx)))
                    y = min(sh - 1, max(0, int(pts[3 * i + 1] * fy)))
                    rid = ap[x, y] or last
                    per.append(rid)
                    last = rid
                runs: list[list[int]] = []
                for i, rid in enumerate(per):
                    if runs and runs[-1][2] == rid:
                        runs[-1][1] = i
                    else:
                        runs.append([i, i, rid])
                # Fold blips (shorter than ~0.5% of the route, at least 4 points)
                # into the run before them.
                tiny = max(4, n // 200)
                merged: list[list[int]] = []
                for r in runs:
                    if merged and (r[1] - r[0] + 1 < tiny or merged[-1][2] == r[2]):
                        merged[-1][1] = r[1]
                    else:
                        merged.append(r)
                out["rooms"] = [r for r in merged if r[2]]
                palette = ColorsPalette()
                out["colors"] = {str(r): "#%02x%02x%02x" % tuple(palette.get_room_color(r))[:3]
                                 for r in room_ids}
        except Exception:  # noqa: BLE001 -- the bar works without colours
            _LOGGER.debug("route bands unavailable", exc_info=True)
    home: list[list[int]] = []
    for i in range(n):
        if pts[3 * i + 2] & TRACE_HOME:
            if home and i - home[-1][1] <= 3:
                home[-1][1] = i
            else:
                home.append([i, i])
    out["home"] = home
    return out


def _display_render(raw_api_response: bytes | None) -> bytes | None:
    """The floor as displayed: rooms, walls and the dock -- no robot, no route.

    Both of those are drawn by the browser over it now, so this picture changes
    only when the floor does, and is cached on the floor's own digest. Encoded as
    lossless WebP: the render is a handful of flat colours upscaled 4x, which WebP
    stores in ~4 KB where the PNG with the route baked in was ~58 KB.
    """
    if not raw_api_response:
        return None
    key = hash((_floor_digest(raw_api_response), _RENDER_REV))
    if key in _DISPLAY_RENDER_CACHE:
        return _DISPLAY_RENDER_CACHE[key]
    out = None
    try:
        import io

        from PIL import Image
        from vacuum_map_parser_base.config.drawable import Drawable

        from roborock.map.map_parser import MapParser, MapParserConfig

        parsed = MapParser(MapParserConfig(drawables=[Drawable.CHARGER])).parse(
            bytes(raw_api_response))
        if parsed and parsed.image_content:
            buf = io.BytesIO()
            Image.open(io.BytesIO(parsed.image_content)).convert("RGBA").save(
                buf, format="WEBP", lossless=True, quality=100, method=6)
            out = buf.getvalue()
    except Exception:  # noqa: BLE001
        _LOGGER.debug("display render failed", exc_info=True)
        out = None
    _DISPLAY_RENDER_CACHE.clear()
    _DISPLAY_RENDER_CACHE[key] = out
    return out


def _clean_room_render(raw_api_response: bytes | None) -> bytes | None:
    """Re-render the map with ONLY room fills — no vacuum path/position, charger or
    walls — so colour-based room segmentation isn't polluted by the vacuum's trace
    (which is the same colour as a room) or by walls. Same geometry as the display
    map, so masks/centroids computed on it still align. Cached per map."""
    if not raw_api_response:
        return None
    key = len(raw_api_response)
    if key in _CLEAN_RENDER_CACHE:
        return _CLEAN_RENDER_CACHE[key]
    out = None
    try:
        from roborock.map.map_parser import MapParser, MapParserConfig

        parsed = MapParser(MapParserConfig(drawables=[], show_walls=False, show_rooms=True)).parse(bytes(raw_api_response))
        out = parsed.image_content if parsed else None
    except Exception:  # noqa: BLE001
        out = None
    _CLEAN_RENDER_CACHE.clear()
    _CLEAN_RENDER_CACHE[key] = out
    return out


def _room_pixels(image_content: bytes | None, room_ids: list[int]) -> dict:
    """Analyse the rendered map ONCE: assign each pixel to its room (by nearest
    palette colour) on a downsampled copy. Returns {centroids, assign, size}:
      - centroids: {seg: (left%, top%)} snapped to a real room pixel (always
        inside — fixes L-shaped rooms whose bbox centre is outside).
      - assign: PIL 'L' image (downsampled) whose pixel value is the room id (0=none),
        used to build shape-following selection masks.
    Each room is despeckled before either is derived -- see _despeckle_rooms for
    why nearest-colour assignment needs it.
    Cached per map. Returns empty on any problem (callers fall back)."""
    empty = {"centroids": {}, "assign": None, "size": (0, 0)}
    if not image_content or not room_ids:
        return empty
    key = hash((len(image_content), tuple(sorted(room_ids))))
    if key in _ROOM_PIXELS_CACHE:
        return _ROOM_PIXELS_CACHE[key]
    result = empty
    try:
        import io
        from PIL import Image
        from vacuum_map_parser_base.config.color import ColorsPalette

        palette = ColorsPalette()
        colors = [(rid, tuple(palette.get_room_color(rid))[:3]) for rid in room_ids]
        img = Image.open(io.BytesIO(image_content)).convert("RGB")
        w, h = img.size
        step = max(1, w // _ROOM_MASK_RES)
        sw, sh = max(1, w // step), max(1, h // step)
        small = img.resize((sw, sh), Image.NEAREST)
        # One nearest-colour decision per DISTINCT colour, not per pixel. The
        # render uses a fixed palette -- one colour per room plus floor, walls and
        # the path trace -- so this is a few dozen decisions memoised in a dict
        # rather than a few hundred thousand nested loops. On the cold /api/state
        # path that is the difference between seconds and a moment, and the result
        # is identical: the decision only ever depended on the colour.
        # A flat buffer rather than PIL pixel access because the despeckle pass
        # reads it back by index; the image is rebuilt from it at the end.
        # tobytes(), not getdata(): getdata is deprecated for removal in Pillow 14,
        # and reading the raw RGB triples is faster than materialising a tuple per
        # pixel anyway.
        lut: dict[bytes, int] = {}
        data = small.tobytes()
        buf = bytearray(sw * sh)
        for i in range(sw * sh):
            rgb = data[i * 3:i * 3 + 3]
            rid = lut.get(rgb)
            if rid is None:
                r, g, b = rgb[0], rgb[1], rgb[2]
                best, bestd = 0, 1 << 30
                for cand, (cr, cg, cb) in colors:
                    d = (r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2
                    if d < bestd:
                        bestd, best = d, cand
                dw = (r - 255) ** 2 + (g - 255) ** 2 + (b - 255) ** 2  # dist to white bg
                rid = best if (best and bestd < dw and bestd < 3000) else 0
                lut[rgb] = rid
            if rid:
                buf[i] = rid
        dithered = _close_dither(buf, sw, sh)
        if dithered:
            _LOGGER.info("room index: closed %d dithered pixel(s) (carpet is chequered, "
                         "and the gaps used to be despeckled away)", dithered)
        kept = _despeckle_rooms(buf, sw, sh)
        assign = Image.frombytes("L", (sw, sh), bytes(buf))
        # Centroids come from the CLEANED runs: a speckle left in would have been a
        # candidate for the snap below, which is how a room's bubble ends up in the
        # middle of somebody else's floor. Sums come out of the runs arithmetically
        # (a run of length n starting at x0 contributes n*x0 + n(n-1)/2), so no list
        # of every pixel in the flat is ever built.
        cents: dict[int, tuple] = {}
        for rid, runs in kept.items():
            count = sum(x1 - x0 for _y, x0, x1 in runs)
            if count < 3:
                continue
            sx = sum(n * x0 + n * (n - 1) // 2 for _y, x0, x1 in runs for n in (x1 - x0,))
            sy = sum(y * (x1 - x0) for y, x0, x1 in runs)
            mx, my = sx / count, sy / count
            # Nearest real room pixel to the centre of mass -- an L-shaped room's
            # centre of mass is often outside the room. Each run's closest point is
            # its clamped x on its own row, so this stays O(runs).
            best_pt, best_d = None, None
            for y, x0, x1 in runs:
                cx = min(max(mx, x0), x1 - 1)
                d = (cx - mx) ** 2 + (y - my) ** 2
                if best_d is None or d < best_d:
                    best_d, best_pt = d, (int(round(cx)), y)
            nx, ny = best_pt
            cents[rid] = (100.0 * (nx + 0.5) / sw, 100.0 * (ny + 0.5) / sh)
        result = {"centroids": cents, "assign": assign, "size": (sw, sh)}
    except Exception:  # noqa: BLE001
        result = empty
    _ROOM_PIXELS_CACHE.clear()  # only keep the latest map's analysis
    _ROOM_PIXELS_CACHE[key] = result
    return result


def _room_overlays(map_data: typing.Any, image_content: bytes | None = None) -> list[dict[str, typing.Any]]:
    overlays: list[dict[str, typing.Any]] = []
    if map_data is None or getattr(map_data, "rooms", None) is None:
        return overlays
    proj = _map_projector(map_data)
    if proj is None:
        return overlays
    to_pct, _, _ = proj
    centroids = _room_pixels(image_content, [int(s) for s in map_data.rooms.keys()])["centroids"]
    for seg_id, room in map_data.rooms.items():
        if int(seg_id) in centroids:
            left, top = centroids[int(seg_id)]
            overlays.append({"segment_id": int(seg_id), "left_pct": left, "top_pct": top})
            continue
        cx = room.pos_x if room.pos_x is not None else (room.x0 + room.x1) / 2
        cy = room.pos_y if room.pos_y is not None else (room.y0 + room.y1) / 2
        try:
            left, top = to_pct(cx, cy)
            overlays.append({"segment_id": int(seg_id), "left_pct": left, "top_pct": top})
        except Exception:  # noqa: BLE001
            continue
    return overlays


def _room_bboxes(map_data: typing.Any) -> dict[int, list]:
    """Per-room bounding box as [x%, y%, w%, h%] over the rendered PNG, so the
    whole room area is clickable on the map (not just the number marker)."""
    out: dict[int, list] = {}
    if map_data is None or getattr(map_data, "rooms", None) is None:
        return out
    proj = _map_projector(map_data)
    if proj is None:
        return out
    to_pct, _, _ = proj
    for seg_id, room in map_data.rooms.items():
        try:
            a, b = to_pct(room.x0, room.y0), to_pct(room.x1, room.y1)
            left, top = min(a[0], b[0]), min(a[1], b[1])
            right, bottom = max(a[0], b[0]), max(a[1], b[1])
            out[int(seg_id)] = [left, top, right - left, bottom - top]
        except Exception:  # noqa: BLE001
            continue
    return out


_KNOWN_BLOCK_TYPES = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 1024}


def _debug_blocks(map_data: typing.Any, raw: bytes | None) -> list[dict]:
    """Decode UNKNOWN map blocks that contain coordinate-like data and return them
    as labelled overlays (image %) so unidentified features (thresholds, carpet,
    etc.) can be seen on the editor. Best-effort geometric interpretation."""
    proj = _map_projector(map_data)
    if proj is None or not raw:
        return []
    to_pct, _, _ = proj
    raw = bytes(raw)
    u16 = lambda o: int.from_bytes(raw[o:o + 2], "little")  # noqa: E731
    u32 = lambda o: int.from_bytes(raw[o:o + 4], "little")  # noqa: E731
    LO, HI = 6000, 46000  # plausible map-coordinate range (frame centred ~25500)
    inrange = lambda v: LO <= v <= HI  # noqa: E731

    def pt(x, y):
        try:
            p = to_pct(x, y)
            return [round(p[0], 2), round(p[1], 2)]
        except Exception:  # noqa: BLE001
            return None

    out: list[dict] = []
    try:
        pos = u16(0x02)
        while pos < len(raw) - 6:
            bt = u16(pos)
            bhl = u16(pos + 0x02)
            bdl = u32(pos + 0x04)
            if bhl == 0 or pos + bhl + bdl > len(raw):
                break
            if bt not in _KNOWN_BLOCK_TYPES and bdl >= 4:
                ds = pos + bhl
                cnt = u16(pos + 0x08) if bhl >= 10 else 0
                vals = [u16(ds + i) for i in range(0, bdl, 2)]
                shapes: list = []
                kind = "point"
                stride = (bdl // cnt) if cnt else 0
                if cnt and stride >= 16 and stride % 2 == 0:  # quads (+optional trailing meta)
                    kind = "poly"
                    for i in range(min(cnt, bdl // 16)):  # quads are 16 bytes; ignore trailing meta
                        b = i * 8
                        corners = [(vals[b + j * 2], vals[b + j * 2 + 1]) for j in range(4)]
                        if all(inrange(cx) and inrange(cy) for cx, cy in corners):
                            pcs = [pt(cx, cy) for cx, cy in corners]
                            if all(pcs):
                                shapes.append(pcs)
                elif cnt and stride == 8:  # lines
                    kind = "line"
                    for i in range(cnt):
                        b = i * 4
                        if all(inrange(vals[b + k]) for k in range(4)):
                            a, c = pt(vals[b], vals[b + 1]), pt(vals[b + 2], vals[b + 3])
                            if a and c:
                                shapes.append([a[0], a[1], c[0], c[1]])
                elif cnt and stride in (4, 6):  # points (x,y[,flag])
                    st = stride // 2
                    for i in range(cnt):
                        b = i * st
                        if inrange(vals[b]) and inrange(vals[b + 1]):
                            p = pt(vals[b], vals[b + 1])
                            if p:
                                shapes.append(p)
                else:  # fallback: any consecutive (x,y) both in range
                    for i in range(0, len(vals) - 1, 2):
                        if inrange(vals[i]) and inrange(vals[i + 1]):
                            p = pt(vals[i], vals[i + 1])
                            if p:
                                shapes.append(p)
                if shapes:
                    out.append({"type": bt, "kind": kind, "count": cnt, "shapes": shapes})
            pos += bhl + bdl
    except Exception:  # noqa: BLE001
        return out
    return out


def _block_quad_overlays(map_data: typing.Any, raw: bytes | None, block_type: int) -> list:
    """Quads of a specific map block type (e.g. 28=thresholds, 39=carpet) as
    image-% polygons ([[x%,y%]×4])."""
    proj = _map_projector(map_data)
    if proj is None or not raw:
        return []
    to_pct, _, _ = proj
    raw = bytes(raw)
    u16 = lambda o: int.from_bytes(raw[o:o + 2], "little")  # noqa: E731
    u32 = lambda o: int.from_bytes(raw[o:o + 4], "little")  # noqa: E731
    out: list = []
    try:
        pos = u16(0x02)
        while pos < len(raw) - 6:
            bt = u16(pos)
            bhl = u16(pos + 0x02)
            bdl = u32(pos + 0x04)
            if bhl == 0 or pos + bhl + bdl > len(raw):
                break
            if bt == block_type and bhl >= 10:
                ds = pos + bhl
                cnt = u16(pos + 0x08)
                # quads are always 16 bytes (8 int16); any extra bytes are trailing metadata.
                nq = min(cnt, bdl // 16)
                for i in range(nq):
                    v = [u16(ds + i * 16 + j * 2) for j in range(8)]
                    corners = [(v[0], v[1]), (v[2], v[3]), (v[4], v[5]), (v[6], v[7])]
                    if all(6000 <= cx <= 46000 and 6000 <= cy <= 46000 for cx, cy in corners):
                        out.append([[round(p[0], 2), round(p[1], 2)] for p in (to_pct(x, y) for x, y in corners)])
            pos += bhl + bdl
    except Exception:  # noqa: BLE001
        return out
    return out


def _barrier_overlays(map_data: typing.Any) -> dict[str, list]:
    """Existing map barriers as percentage geometry (read-only overlays).

    walls / no_go / no_mop / zones come straight from the parsed map. Lines are
    [x0%,y0%,x1%,y1%]; areas/zones are lists of [x%,y%] polygon vertices.
    """
    out: dict[str, list] = {"walls": [], "no_go": [], "no_mop": [], "zones": []}
    proj = _map_projector(map_data)
    if proj is None:
        return out
    to_pct, _, _ = proj

    def line(obj):
        a, b = to_pct(obj.x0, obj.y0), to_pct(obj.x1, obj.y1)
        return [a[0], a[1], b[0], b[1]]

    def area(obj):  # Area has x0..x3,y0..y3 (quad); Zone has x0,y0,x1,y1 (rect)
        if hasattr(obj, "x2"):
            pts = [(obj.x0, obj.y0), (obj.x1, obj.y1), (obj.x2, obj.y2), (obj.x3, obj.y3)]
        else:
            pts = [(obj.x0, obj.y0), (obj.x1, obj.y0), (obj.x1, obj.y1), (obj.x0, obj.y1)]
        return [list(to_pct(x, y)) for x, y in pts]

    for src, key, fn in (
        (getattr(map_data, "walls", None), "walls", line),
        (getattr(map_data, "no_go_areas", None), "no_go", area),
        (getattr(map_data, "no_mopping_areas", None), "no_mop", area),
        (getattr(map_data, "zones", None), "zones", area),
    ):
        for obj in (src or []):
            try:
                out[key].append(fn(obj))
            except Exception:  # noqa: BLE001
                continue
    return out


def _image_to_vacuum(map_data: typing.Any):
    """Return f(px, py) -> (vac_x, vac_y) derived from the map calibration, or None.

    Zone/goto commands use raw vacuum coordinates; the browser gives image
    pixels. Calibration provides 3 (vacuum, image) point pairs → affine solve.
    """
    try:
        cal = map_data.calibration()
    except Exception:  # noqa: BLE001
        return None
    if not cal or len(cal) < 3:
        return None
    v = [(c["vacuum"]["x"], c["vacuum"]["y"]) for c in cal]
    m = [(c["map"]["x"], c["map"]["y"]) for c in cal]
    # Solve v = M*m + t from 3 pairs.
    dm1 = (m[1][0] - m[0][0], m[1][1] - m[0][1])
    dm2 = (m[2][0] - m[0][0], m[2][1] - m[0][1])
    dv1 = (v[1][0] - v[0][0], v[1][1] - v[0][1])
    dv2 = (v[2][0] - v[0][0], v[2][1] - v[0][1])
    det = dm1[0] * dm2[1] - dm1[1] * dm2[0]
    if not det:
        return None
    # inverse of [dm1 dm2] (columns)
    inv = ((dm2[1] / det, -dm2[0] / det), (-dm1[1] / det, dm1[0] / det))
    # M = [dv1 dv2] * inv  (columns dv1, dv2)
    m00 = dv1[0] * inv[0][0] + dv2[0] * inv[1][0]
    m01 = dv1[0] * inv[0][1] + dv2[0] * inv[1][1]
    m10 = dv1[1] * inv[0][0] + dv2[1] * inv[1][0]
    m11 = dv1[1] * inv[0][1] + dv2[1] * inv[1][1]
    t0 = v[0][0] - (m00 * m[0][0] + m01 * m[0][1])
    t1 = v[0][1] - (m10 * m[0][0] + m11 * m[0][1])

    def f(px: float, py: float) -> tuple[int, int]:
        vx = round(m00 * px + m01 * py + t0)
        vy = round(m10 * px + m11 * py + t1)
        if FLIP_Y:
            vy = 51200 - vy
        return (vx, vy)

    return f


def _enum_label(enum_cls, value):
    try:
        return enum_cls(value).name
    except Exception:  # noqa: BLE001
        return str(value)


# --------------------------------------------------------------------------- #
# Read endpoints
# --------------------------------------------------------------------------- #
async def handle_index(_request: web.Request) -> web.Response:
    """The whole panel, and it must never be served stale.

    FileResponse sets ETag and Last-Modified but no Cache-Control, and with no
    Cache-Control a browser is free to apply its own heuristic: it caches the page
    and reuses it WITHOUT revalidating, for a fraction of the age of the file. On a
    phone that is minutes to hours -- so a deploy landed, the panel was restarted,
    and the browser went on showing the previous build. Every UI fix looked like it
    had not been made.

    `no-cache` does not mean "do not store", it means "always revalidate", and the
    ETag is already there to answer with a 304 -- so the cost is one conditional
    request, not 420KB. The static assets keep their own caching; they are the
    things that genuinely do not change.
    """
    resp = web.FileResponse(HERE / "index.html")
    resp.headers["Cache-Control"] = "no-cache"
    return resp


def _int_or_none(v: typing.Any) -> int | None:
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


# Deciding "was this room cleaned" from the path trace.
#
# The raw number is the fraction of a room's cells the path passes through. That
# is NOT ~1.0 for a cleaned room: the trace is a set of parallel lines with gaps
# between passes, so a fully cleaned room typically lands somewhere around
# 0.2-0.4 depending on map resolution and how tightly the robot tracks. An
# absolute cutoff would therefore be a resolution-dependent guess.
#
# So the test is relative: compare each room against the best-covered room in
# the same run, which is by definition a room the robot did clean. A room with
# less than COVERAGE_REL of that was not cleaned. COVERAGE_FLOOR separately
# rejects rooms the robot merely drove across on its way somewhere else (a
# single transit line touches well under 1% of a room's cells).
# The two failure modes are not equally bad: re-cleaning a room that was already
# done costs a few minutes, while skipping a room leaves it dirty and defeats the
# point of the resume. So COVERAGE_REL sits above half deliberately -- a room the
# robot got only part-way through is treated as unfinished.
# A run is under way in these; anything the previous run left behind is no
# longer something you can go back to.
_CLEANING_STATES = {
    RoborockStateCode.starting.value,
    RoborockStateCode.cleaning.value,
    RoborockStateCode.segment_cleaning.value,
    RoborockStateCode.zoned_cleaning.value,
    RoborockStateCode.spot_cleaning.value,
    # The 63xx family is how this generation reports the same thing while the
    # mop is down; the base enum above is the 2016 table and does not cover it.
    RoborockStateCode.robot_status_mopping.value,
    RoborockStateCode.clean_mop_cleaning.value,
    RoborockStateCode.clean_mop_mopping.value,
    RoborockStateCode.segment_mopping.value,
    RoborockStateCode.segment_clean_mop_cleaning.value,
    RoborockStateCode.segment_clean_mop_mopping.value,
    RoborockStateCode.zoned_mopping.value,
}
# ...and in these the robot is finishing up rather than starting out, so a
# lingering in_cleaning flag must not be read as "a new job is running". This is
# the ordinary way an interrupted run ends: it gives up and takes itself home,
# with in_cleaning still set on the way.
_NOT_RUNNING_STATES = {
    RoborockStateCode.returning_home.value,
    RoborockStateCode.charging.value,
    RoborockStateCode.charging_complete.value,
    RoborockStateCode.idle.value,
    RoborockStateCode.paused.value,
}

def _is_running(state: int | None, in_cleaning: typing.Any) -> bool:
    """Is a cleaning run actually under way right now?

    `in_cleaning` alone is not enough: it stays set while the robot drives home,
    which is how an interrupted run ends rather than a sign that a new one has
    begun. The state alone is not enough either -- this model reports the 63xx
    family the base enum has never heard of. So: either the state says cleaning,
    or the flag says cleaning and the state is not plainly winding down.
    """
    if state in _CLEANING_STATES:
        return True
    return bool(in_cleaning) and state not in _NOT_RUNNING_STATES


COVERAGE_REL = 0.6
COVERAGE_FLOOR = 0.03
_COVERAGE_CACHE: dict[int, dict] = {}
_ROOM_TOTALS_CACHE: dict[int, dict] = {}

# What the last run's own event log says is still outstanding, keyed by its
# `begin`. Filled by /api/state (which can do RPCs) and read by _resume_plan
# (which cannot) -- the same arrangement _LAST_RUN_CACHE uses.
_RUN_TODO_CACHE: dict[int, dict] = {}


def _coverage(props: typing.Any) -> dict | None:
    """Per-room coverage of the CURRENT run, read off the vacuum's path trace.

    The map carries the path the robot has actually driven this run, so which
    rooms did and did not get cleaned is recoverable from the map alone -- no
    job record needed. This is what makes resume work for a run that was
    already in progress before any of this tracking existed.

    Returns {seg: fraction} or None if the map has no usable path.
    """
    try:
        content = props.map_content
        md = content.map_data
        path_obj = getattr(md, "path", None)
        subpaths = getattr(path_obj, "path", None) if path_obj is not None else None
        if not subpaths:
            return None
        proj = _map_projector(md)
        if proj is None:
            return None
        to_pct, _w, _h = proj
        room_ids = [int(s) for s in md.rooms.keys()] if getattr(md, "rooms", None) else []
        if not room_ids:
            return None
        clean = _clean_room_render(content.raw_api_response) or content.image_content
        rp = _room_pixels(clean, room_ids)
        assign, (sw, sh) = rp["assign"], rp["size"]
        if assign is None or not sw or not sh:
            return None
        ap = assign.load()

        # Both loops below are ~200k cell reads, and /api/status is polled every
        # second, so memoise on the raw map bytes: identical map -> identical
        # answer, and the map only changes when the robot moves.
        raw = content.raw_api_response or b""
        ck = hash(raw)
        if ck in _COVERAGE_CACHE:
            return _COVERAGE_CACHE[ck]

        # Total cells per room, so coverage is a fraction rather than a raw count
        # (a big room legitimately collects more path points than a small one).
        tkey = hash((len(clean or b""), tuple(sorted(room_ids))))
        totals = _ROOM_TOTALS_CACHE.get(tkey)
        if totals is None:
            totals = {}
            for y in range(sh):
                for x in range(sw):
                    v = int(ap[x, y])
                    if v:
                        totals[v] = totals.get(v, 0) + 1
            _ROOM_TOTALS_CACHE.clear()
            _ROOM_TOTALS_CACHE[tkey] = totals

        touched: dict[int, set] = {}
        for sub in subpaths:
            for p in sub:
                left, top = to_pct(p.x, p.y)
                x = min(sw - 1, max(0, int(round(left / 100.0 * sw))))
                y = min(sh - 1, max(0, int(round(top / 100.0 * sh))))
                v = int(ap[x, y])
                if v:
                    touched.setdefault(v, set()).add((x, y))
        out = {rid: len(touched.get(rid, ())) / tot for rid, tot in totals.items() if tot}
        _COVERAGE_CACHE.clear()
        _COVERAGE_CACHE[ck] = out
        return out
    except Exception:  # noqa: BLE001 -- best effort; callers fall back to the ledger
        _LOGGER.debug("coverage computation failed", exc_info=True)
        return None


def _mode_label(status_cls: type | None, field: str, value: typing.Any) -> str | None:
    """Name of the enum member for `value` on this model's Status class."""
    val = _int_or_none(value)
    if val is None:
        return None
    for name, num in _enum_options(status_cls, field).items():
        if num == val:
            return name
    return str(val)


# water_box_mode == off is the one unambiguous "no water is being used" signal;
# every other value puts the pads down at some intensity.
WATER_OFF = 200


def _modes(st: typing.Any, status_cls: type | None, override: dict | None = None) -> dict:
    """The settings a resume WOULD use -- i.e. what the device reports now.

    Deliberately not described as "what the interrupted run used": that is not
    knowable. The clean-record payload carries no fan / water / mop fields (only
    begin, duration, area, error, complete, start_type, clean_type, wash_count
    and friends), and the live values drift as the robot resets to defaults or
    the user changes them. The single mode fact a past run does leave behind is
    wash_count > 0, which proves it was mopping -- see _last_run.
    """
    override = override or {}
    fan = override.get("fan", st.fan_power)
    water = override.get("water", st.water_box_mode)
    mop = override.get("mop", st.mop_mode)
    fan_l = _mode_label(status_cls, "fan_power", fan)
    water_l = _mode_label(status_cls, "water_box_mode", water)
    mop_l = _mode_label(status_cls, "mop_mode", mop)
    mopping = (_int_or_none(water) or WATER_OFF) != WATER_OFF
    vacuuming = (fan_l or "") != "off"
    kind = "Vacuum + mop" if (mopping and vacuuming) else "Mop only" if mopping else "Vacuum only"
    pretty = lambda s: (s or "?").replace("_", " ")  # noqa: E731
    bits = [kind]
    if vacuuming:
        bits.append(f"suction {pretty(fan_l)}")
    if mopping:
        bits.append(f"water {pretty(water_l)}")
        bits.append(f"route {pretty(mop_l)}")
    return {
        "fan": fan, "water": water, "mop": mop,
        "fan_label": fan_l, "water_label": water_l, "mop_label": mop_l,
        "mopping": mopping, "vacuuming": vacuuming,
        "repeat": override.get("repeat", st.repeat),
        "summary": " · ".join(bits),
    }


def _zone_resume(duid: str, last: dict | None) -> list[dict]:
    """The zones of the last zone job that still need doing.

    Empty when we did not launch it (no rectangles, nothing to offer) or when
    every zone got its clean. A zone with no milestone at all was never reached;
    one that ended mid-zone or lasted seconds did not count.

    The pairing is pseudo segment 41+i -> the i-th zone we launched, the same rule
    the timeline names rows by. It is now sound by construction for ordered jobs:
    those go out through app_zoned_clean_subdivision, which carries our own ids.
    """
    begin = _int_or_none((last or {}).get("begin")) or 0
    logged = RUNS.for_run(duid, begin) if duid else None
    if not logged or logged.get("kind") != "zone":
        return []
    zones = [z for z in (logged.get("requested_zones") or []) if z.get("rect")]
    if not zones:
        return []
    todo = _RUN_TODO_CACHE.get(begin) or {}
    seen = set(todo.get("zones_seen") or ())
    short = todo.get("zones") or {}
    # Kept whole-job on purpose: the question is only whether the job as a whole
    # got through -- every zone reached, none of them cut short -- and if it did
    # not, the whole job goes back on offer.
    if len(seen) >= len(zones) and not short:
        return []
    if (last or {}).get("complete") and not seen:
        return []                    # completed, and no event log to say otherwise
    return [{"name": (z.get("name") or "").strip() or f"Zone {i + 1}",
             "rect": z.get("rect"), "why": "unknown",
             **{k: z.get(k) for k in ("fan", "water", "mop", "repeat")}}
            for i, z in enumerate(zones)]


def _resume_plan(duid: str, st: typing.Any, props: typing.Any = None, status_cls: type | None = None) -> dict:
    """What, if anything, 'resume' should do right now.

    Two different things wear that name:
      * vacuum resume -- the robot is merely paused and its own firmware still
        holds the job. app_start picks up exactly where it left off.
      * program resume -- the run was aborted (error, stop, dock) and the robot
        has forgotten it. We re-issue a segment clean for only the rooms our
        job record says never got cleaned. That is a true resume of the
        *program*, not of the robot's internal job.
    """
    state = _int_or_none(st.state)
    err = _int_or_none(st.error_code) or 0
    in_cleaning = _int_or_none(st.in_cleaning) or 0
    plan: dict = {
        "available": False, "kind": None, "reason": "", "rooms": [],
        "room_reasons": {}, "zones": [],
        "blocked_by_error": False, "source": None, "coverage": {},
        "modes": _modes(st, status_cls),
        "modes_source": "current",
        "pending": TRACKER.pending(duid),
    }
    # The run log is the only record of what the interrupted run was using --
    # the device keeps none. Prefer it, and say which one is on screen.
    logged = RUNS.last_run(duid) if duid else None
    if logged and logged.get("modes"):
        plan["modes"] = _modes(st, status_cls, logged["modes"])
        plan["modes_source"] = "run-log"
        plan["logged_run"] = {
            "started_at": logged.get("started_at"), "ended_at": logged.get("ended_at"),
            "kind": logged.get("kind"), "samples": logged.get("samples"),
            "requested_rooms": logged.get("requested_rooms") or [],
            "errors": logged.get("errors") or [],
        }

    if state == RoborockStateCode.paused.value:
        plan.update(available=True, kind="vacuum", reason="Robot is paused; its own job is still loaded.")

    # No usable job record -- reconstruct from the map's own path trace.
    #
    # This deliberately does NOT require the robot to still be mid-run. The most
    # common way to end up with half a house cleaned is the robot giving up and
    # returning to the dock, at which point error clears, in_cleaning goes to 0
    # and every "is it interrupted" flag reads normal. The map still holds the
    # partial trace, and that is the only remaining evidence.
    #
    # But the trace cannot tell a half-finished full clean from a room or zone
    # clean that did exactly what it was asked to: both leave most of the house
    # untouched. The clean record can, because it says whether the run completed
    # -- so that is the gate. Without it, finishing a four-minute clean of one
    # room offered to "resume" the other nine.
    last = _last_run_cached()
    # Just the verdict, not a second copy of the record: /api/state already
    # carries last_run in full, and this rides along on the 1s status poll.
    plan["last_run_outcome"] = (last.get("finish") or {}).get("outcome") if last else None
    # WHICH run this offer is about. The panel used to infer it from last_run in
    # /api/state, a different payload on a different refresh -- so when that one
    # lagged, the dot on the history list landed on the previous run's row while
    # the offer itself described the newer one. The plan names its own run.
    plan["begin"] = _int_or_none((last or {}).get("begin"))
    # Dismissed by somebody, on some device. This silences the offer -- the dots
    # advertising it, everywhere -- and nothing else: the plan is still worked out
    # in full and resume stays available, because "stop telling me about this" is
    # not "I never want to finish this run". Deciding it was unresumable here was
    # wrong, and left the run detail saying the resume was impossible.
    if last is not None and duid and RUNS.resume_dismissed(
            duid, float(_int_or_none(last.get("begin")) or 0)):
        plan["dismissed"] = True
    if last is not None:
        if last.get("complete") and not last.get("error"):
            plan["not_resumable"] = "completed"
        elif last.get("clean_type") == 2:
            # The device keeps no rectangles, which is why this used to be the end
            # of it -- but the run log has kept them, with their per-zone settings,
            # ever since zone jobs started being logged. So the same question can be
            # asked of a zone job as of a room job: which of the things we asked for
            # did not get done, and can we ask for those again.
            plan["not_resumable"] = "zone"
            zplan = _zone_resume(duid, last)
            if zplan:
                plan.pop("not_resumable", None)
                plan.update(available=True, kind="zones", zones=zplan,
                            source="run-log",
                            reason=f"{len(zplan)} zone(s) left unfinished.")
    #
    # If we launched a segment clean ourselves, the check is scoped to the rooms
    # it asked for: the rest were never meant to be cleaned, so offering them
    # would be noise. That room list is the only run state kept, it lives in
    # memory, and the sampler drops it once the robot is docked and idle.
    launch = TRACKER.last_launch(duid)
    scope = set(launch["segments"]) if launch and launch["kind"] == "segment" and launch["segments"] else None
    # TRACKER lives in memory, so a restart forgets which rooms the last run was
    # asked for and the offer widens to whatever the map trace happens to think.
    # The run log is on disk and is matched to this very record by when it ran,
    # so it answers the same question across a restart.
    if scope is None and last:
        logged = RUNS.for_run(duid, _int_or_none(last.get("begin")) or 0)
        if logged and logged.get("kind") == "segment" and logged.get("requested_rooms"):
            scope = {int(s) for s in logged["requested_rooms"]}

    # A newer job has taken over. The offer below is about the run that ENDED,
    # and that run stops being anything you can go back to the moment a
    # different one is under way -- the rooms it left are being cleaned now, or
    # are about to be, by something else.
    #
    # Waiting for the new run to close and show up in the clean summary is far
    # too late: that is a whole clean's worth of a badge pointing at history
    # that has already been overtaken. Two signals, because either can arrive
    # first: our own launch stamp (instant, and the only one that catches a job
    # starting before the robot has moved) and the robot reporting that it is
    # cleaning (the only one that catches a job we did not start).
    last_end = float((last or {}).get("end") or (last or {}).get("begin") or 0)
    superseded = bool(launch and float(launch.get("at") or 0) > last_end)
    if _is_running(state, in_cleaning):
        superseded = True

    # The run's own event log first. It is strictly better informed than the
    # path trace: the trace only knows where the robot drove, the log also knows
    # what it was doing there. A house the robot finished vacuum-only because
    # the water tank faulted half way reads as fully covered on the trace and as
    # five rooms still owed a mop pass in the log -- and the log is right.
    #
    # That is also why this is allowed to overrule `not_resumable == "completed"`:
    # a run can reach the end of its room list and still not have done the job it
    # was started with. It never overrules "zone", where there are no rectangles
    # left to go back to.
    todo = _RUN_TODO_CACHE.get(_int_or_none((last or {}).get("begin")) or -1)
    if todo and not superseded and plan["kind"] not in ("vacuum", "zones") \
            and plan.get("not_resumable") != "zone":
        rooms, notes = _todo_rooms(todo, scope)
        if rooms:
            plan.pop("not_resumable", None)
            plan.update(
                available=True, kind="program", source="timeline",
                rooms=rooms, room_reasons=notes, reason=_todo_reason(notes),
            )
    resumable = plan.get("not_resumable") is None

    if props is not None and plan["kind"] != "vacuum" and resumable and not superseded:
        cov = _coverage(props)
        if cov and scope:
            cov = {k: v for k, v in cov.items() if k in scope}
        if cov:
            plan["coverage"] = {str(k): round(v, 3) for k, v in sorted(cov.items())}
            best = max(cov.values())
            cutoff = max(COVERAGE_FLOOR, best * COVERAGE_REL)
            missed = [rid for rid, frac in sorted(cov.items()) if frac < cutoff]
            # If nothing was cleaned at all, `best` is meaningless and every room
            # looks "missed" -- that is a run that never started, not a resume.
            # The trace is only consulted when the event log had no answer.
            if not plan["rooms"] and missed and len(missed) < len(cov) and best >= COVERAGE_FLOOR:
                plan.update(
                    available=True,
                    kind="program",
                    source="map",
                    rooms=missed,
                    reason=(
                        ("The robot docked without finishing. " if not (err or in_cleaning) else "")
                        + f"{len(missed)} of {len(cov)} rooms show no cleaning coverage."
                    ),
                )
    if duid and plan["kind"] == "program" and plan["rooms"]:
        RUNS.note_outcome(duid, plan["rooms"], plan["coverage"])

    if err:
        # Still 'available', but the caller must clear the fault or pass force.
        plan["blocked_by_error"] = jobs_mod.describe_error(err)["severity"] == "blocking"
    return plan


# Parts the robot can tell us are missing or full. All of them are decoded by the
# library off one dock-status bitfield (`dss`) or off a plain status field, and none
# of them raise an error code -- so with the dust bin out, nothing in the panel said
# so. Reported as an explicit label rather than a raw number: `water_box_status` of 0
# means "no tank fitted", which reads as false rather than as absent.
_PART_LABELS_FR = {
    "dust_bin": {"not_installed": "bac à poussière retiré"},
    "water_box": {"not_installed": "réservoir d'eau retiré"},
    "mop": {"not_installed": "serpillière retirée"},
}
# The robot's own dust bin. The decoding lives in the fork now
# (Status.dust_bin_status / DustBinStatus), which is where protocol knowledge
# belongs -- the evidence for it is recorded there. All this does is turn the
# library's enum into the panel's own vocabulary; `unknown` becomes None, so an
# unexpected value is never presented as a fact.
_DUST_BIN_BY_NAME = {"installed": "okay", "not_installed": "not_installed"}
# The two the dock removes and refits by itself. The bin is not one of them: nobody
# takes that out mid-run, so if it goes it is worth hearing about whatever the robot
# happens to be doing.
_MOP_PARTS = {"mop", "water_box"}


# What the DOCK is doing while the robot sits on it. Neither shows up in the state
# code -- the robot reports `charging` throughout -- so without this the panel said
# "docked" through a two-hour drying cycle.
_DOCK_BUSY_FR = {
    "drying": "séchage de la serpillière",
    "washing": "lavage de la serpillière",
}
# The state codes that ARE the robot washing, from the library's own enum. Included
# because a wash can also be reported this way, and then the headline already says
# it -- this only has to catch the case where the code says `charging`.
_WASHING_STATES = {23, 25, 26}


_LAST_DOCK_BUSY: tuple | None = None


def _note_dock_busy(st: typing.Any, busy: list[str]) -> None:
    """Journal every start and stop of a dock cycle, with why it might have happened.

    Written because the obvious questions about drying cannot be answered from
    anything that is kept: does the dock dry after every mop job, and does pressing
    stop suppress the next one? A run's own event log cannot say -- drying begins
    after job_end, once the robot is docked, which is outside that log entirely --
    and nothing else records the transition. So it is recorded here, on change only,
    which for a dock is a handful of lines a day rather than a stream.

    The context is what makes it answerable afterwards: the state it was in, whether
    a job was still outstanding, and whether the pads were even fitted.
    """
    global _LAST_DOCK_BUSY
    now = tuple(busy)
    if _LAST_DOCK_BUSY is None:
        _LAST_DOCK_BUSY = now
        return
    if now == _LAST_DOCK_BUSY:
        return
    was, _LAST_DOCK_BUSY = _LAST_DOCK_BUSY, now
    _journal("dock_busy", {
        "from": list(was), "to": list(now),
        "state": _int_or_none(getattr(st, "state", None)),
        "battery": _int_or_none(getattr(st, "battery", None)),
        "in_cleaning": _int_or_none(getattr(st, "in_cleaning", None)),
        "dry_status": _int_or_none(getattr(st, "dry_status", None)),
        "wash_phase": _int_or_none(getattr(st, "wash_phase", None)),
        # 1 when the pads are on the robot; the dock has nothing to dry without them.
        "pads_fitted": _int_or_none(getattr(st, "water_box_carriage_status", None)),
        "clean_time": _int_or_none(getattr(st, "clean_time", None)),
    })


def _dock_activity(st: typing.Any) -> list[str]:
    """Drying / washing, as keys into _DOCK_BUSY_FR.

    `dry_status` is the live drying flag, not the auto-dry setting: driving
    app_set_dryer_status both ways moves it (1 -> 0 -> 1) while
    APP_GET_DRYER_SETTING's own `status` stays at 1 throughout. It carries a DPS code
    (DRYING_STATUS) too, so it arrives pushed rather than polled.

    What it is NOT is a latch that stays off until something re-arms it -- an earlier
    comment here claimed that, on the strength of one press appearing to persist.
    It re-armed on its own within the day, and APP_GET_DRYER_SETTING's `cron` moved
    with it (38 0 * * * -> 2 14 * * *), so that field tracks the cycle rather than
    holding a schedule anybody set. How the dock decides to start one is not known
    from here.

    `wash_status` is NOT a flag and is deliberately not read here. Reading it as
    "non-zero means washing" left the chip stuck on "lavage de la serpillière" for
    hours after a run: it sits at 512 on a parked, dry, idle robot, which is a bit
    in a word rather than a state. `wash_phase` is the one that reads 0 when nothing
    is happening.

    The washing STATE codes are not read either, and that is the point: when the
    robot reports washing_the_mop the chip's headline already says so, and adding
    the French label beside it produced "washing_the_mop . lavage de la
    serpillière" -- one fact, twice, in two languages. This list is only for what
    the state does not already say, which is why drying belongs in it (the robot
    reports `charging` throughout) and washing-as-a-state does not.
    """
    out: list[str] = []
    if _int_or_none(getattr(st, "dry_status", None)):
        out.append("drying")
    if _int_or_none(getattr(st, "wash_phase", None)):
        out.append("washing")
    return out


# What a fresh clean-settings sheet should offer, by NAME rather than by code, so it
# survives a device whose option table numbers things differently.
#
# The sheet used to seed itself from whatever the robot happened to be set to. That
# is fine until the official app is used, which resets the device to its own idea of
# a default -- smart_mode, or the `custom` markers left over from a per-room run --
# and neither is a thing this panel's editor can represent. The sheet then opened on
# a mode with no sliders, or on "Personnalisé" with nothing behind it.
DEFAULT_CLEAN = {
    "mode": os.environ.get("RR_DEFAULT_MODE", "vacuum"),
    "fan": os.environ.get("RR_DEFAULT_FAN", "max"),
    "water": os.environ.get("RR_DEFAULT_WATER", "off"),
    "mop": os.environ.get("RR_DEFAULT_ROUTE", "standard"),
    "repeat": int(os.environ.get("RR_DEFAULT_REPEAT", "1") or 1),
}


def _default_clean(options: dict) -> dict:
    """DEFAULT_CLEAN resolved against this device's own option table.

    A name that the device does not offer is dropped rather than guessed at, so the
    frontend can tell "not configured" from "configured to something impossible".
    """
    out: dict[str, typing.Any] = {"mode": DEFAULT_CLEAN["mode"],
                                  "repeat": max(1, min(3, DEFAULT_CLEAN["repeat"]))}
    for key in ("fan", "water", "mop"):
        table = options.get(key) or {}
        name = DEFAULT_CLEAN[key]
        if name in table:
            out[key] = table[name]
            out[key + "_name"] = name
        else:
            out[key] = None
            _LOGGER.debug("default %s=%r not offered by this device", key, name)
    return out


def _parts_state(st: typing.Any) -> dict:
    """Which removable parts the robot reports as absent or full.

    The two plain fields are trustworthy: water_box_status and
    water_box_carriage_status are 0 when nothing is fitted, and that is confirmed
    behaviour.

    The robot's own dust bin comes from Status.dust_bin_status, added to the fork
    for this: see DustBinStatus there for how it was traced.

    The dock's `dss` bitfield is NOT read through the library's enums here, and that
    is deliberate. RoborockDssCodes._missing_ returns `okay` for any value it does
    not recognise -- "the math will sometimes result in you getting numbers that
    don't matter", per its own comment -- so a field this model sets to a value the
    2016 table never had is reported as fine. With the dust bin physically removed
    this device reads dss=2216, whose dust-bag field is 2: not in the enum, silently
    okay. So the raw two-bit values are carried through instead, and a label is
    attached only where the meaning is confirmed for this model. Same rule as
    UNVERIFIED_STATE_CODES: never present a code as though it were understood.
    """
    # A job in progress changes what "missing" is worth saying. The dock takes the
    # pads OFF for a vacuum-only run -- that is the job working as asked, not a part
    # gone astray -- so announcing "serpillière retirée" for the whole clean is
    # noise, and it turns the chip amber for it too. Docked, the same fact IS worth
    # seeing: it is how you notice the dock has just taken them off.
    # "A job is outstanding", not "it is moving right now": paused mid-run, the pads
    # are off for exactly the same reason, so in_cleaning counts as well.
    in_cleaning = getattr(st, "in_cleaning", None)
    running = bool(in_cleaning) or _is_running(
        _int_or_none(getattr(st, "state", None)), in_cleaning)
    out: dict[str, typing.Any] = {}
    wb = _int_or_none(getattr(st, "water_box_status", None))
    out["water_box"] = None if wb is None else ("okay" if wb else "not_installed")
    carriage = _int_or_none(getattr(st, "water_box_carriage_status", None))
    out["mop"] = None if carriage is None else ("okay" if carriage else "not_installed")

    try:
        bin_state = getattr(st, "dust_bin_status", None)
    except Exception:  # noqa: BLE001 -- a property that throws is not a fact
        bin_state = None
    out["dust_bin"] = _DUST_BIN_BY_NAME.get(getattr(bin_state, "name", "") or "")
    out["rss"] = _int_or_none(getattr(st, "rss", None))   # the raw value, for diagnosis

    dss = _int_or_none(getattr(st, "dss", None))
    out["dss"] = dss
    if dss is not None:
        # The library's own field offsets, reported as the numbers they are.
        out["dss_fields"] = {
            "am": dss & 3, "clear_water_box": (dss >> 2) & 3,
            "dirty_water_box": (dss >> 4) & 3, "dust_bag": (dss >> 6) & 3,
            "water_box_filter": (dss >> 8) & 3, "clean_fluid": (dss >> 10) & 3,
        }
    out["missing"] = [
        _PART_LABELS_FR[k][v]
        for k, v in out.items()
        if isinstance(v, str) and k in _PART_LABELS_FR and v in _PART_LABELS_FR[k]
        and not (k in _MOP_PARTS and running)
    ]
    # Alongside, not inside `missing`: a dock that is drying is not a fault and must
    # not turn the chip amber the way a removed part does.
    busy = _dock_activity(st)
    _note_dock_busy(st, busy)
    out["busy"] = busy
    out["busy_labels"] = [_DOCK_BUSY_FR[k] for k in busy if k in _DOCK_BUSY_FR]
    return out


def _mop_errand_label(state: int | None, state_label: str, st: typing.Any,
                      duid: str) -> str:
    """Say which mop errand the robot is driving to the dock for.

    The firmware reports ONE code for the trip -- 26, which the enum calls
    going_to_wash_the_mop -- and only reveals what it actually went for once it
    arrives: 33 attaching_the_mop, 34 detaching_the_mop, 23 washing_the_mop. So
    the chip announced a wash every time, including on the way to drop the pads
    off before a vacuum-only job, which is the one errand it certainly was not.

    What it went for follows from the job and the plate: a job with no water and
    the pads still on can only be going to take them off; a job that mops with no
    pads on can only be going to fetch them. Anything else -- mid-job, pads and
    water in agreement -- is a wash, which is what the code says.

    Only the trip is renamed. Once the robot is there, 33 / 34 / 23 say it
    themselves and are left alone.
    """
    if state != RoborockStateCode.going_to_wash_the_mop.value:
        return state_label
    pads = _int_or_none(getattr(st, "water_box_carriage_status", None))
    logged = RUNS.last_run(duid) if duid else None
    if not logged or not _int_or_none(getattr(st, "in_cleaning", None)):
        return state_label
    wet = _run_used_mop(logged)
    if not wet and pads:
        return "going_to_detach_the_mop"
    if wet and pads == 0:
        return "going_to_attach_the_mop"
    return state_label


def _status_dict(st: typing.Any, duid: str | None = None, props: typing.Any = None,
                 status_cls: type | None = None) -> dict:
    state = _int_or_none(st.state)
    state_label = STATE_LABELS.get(state if state is not None else -1, str(st.state))
    err = _int_or_none(st.error_code) or 0
    err_info = jobs_mod.describe_error(err)
    duid = duid or (STATE.selected_duid or "")

    ev = TRACKER.observe_error(duid, err) if duid else None
    if ev is not None:
        since = float(ev["started_at"])
        err_info = {
            **err_info,
            "since": since,
            "seconds": max(0.0, time.time() - since),
            "duration": jobs_mod.humanize_duration(time.time() - since),
        }

    state_label = _mop_errand_label(state, state_label, st, duid)
    return {
        "state": state,
        "state_label": state_label,
        # The name above comes from an enum never validated against this model;
        # false here means "show the error / motion flags instead".
        "state_trusted": state not in UNVERIFIED_STATE_CODES,
        # Observed wording for a code the enum names wrongly; see MODEL_STATE_LABELS.
        "state_observed": MODEL_STATE_LABELS.get(state),
        "error": err_info,
        "position": _vacuum_position(props, state_label),
        # What the robot is physically doing, for the live marker on the map.
        "running": _is_running(state, st.in_cleaning),
        "resume": _resume_plan(duid, st, props, status_cls) if duid else {"available": False},
        # Removable parts the robot says are missing or full -- the dust bin, the
        # tanks, the mop plate. None of them raise an error code, so without this the
        # panel had no way to mention them.
        "parts": _parts_state(st),
        # Consumables near the end of their life. Rides along here because the
        # counters are pushed datapoints, so it is free and it is live -- the dock
        # button can carry a badge without the panel ever fetching /api/maintenance.
        "wear": _wear_alert(props) if props is not None else {},
        "battery": st.battery,
        "clean_area": round((st.clean_area or 0) / 1_000_000, 1),  # mm² -> m²
        "clean_time_min": round((st.clean_time or 0) / 60),
        "clean_time": st.clean_time,
        "clean_percent": st.clean_percent,
        "error_code": str(st.error_code),
        "in_cleaning": st.in_cleaning,
        "in_returning": st.in_returning,
        "charge_status": st.charge_status,
        "dock_error": _int_or_none(getattr(st, "dock_error_status", None)),
        "dock_error_info": jobs_mod.describe_dock_error(_int_or_none(getattr(st, "dock_error_status", None))),
        "fan_power": st.fan_power,
        "water_box_mode": st.water_box_mode,
        "mop_mode": st.mop_mode,
        "repeat": st.repeat,
        "dock_type": str(st.dock_type),
    }


# --------------------------------------------------------------------------- #
# Live push
# --------------------------------------------------------------------------- #
# The panel used to ask, once a second per open tab: /api/status (a status RPC)
# and -- while a job was running -- /api/map.png?fresh=1, which is a ~400 KB map
# read. Neither was necessary. The vacuum PUSHES its own state changes over MQTT
# as DPS datapoints (state, battery, error, fan power, water box, charge, drying,
# task complete), and python-roborock already decodes them into the status trait
# and fires add_update_listener. So the panel can be told instead of asking, and
# the only things left to poll are the ones the device does not volunteer:
# progress numbers (clean_time / area / percent) and the map, both of which are
# only interesting while a job is running and somebody is watching.
#
# What travels over the socket:
#   status  -- the same object /api/status returns, whenever it changes
#   map     -- a version number; the page re-fetches the PNG only when it moves
#   runs    -- "the history and the resume offer are stale", on a job ending
#   state   -- "structure changed, re-read /api/state"
# Nothing is broadcast unless it differs from what was last sent.
class Hub:
    """The browsers currently watching.

    Whether anyone is watching is a real input, not bookkeeping: it decides how
    often the sampler bothers the vacuum. Nobody watching means the slow cadence
    that only exists to keep the run log honest.
    """

    def __init__(self) -> None:
        self._clients: set[web.WebSocketResponse] = set()
        self._last: dict[str, str] = {}

    def __len__(self) -> int:
        return len(self._clients)

    def add(self, ws: web.WebSocketResponse) -> None:
        self._clients.add(ws)

    def discard(self, ws: web.WebSocketResponse) -> None:
        self._clients.discard(ws)
        if not self._clients:
            # Nothing has been sent to the next client that connects, so it must
            # not be told "unchanged" about a frame it never received.
            self._last.clear()

    async def send(self, kind: str, frame: dict, *, force: bool = False) -> None:
        if not self._clients:
            return
        body = json.dumps({"t": kind, **frame}, default=str)
        if not force and self._last.get(kind) == body:
            return
        self._last[kind] = body
        dead = []
        for ws in list(self._clients):
            try:
                await ws.send_str(body)
            except Exception:  # noqa: BLE001 -- a closed socket is not an error here
                dead.append(ws)
        for ws in dead:
            self.discard(ws)

    async def close_all(self) -> None:
        """Let every watcher go. Called on shutdown, before aiohttp waits.

        handle_ws parks in `async for msg in ws` for as long as the browser keeps
        the socket open -- which is the whole point of it, and on shutdown means a
        handler that never returns. run_app's graceful phase waits for handlers, so
        with nothing closing these the wait ran to shutdown_timeout and Docker's
        10s grace expired first: the container died by SIGKILL every time.
        """
        clients, self._clients = list(self._clients), set()
        self._last.clear()
        for ws in clients:
            try:
                await ws.close(code=1001, message=b"server shutting down")
            except Exception:  # noqa: BLE001 -- it is going away regardless
                pass
        if clients:
            _LOGGER.info("closed %d live socket(s) for shutdown", len(clients))

    async def send_to(self, ws: web.WebSocketResponse, kind: str, frame: dict) -> None:
        try:
            await ws.send_str(json.dumps({"t": kind, **frame}, default=str))
        except Exception:  # noqa: BLE001
            self.discard(ws)


HUB = Hub()
_STATUS_DIRTY = asyncio.Event()
PUSH_COALESCE = 0.15   # s: a single MQTT frame can carry several datapoints


def _mark_status_dirty() -> None:
    """Called from the MQTT thread's own callback -- must not block or await."""
    _STATUS_DIRTY.set()


def _status_frame() -> dict | None:
    """The status object, built from traits the MQTT layer keeps current.

    No lock and no RPC: this is exactly why the push exists. `props` is read
    directly rather than through STATE.props(), which would try to connect.
    """
    device = STATE.device()
    if device is None or getattr(device, "v1_properties", None) is None:
        return None
    props = device.v1_properties
    try:
        return _status_dict(props.status, props=props, status_cls=_status_class(device))
    except Exception:  # noqa: BLE001 -- a push must never take down the pusher
        _LOGGER.debug("status frame failed", exc_info=True)
        return None


async def push_status() -> None:
    frame = _status_frame()
    if frame is not None:
        await HUB.send("status", {"status": frame})


async def push_obstacles(found: list[dict]) -> None:
    """The live map's detections, as they are found.

    The vacuum adds them to the live map during the run, so this is genuinely
    live -- and the panel only ever read them once, at load. That meant the pins on
    screen were the PREVIOUS run's for the whole of a clean: nothing new appeared as
    it was found, and the old ones stayed put after the map had moved on. The list
    comes off the parse the sampler already did, so it costs no round trip, and
    Hub.send drops a frame identical to the last -- which is most of them.
    """
    await HUB.send("obstacles", {"obstacles": found})


async def push_map() -> None:
    """Tell the watchers which picture is current.

    Recomputes the token first, because what is drawn depends on the run state as
    well as on the map bytes -- a job starting switches the trace without the bytes
    having changed yet. Hub.send drops a frame identical to the last, so calling
    this when nothing has moved costs nothing.
    """
    await HUB.send("map", {"v": _current_map_token()})


async def _pusher(_app: web.Application) -> None:
    """Coalesce datapoint pushes into one frame per burst."""
    while True:
        await _STATUS_DIRTY.wait()
        # Wait out the burst BEFORE clearing, not after: one MQTT message often
        # decodes to several datapoints and a state change arrives alongside the
        # battery reading. Clearing first meant the datapoints that landed during
        # the wait re-armed the flag, and the burst became two frames instead of
        # one. Clearing after means a datapoint that arrives during the push
        # itself still survives into the next lap, which is the case that matters.
        await asyncio.sleep(PUSH_COALESCE)
        _STATUS_DIRTY.clear()
        try:
            await push_status()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            _LOGGER.debug("status push failed: %s", exc)


_LAST_TIMELINE: dict | None = None
"""The running job's events as last read by the sampler.

Kept because Hub.send drops a frame that says nothing new -- which is right for a
steady stream, and wrong for a socket that has just arrived: between two changes it
would be sent nothing at all and show an empty Déroulement while a job was under
way. The hello handshake sends this snapshot the same way it sends the status one.
"""


async def handle_ws(request: web.Request) -> web.WebSocketResponse:
    """The live channel. Read-only: the panel still acts over the REST endpoints.

    heartbeat keeps the connection alive through the reverse proxy in front of
    this (see NOTES.md on the sslh/nginx path) -- an idle vacuum can go minutes
    without a datapoint, which some proxies read as a dead socket.
    """
    ws = web.WebSocketResponse(heartbeat=25)
    await ws.prepare(request)
    HUB.add(ws)
    _LOGGER.info("ws client connected (%d watching)", len(HUB))
    try:
        # A snapshot immediately, so a page that has just opened does not wait for
        # the vacuum to volunteer something before it can draw.
        await HUB.send_to(ws, "hello", {"map_v": _current_map_token()})
        frame = _status_frame()
        if frame is not None:
            await HUB.send_to(ws, "status", {"status": frame})
        # ...and the running job's events, for the same reason: a page that has just
        # opened must not have to wait for the next CHANGE before it can draw.
        if _LAST_TIMELINE is not None:
            await HUB.send_to(ws, "timeline", {"timeline": _LAST_TIMELINE})
        async for msg in ws:
            if msg.type == WSMsgType.TEXT and msg.data == "ping":
                await ws.send_str('{"t":"pong"}')
            elif msg.type == WSMsgType.ERROR:
                break
    finally:
        HUB.discard(ws)
        _LOGGER.info("ws client gone (%d watching)", len(HUB))
    return ws


async def handle_status(_request: web.Request) -> web.Response:
    """Lightweight live status (just a status.refresh).

    Only the fallback path now: the page reads status off the WebSocket, and comes
    back here when the socket is down (a proxy that will not upgrade, a dead
    network) so nothing depends on the push having worked.
    """
    async with STATE._lock:
        props = await STATE.props()
        await props.status.refresh()
        dev = STATE.device()
        return web.json_response(
            _status_dict(props.status, props=props, status_cls=_status_class(dev) if dev else None)
        )


def _map_bg_hex() -> str:
    """The colour the renderer fills the area OUTSIDE the map with.

    The map stage is taller than the picture on a phone, and the leftover strip
    above and below it was showing the panel's own inset colour -- a near-black
    band in the dark theme. Filling it with this instead makes the letterbox
    disappear into the render. Read off the palette rather than pinned in the
    CSS: it is the renderer's colour, not a choice of ours.
    """
    try:
        from vacuum_map_parser_base.config.color import ColorsPalette, SupportedColor

        r, g, b = tuple(ColorsPalette().get_color(SupportedColor.MAP_OUTSIDE))[:3]
        return f"#{r:02x}{g:02x}{b:02x}"
    except Exception:  # noqa: BLE001
        return "#135794"   # the library default, which is what it has always been


# When the map bytes were last pulled, by anyone -- a request, the sampler, the
# warm-up -- and a version that changes only when the bytes actually do. A map is
# ~400 KB over MQTT. The browser used to ask for a fresh one every second while a
# job was running (see handle_map_png), so watching a clean cost ~400 KB/s per open
# tab; now the server decides when the map is worth re-reading and tells the page
# over the socket, and /api/map.png only ever renders what is already in hand.
_MAP_READ_AT = [0.0]
_LAST_MAP_PNG: list[bytes | None] = [None]   # the last render that worked
_MAP_DIGEST = [""]      # content hash of the map bytes
_MAP_VERSION = [""]     # what /api/map.png?v= carries: see _map_token
MAP_TTL_IDLE = 25.0
# Bump when the DRAWING changes (_draw_trace, _display_render, the palette). The
# token below is derived from the map's content, which is the point -- but that
# means identical bytes rendered by new code would keep the same URL, and browsers
# hold these `immutable`. This is the part of the identity that says "rendered by
# this version of the renderer".
_RENDER_REV = "3"
# The floor under a hand-driven ?fresh=1. Not zero: the fallback poller asks for
# one while a job runs, and three tabs on the fallback must not mean three ~400 KB
# reads a second. One read serves all of them.
MAP_TTL_FRESH = 2.0


def _map_token() -> str:
    """The identity of the picture /api/map.png would return right now.

    Everything that changes what is drawn, and nothing that does not: the map's
    own content hash, which trace is being drawn over it, and the renderer's
    revision.

    It used to be a counter, and that was a real bug. A counter is process-local
    and restarts at zero, so `?v=4` meant one picture before a restart and a
    different one after -- while browsers had been told to keep it `immutable` for
    a year. A phone that had cached `?v=4` showed that old map indefinitely; a
    fresh private tab, having cached nothing, looked correct. Deriving the token
    from the content makes the promise true: same bytes, same URL, any process.
    """
    return f"{_RENDER_REV}-{_MAP_DIGEST[0] or '0'}"


def _sync_map_token() -> bool:
    """Recompute the token; True when it moved."""
    token = _map_token()
    if token == _MAP_VERSION[0]:
        return False
    _MAP_VERSION[0] = token
    return True


def _current_map_token() -> str:
    """The token to hand out, brought up to date first."""
    _sync_map_token()
    return _MAP_VERSION[0]


def _note_map_bytes(props: typing.Any) -> bool:
    """Re-hash the map bytes and refresh the token. True when the picture changed.

    A content hash rather than a timestamp: the sampler re-reads the map on a timer
    whether or not anything moved, and telling every browser to re-fetch a picture
    identical to the one it is showing is exactly the traffic this is trying to
    remove.
    """
    raw = props.map_content.raw_api_response
    # The FLOOR's digest: the picture no longer moves with the route or the robot,
    # so neither should the token -- otherwise every read of a run re-sends it.
    _MAP_DIGEST[0] = _floor_digest(raw)
    _note_live_map(props.map_content.map_data, raw)
    return _sync_map_token()


async def _refresh_map(props: typing.Any, running: bool = False) -> bool:
    """Re-read the map unless a fresh enough copy is already in hand.

    Returns True when the bytes changed. While the robot is moving there is no
    TTL: the path trace is the map's whole point then. Anything that CHANGES the
    map out of band (a rename, a barrier edit, a reset, a map switch) calls
    _drop_map_ttl, so the next read is real -- a stale floorplan after an edit
    would be a worse bug than a slow one.
    """
    if not running and props.map_content.map_data is not None \
            and (time.time() - _MAP_READ_AT[0]) < MAP_TTL_IDLE:
        return False
    await props.map_content.refresh()
    _MAP_READ_AT[0] = time.time()
    return _note_map_bytes(props)


def _drop_map_ttl() -> None:
    _MAP_READ_AT[0] = 0.0


async def handle_state(request: web.Request) -> web.Response:
    device = STATE.device()
    payload: dict[str, typing.Any] = {
        "base_url": BASE_URL,
        "map_bg": _map_bg_hex(),
        "devices": [
            {"duid": d.duid, "name": d.name, "model": (d.product.model if d.product else None)}
            for d in STATE.devices
        ],
        "selected": STATE.selected_duid,
    }
    if device is None:
        return web.json_response(payload)

    async with STATE._lock:
        props = await STATE.props()
        await props.status.refresh()
        try:
            await props.maps.refresh()
        except Exception as exc:  # noqa: BLE001
            _LOGGER.info("maps.refresh skipped: %s", exc)
        await props.rooms.refresh()
        st = props.status
        # A map read that times out must not take the whole payload down with it.
        # Everything below either works from the copy already in hand or reports
        # nothing for that field, which is a far better answer than a 500 -- and on
        # a busy vacuum (a clean in progress, several tabs open) a 10s command
        # timeout here is a thing that happens.
        try:
            await _refresh_map(props, running=_is_running(_int_or_none(st.state), st.in_cleaning))
        except Exception as exc:  # noqa: BLE001
            _LOGGER.info("state: map refresh skipped (%s); using the copy in hand",
                         exc.__class__.__name__)

        room_bboxes = _room_bboxes(props.map_content.map_data)  # {segment_id: [x%,y%,w%,h%]}
        map_names = _map_room_names(props)  # authoritative where room_map is corrupt
        room_tags = await _room_tags(props)
        rooms = [
            {"segment_id": int(s), "name": map_names.get(int(s)) or _room_name(rm, int(s)),
             "bbox": room_bboxes.get(int(s)), "tag": room_tags.get(int(s))}
            for s, rm in sorted(props.rooms.room_map.items())
        ]
        rooms = _order_rooms(rooms)
        # One render, three readers: the markers, the room index, and nothing else
        # needs to re-parse the map bytes.
        clean_render = (_clean_room_render(props.map_content.raw_api_response)
                        or props.map_content.image_content)
        status_cls = _status_class(device)
        # Ahead of _status_dict: the resume gate reads the cached clean record,
        # so it has to be warm before the status block is built, not after it.
        last_run = await _last_run(props)
        # Same reason, one step further: what that run left undone comes from its
        # event log, which is an RPC the 1s status poll cannot make. One fetch
        # per run, memoised on its begin.
        await _run_todo(props, last_run, {int(s) for s in props.rooms.room_map})

        maps_info = []
        try:
            for mi in (props.maps.map_info or []):
                maps_info.append({"flag": mi.map_flag, "name": getattr(mi, "name", None) or f"Map {mi.map_flag}"})
        except Exception:  # noqa: BLE001
            pass

        payload.update(
            {
                "model": device.product.model if device.product else None,
                "status": _status_dict(st, props=props, status_cls=status_cls),
                "rooms": rooms,
                "overlays": _room_overlays(props.map_content.map_data, clean_render),
                "barriers": {
                    **_barrier_overlays(props.map_content.map_data),
                    "thresholds": _block_quad_overlays(props.map_content.map_data, props.map_content.raw_api_response, 28),
                    "carpet": _block_quad_overlays(props.map_content.map_data, props.map_content.raw_api_response, 39),
                },
                "can_zone": _image_to_vacuum(props.map_content.map_data) is not None,
                # The version of the map bytes this payload was built from, so the
                # page can ask for that exact picture -- and so a page whose socket
                # has not opened yet still starts out in step with the pushes.
                "map_v": _current_map_token(),
                "maps": {"current": props.maps.current_map, "list": maps_info},
                "last_run": last_run,
                "options": {
                    **(await _mode_options(props)),
                    "cleaning_modes": await _cleaning_modes(props),
                    # Numeric water-slider positions -> level name (6.x).
                    "water_slider": {int(k): v.value for k, v in WATER_SLIDE_MODE_MAPPING.items()},
                    "materials": GROUND_MATERIALS,
                    "carpet_modes": CARPET_CLEAN_MODES,
                },
            }
        )
        # Resolved here, where the device's own option table is already in hand.
        payload["defaults"] = _default_clean(payload["options"])
        ri = _room_index_payload(clean_render, [r["segment_id"] for r in rooms])
        if ri is not None:
            # The client echoes the key it already holds; an unchanged map then
            # costs 40 bytes instead of the whole index on every 30s poll.
            payload["room_index"] = ({"key": ri["key"]}
                                     if request.query.get("room_index_key") == ri["key"] else ri)
    # The biggest response the panel fetches, and on a first load it carries the
    # whole room index. Compression is negotiated from the request's own
    # Accept-Encoding, so a client that does not want it does not get it.
    resp = web.json_response(payload)
    resp.enable_compression()
    return resp


_ROOM_INDEX_CACHE: dict[tuple, dict] = {}


def _room_index_payload(image_content: bytes | None, room_ids: list[int]) -> dict | None:
    """The per-pixel room index -- greyscale PNG whose pixel value is the segment
    id under it (0 = no room) -- base64'd for inlining in /api/state.

    Which room a tap landed in cannot be answered by a bounding box. The rooms
    here are L-shaped and their boxes overlap heavily -- a corridor's box covers
    half the flat -- so whichever box happened to be drawn last won the click and
    you selected the room next door. So the browser gets the per-pixel assignment
    itself and resolves every tap, every hover, and every selection highlight
    locally and exactly.

    It rides along with the state that describes it: /api/state is already the
    thing that knows the map changed, and fetching the shapes separately only
    added a round trip the page had to survive. `key` (content hash) lets the
    poll re-send nothing while the map holds still.

    Greyscale and profile-free on purpose: the browser must hand back the byte
    that was written, not a colour-managed approximation of it.
    """
    if not image_content or not room_ids:
        return None
    ck = (len(image_content), tuple(sorted(room_ids)))
    hit = _ROOM_INDEX_CACHE.get(ck)
    if hit is not None:
        return hit
    assign = _room_pixels(image_content, room_ids)["assign"]
    if assign is None:
        return None
    import base64
    import hashlib
    import io

    buf = io.BytesIO()
    assign.save(buf, format="PNG", optimize=True)
    png = buf.getvalue()
    payload = {
        "key": hashlib.sha1(png).hexdigest(),
        "w": assign.width,
        "h": assign.height,
        "png": base64.b64encode(png).decode(),
    }
    _ROOM_INDEX_CACHE.clear()  # only the current map's index is ever wanted
    _ROOM_INDEX_CACHE[ck] = payload
    return payload


async def handle_raw_map(request: web.Request) -> web.Response:
    """Read-only: the raw map bytes (base64) for protocol analysis. ?persist=1 uses
    get_persist_map instead of the live map."""
    import base64
    async with STATE._lock:
        props = await STATE.props()
        content = props.map_content
        if content.raw_api_response is None:
            await content.refresh()
        raw = content.raw_api_response
    if not raw:
        raise web.HTTPNotFound(reason="no raw map")
    return web.json_response({"len": len(raw), "b64": base64.b64encode(bytes(raw)).decode()})


async def handle_map_debug(_request: web.Request) -> web.Response:
    """Unknown map blocks (thresholds, carpet, etc.) as labelled overlays."""
    async with STATE._lock:
        props = await STATE.props()
        content = props.map_content
        if content.image_content is None:
            await content.refresh()
        blocks = _debug_blocks(content.map_data, content.raw_api_response)
    return web.json_response({"blocks": blocks})


async def handle_features(_request: web.Request) -> web.Response:
    """Read-only: which capability flags this device reports (helps answer 'can it X')."""
    import dataclasses
    async with STATE._lock:
        props = await STATE.props()
        df = props.device_features
        try:
            await df.refresh()
        except Exception as exc:  # noqa: BLE001
            _LOGGER.info("device_features.refresh: %s", exc)
        flags = {f.name: getattr(df, f.name, None) for f in dataclasses.fields(df)}
    return web.json_response({"supported": sorted(k for k, v in flags.items() if v is True)})


async def handle_status_raw(_request: web.Request) -> web.Response:
    """Read-only: every field the status trait carries, and every property it
    derives. For answering "does this robot even report X?" without guessing --
    same purpose as /api/raw_map and /api/rooms_raw.
    """
    async with STATE._lock:
        props = await STATE.props()
        await props.status.refresh()
        st = props.status
        out: dict[str, typing.Any] = {}
        try:
            out["fields"] = st.as_dict()
        except Exception as exc:  # noqa: BLE001
            out["fields"] = {"error": str(exc)}
        derived: dict[str, typing.Any] = {}
        for name in dir(type(st)):
            if name.startswith("_") or not isinstance(getattr(type(st), name, None), property):
                continue
            try:
                val = getattr(st, name)
            except Exception as exc:  # noqa: BLE001
                derived[name] = f"<raised {exc.__class__.__name__}>"
                continue
            derived[name] = getattr(val, "name", None) or (
                val if isinstance(val, (int, float, str, bool, type(None))) else repr(val))
        out["derived"] = derived
    return web.json_response(out, dumps=lambda o: json.dumps(o, default=str, ensure_ascii=False))


PROBE_COMMANDS = (
    "GET_DOCK_INFO",
    "GET_DUST_COLLECTION_SWITCH_STATUS",
    "GET_HANDLE_LEAK_WATER_STATUS",
    "GET_WASH_TOWEL_PARAMS",
    "GET_WASH_DEBUG_PARAMS",
    "GET_WATER_BOX_CUSTOM_MODE",
    "GET_WASH_WATER_TEMPERATURE",
    "GET_DUST_COLLECTION_MODE",
    "GET_SMART_WASH_PARAMS",
)


async def handle_raw_command(request: web.Request) -> web.Response:
    """Send one command verbatim. Off unless RR_RAW_CMD=1.

    For bisecting a payload the robot rejects with nothing but
    `{'code': -10007, 'message': 'invalid params'}`. Being able to replay a capture
    byte-for-byte and then change one field at a time is the difference between
    knowing which field is wrong and guessing. It can start the vacuum, hence the
    flag and hence not being wired into the UI.

    Body: {"method": "...", "params": <whatever the command takes>}
    """
    if os.environ.get("RR_RAW_CMD", "").strip() not in ("1", "true", "yes"):
        raise web.HTTPForbidden(reason="set RR_RAW_CMD=1 to enable")
    body = await request.json()
    method = str(body.get("method") or "")
    if not method:
        raise web.HTTPBadRequest(reason="method required")
    params = body.get("params")
    _LOGGER.info("raw %s <- %s", method, json.dumps(params, separators=(",", ":"), default=str))
    async with STATE._lock:
        props = await STATE.props()
        try:
            result = await props.command.send(method, params)
        except Exception as exc:  # noqa: BLE001 -- the rejection IS the answer here
            return web.json_response({"ok": False, "error": str(exc)}, status=200)
    return web.json_response({"ok": True, "result": result})


async def handle_probe(_request: web.Request) -> web.Response:
    """Read-only: send every GET the library knows about that we never use, and
    return whatever comes back.

    For the same question /api/status_raw answers, one level out: the status trait
    is not the only thing the robot will talk about. The dock's tanks are not in
    it -- pulling the clean water tank, the dirty water tank and the detergent
    cartridge moved no status field at all, and the library only expresses those
    parts as *errors* (RoborockDockErrorCode.water_empty, .waste_water_tank_full),
    which the dock plausibly only raises when it tries to use them. So: ask the
    dock directly. A command this robot does not implement is logged and comes
    back null, which is itself the answer.

    Result on this device (Saros 10R), so it need not be asked twice: none of
    these carry tank or cartridge presence. get_dock_info, the one that sounds
    like it would, is not implemented and comes back null; the rest return only
    settings (modes, intervals, temperature). Combined with the status trait not
    moving, the conclusion is that fitting and removing the clean water tank, the
    dirty water tank and the detergent cartridge is not observable while the dock
    is idle. Contrast the robot's own dust bin, which the robot does detect
    itself and reports in `rss`.

    What is left untested is the use-time path: whether starting a mop wash with
    a tank out raises dock_error_status 38/39. That needs the machine to run, so
    it is the owner's call, not something to probe from here.
    """
    out: dict[str, typing.Any] = {}
    async with STATE._lock:
        props = await STATE.props()
        cmd = cmd_of(props)
        for name in PROBE_COMMANDS:
            command = getattr(RoborockCommand, name, None)
            if command is None:
                out[name.lower()] = "<not in this library>"
                continue
            out[name.lower()] = _first(await _raw(cmd, command))
    return web.json_response(out, dumps=lambda o: json.dumps(o, default=str, ensure_ascii=False))


async def handle_diag(_request: web.Request) -> web.Response:
    """Diagnostic: compare room ids across sources to debug map selection."""
    async with STATE._lock:
        props = await STATE.props()
        await props.rooms.refresh()
        await props.map_content.refresh()
        md = props.map_content.map_data
        room_map_ids = sorted(int(s) for s in props.rooms.room_map.keys())
        md_rooms = getattr(md, "rooms", None)
        map_room_ids = sorted(int(k) for k in md_rooms.keys()) if isinstance(md_rooms, dict) else None
        proj = _map_projector(md)
        bboxes = _room_bboxes(md)
        overlays = _room_overlays(md)
        out = {
            "room_map_ids": room_map_ids,
            "map_data_rooms_present": md_rooms is not None,
            "map_data_room_ids": map_room_ids,
            "map_projector_ok": proj is not None,
            "image_present": getattr(md, "image", None) is not None if md else False,
            "bbox_ids": sorted(bboxes.keys()),
            "overlay_ids": sorted(o["segment_id"] for o in overlays),
            "calibration_present": _image_to_vacuum(md) is not None,
        }
    return web.json_response(out)


async def handle_map_png(request: web.Request) -> web.Response:
    """The current map, rendered.

    Deliberately does NOT go and read a new map. It used to, on ?fresh=1, and the
    page asked for that once a second while a job was running -- ~400 KB over MQTT
    per second per open tab, which was the single heaviest thing the panel did.
    Whether the map is worth re-reading is now the sampler's decision, and the page
    is told over the socket when it has changed (see push_map). ?fresh=1 is still
    honoured for a hand-driven refresh, through the shared TTL so a burst of tabs
    costs one read.
    """
    moved = False
    async with STATE._lock:
        props = await STATE.props()
        content = props.map_content
        fresh = request.query.get("fresh") == "1"
        if fresh and time.time() - _MAP_READ_AT[0] > MAP_TTL_FRESH:
            _drop_map_ttl()
        if content.image_content is None or fresh:
            moved = await _refresh_map(props)
        # Running: the live map's own path, which is this job's. Idle: the last
        # finished run's, pinned from its own saved map.
        image = _display_render(content.raw_api_response) or content.image_content
    if moved:
        await push_map()          # outside the lock: see _read_timeline
    else:
        _sync_map_token()         # keep the token in step with what was just drawn
    stale = False
    if not image:
        # The vacuum republishes its map while it relocalises at the start of a run,
        # and for a moment there is nothing to render. 404ing then broke the picture
        # on screen -- the browser drops a failed image and shows its alt text -- for
        # a gap of a few seconds. The last good render is a far better answer to
        # "show me the map" than no answer, so it is kept and served, marked stale so
        # it is never cached as though it were current.
        image, stale = _LAST_MAP_PNG[0], True
    if not image:
        raise web.HTTPNotFound(reason="no map image yet")
    if not stale:
        _LAST_MAP_PNG[0] = image
    # A versioned URL names one specific picture -- the version only moves when the
    # bytes do -- so it is safe to cache hard, which is what makes a re-render of
    # the page or a second tab cost nothing. Without a version it is "whatever is
    # current", which must not be cached at all. A stale answer is never cacheable:
    # the real picture is seconds away.
    cache = "no-store" if (stale or not request.query.get("v")) \
        else "public, max-age=31536000, immutable"
    headers = {"Cache-Control": cache}
    if stale:
        headers["X-Map-Stale"] = "1"
    kind = "image/webp" if image[:4] == b"RIFF" else "image/png"
    return web.Response(body=image, content_type=kind, headers=headers)


async def handle_settings(_request: web.Request) -> web.Response:
    async with STATE._lock:
        props = await STATE.props()
        out: dict[str, typing.Any] = {}
        await props.dnd.refresh()
        out["dnd"] = {
            "enabled": props.dnd.enabled,
            "start_hour": props.dnd.start_hour, "start_minute": props.dnd.start_minute,
            "end_hour": props.dnd.end_hour, "end_minute": props.dnd.end_minute,
        }
        await props.sound_volume.refresh()
        out["volume"] = props.sound_volume.volume
        for key, trait in (("child_lock", props.child_lock), ("led", props.led_status),
                           ("flow_led", props.flow_led_status)):
            if trait is None:
                out[key] = None
                continue
            try:
                await trait.refresh()
                raw = getattr(trait, "lock_status", None)
                raw = raw if raw is not None else getattr(trait, "status", None)
                out[key] = {"supported": True, "enabled": bool(raw)}
            except Exception as exc:  # noqa: BLE001
                out[key] = {"supported": True, "error": str(exc)}
    return web.json_response(out)


def cmd_of(props: typing.Any) -> typing.Any:
    return props.command


async def _raw(cmd: typing.Any, command: typing.Any, params: typing.Any = None) -> typing.Any:
    """Send a command and return its result, or None if the robot rejects it."""
    try:
        return await (cmd.send(command, params) if params is not None else cmd.send(command))
    except Exception as exc:  # noqa: BLE001
        _LOGGER.info("%s rejected: %s", command, exc)
        return None


def _first(value: typing.Any) -> typing.Any:
    """Robot replies are usually [payload]; unwrap a single-element list."""
    if isinstance(value, list) and len(value) == 1:
        return value[0]
    return value


async def handle_dock(_request: web.Request) -> web.Response:
    async with STATE._lock:
        props = await STATE.props()
        out: dict[str, typing.Any] = {}
        if props.dust_collection_mode is not None:
            try:
                await props.dust_collection_mode.refresh()
                out["dust_collection"] = {
                    "supported": True,
                    "mode": props.dust_collection_mode.mode,
                    "options": {e.name: e.value for e in RoborockDockDustCollectionModeCode if e.name != "unknown"},
                }
            except Exception as exc:  # noqa: BLE001
                out["dust_collection"] = {"supported": True, "error": str(exc)}
        else:
            # 4.20 exposes no trait for this model, but the robot answers the
            # command directly -- so ask it rather than reporting "unsupported".
            raw = _first(await _raw(cmd_of(props), RoborockCommand.GET_DUST_COLLECTION_MODE))
            mode = raw.get("mode") if isinstance(raw, dict) else raw
            out["dust_collection"] = None if mode is None else {
                "supported": True, "mode": mode,
                "options": {e.name: e.value for e in RoborockDockDustCollectionModeCode if e.name != "unknown"},
            }
        if props.wash_towel_mode is not None:
            try:
                await props.wash_towel_mode.refresh()
                out["wash_towel"] = {
                    "supported": True,
                    "mode": props.wash_towel_mode.wash_mode,
                    "options": {e.value: e.code for e in WashTowelModes},
                }
            except Exception as exc:  # noqa: BLE001
                out["wash_towel"] = {"supported": True, "error": str(exc)}
        else:
            raw = _first(await _raw(cmd_of(props), RoborockCommand.GET_WASH_TOWEL_MODE))
            mode = raw.get("wash_mode") if isinstance(raw, dict) else raw
            out["wash_towel"] = None if mode is None else {
                "supported": True, "mode": mode,
                "options": {e.value: e.code for e in WashTowelModes},
            }
        if props.smart_wash_params is not None:
            try:
                await props.smart_wash_params.refresh()
                out["smart_wash"] = {
                    "supported": True,
                    "smart_wash": props.smart_wash_params.smart_wash,
                    "wash_interval": props.smart_wash_params.wash_interval,
                }
            except Exception as exc:  # noqa: BLE001
                out["smart_wash"] = {"supported": True, "error": str(exc)}
        else:
            out["smart_wash"] = _first(await _raw(cmd_of(props), RoborockCommand.GET_SMART_WASH_PARAMS))

        # Neither of these has a trait in 4.20 or 6.2.1 -- raw is the only route.
        out["dryer"] = _first(await _raw(cmd_of(props), RoborockCommand.APP_GET_DRYER_SETTING))
        out["water_temperature"] = _first(await _raw(cmd_of(props), RoborockCommand.GET_WASH_WATER_TEMPERATURE))
    return web.json_response(out)


# Below this, a consumable is worth acting on rather than noting. The rows in the
# sheet mark anything under 10% as "low" -- that is the amber bar you read once
# you are already looking. This is the threshold for interrupting you about it.
WEAR_ALERT_PCT = 5


def _consumable_levels(props: typing.Any) -> list[dict]:
    """Every consumable's remaining life, from the trait. No RPC.

    The counters arrive as pushed datapoints (MAIN_BRUSH_WORK_TIME and friends are
    RoborockDataProtocol codes), so the trait is current without asking -- which is
    what lets the status payload carry them and the dock button wear a badge
    without the panel fetching anything.
    """
    out = []
    for key, (_attr, field, limit) in CONSUMABLES.items():
        used = getattr(props.consumables, field, None) or 0
        limit_s = limit if limit else 0
        remaining_pct = max(0, round(100 * (limit_s - used) / limit_s)) if limit_s else None
        hours_left = max(0, round((limit_s - used) / 3600)) if limit_s else None
        out.append({"key": key, "used_hours": round(used / 3600, 1),
                    "remaining_pct": remaining_pct, "hours_left": hours_left})
    return out


def _wear_alert(props: typing.Any) -> dict:
    """What is nearly worn out, for the badge on the dock button."""
    levels = [c for c in _consumable_levels(props) if c["remaining_pct"] is not None]
    low = [c for c in levels if c["remaining_pct"] <= WEAR_ALERT_PCT]
    return {
        "threshold": WEAR_ALERT_PCT,
        "min_pct": min((c["remaining_pct"] for c in levels), default=None),
        "low": [{"key": c["key"], "remaining_pct": c["remaining_pct"]} for c in low],
    }


async def handle_maintenance(_request: web.Request) -> web.Response:
    async with STATE._lock:
        props = await STATE.props()
        await props.consumables.refresh()
        await props.clean_summary.refresh()
        try:
            await props.network_info.refresh()
            net = {"ip": props.network_info.ip, "ssid": props.network_info.ssid,
                   "rssi": props.network_info.rssi, "mac": props.network_info.mac}
        except Exception as exc:  # noqa: BLE001
            net = {"error": str(exc)}

        consumables = _consumable_levels(props)

        cs = props.clean_summary
        out = {
            "consumables": consumables,
            "network": net,
            "summary": {
                "clean_time_hours": round((cs.clean_time or 0) / 3600, 1),
                "clean_area": round((cs.clean_area or 0) / 1_000_000, 1),  # mm² -> m²
                "clean_count": cs.clean_count,
                "dust_collection_count": cs.dust_collection_count,
            },
        }
    return web.json_response(out)


_LAST_RUN_CACHE: dict[int, dict] = {}


def _last_run_cached() -> dict | None:
    """The last run as most recently read, without an RPC.

    _resume_plan is synchronous and runs on the 1s status poll, so it cannot go
    and fetch a clean record itself. _last_run keeps exactly one entry here,
    refreshed by /api/state, which is enough: the gate only needs to know
    whether the run that just ended finished.
    """
    return next(iter(_LAST_RUN_CACHE.values()), None)


async def _last_run(props: typing.Any) -> dict | None:
    """Facts about the most recent run that the device actually records.

    Crucially this does NOT include the suction / water / mop-route settings --
    the firmware simply does not keep them. What it does keep is wash_count,
    and a run that washed its pads was unambiguously mopping. That single bit is
    enough to warn when a resume is about to go out vacuum-only behind a run
    that was mopping.
    """
    try:
        await props.clean_summary.refresh()
        ids = props.clean_summary.records or []
        if not ids:
            return None
        rid = int(ids[0])
        if rid in _LAST_RUN_CACHE:
            return _LAST_RUN_CACHE[rid]
        # Through the shared record cache, not a second get_clean_record of its
        # own: the history reads the same record, and that cache survives a
        # restart -- so on a cold /api/state this is usually no command at all.
        raw = await _clean_record(props, rid)
        if not isinstance(raw, dict):
            return None
        washes = _int_or_none(raw.get("wash_count")) or 0
        complete = _int_or_none(raw.get("complete"))
        clean_type = _int_or_none(raw.get("clean_type"))
        out = {
            "begin": raw.get("begin"),
            "end": raw.get("end"),
            "duration_sec": _int_or_none(raw.get("duration")) or 0,
            "duration_min": round((_int_or_none(raw.get("duration")) or 0) / 60),
            "area": round((_int_or_none(raw.get("area")) or 0) / 1_000_000, 1),
            "complete": complete,
            "error": _int_or_none(raw.get("error")) or 0,
            "clean_type": clean_type,
            "clean_type_label": _CLEAN_TYPE_LABELS.get(clean_type),
            "wash_count": washes,
            # The only mode fact the record preserves.
            "mopped": washes > 0,
            "finish_reason": _enum_label_safe("RoborockFinishReason", raw.get("finish_reason")),
            # Whether this run finished what it was asked to, and why it stopped
            # if it did not. Drives both the wording and the resume gate.
            "finish": jobs_mod.describe_finish(_int_or_none(raw.get("finish_reason")), complete),
        }
        # Only a run that has ENDED is cacheable. While it is still going, or in
        # the seconds before the firmware finalises the record, `complete` is 0 and
        # every verdict derived from it -- interrupted, the resume gate, the rooms
        # outstanding -- is wrong. Freezing that made a finished clean go on
        # offering to resume itself until the next run or a restart.
        if _record_settled(raw):
            _LAST_RUN_CACHE.clear()
            _LAST_RUN_CACHE[rid] = out
        return out
    except Exception:  # noqa: BLE001 -- history is a nicety, never fatal
        _LOGGER.debug("last-run lookup failed", exc_info=True)
        return None


def _enum_label_safe(enum_name: str, value: typing.Any) -> str | None:
    val = _int_or_none(value)
    if val is None:
        return None
    try:
        from roborock.data.v1 import v1_code_mappings as cm

        member = getattr(cm, enum_name)(val)
        return member.name if int(member.value) == val else str(val)
    except Exception:  # noqa: BLE001
        return str(val)


async def handle_records(_request: web.Request) -> web.Response:
    """Clean history. ?raw=1 returns the device's unparsed record payload.

    CleanRecord drops any field its dataclass does not declare, so the raw view
    is the only way to see whether the firmware reports things like the suction
    or water setting a past run actually used.
    """
    raw = _request.query.get("raw") in ("1", "true", "yes")
    async with STATE._lock:
        props = await STATE.props()
        await props.clean_summary.refresh()
        ids = (props.clean_summary.records or [])[:15]
        if raw:
            out = []
            for rid in ids[:3]:
                try:
                    out.append(await props.command.send(RoborockCommand.GET_CLEAN_RECORD, [int(rid)]))
                except Exception as exc:  # noqa: BLE001
                    out.append({"id": rid, "error": str(exc)})
            return web.json_response({"raw": out})
        records = []
        for rid in ids:
            try:
                rec = await props.clean_summary.get_clean_record(int(rid))
                records.append({
                    "begin": rec.begin, "end": rec.end,
                    "duration_sec": rec.duration or 0,
                    "duration_min": round((rec.duration or 0) / 60),
                    "area": round((rec.area or 0) / 1_000_000, 1),  # mm² -> m²
                    "complete": rec.complete, "error": str(rec.error),
                })
            except Exception as exc:  # noqa: BLE001
                _LOGGER.info("clean record %s failed: %s", rid, exc)
    return web.json_response({"records": records})


# Codes the firmware reports on each run. clean_type is confirmed: eight runs
# were read off the phone's own history list and every label matched (1 = Total,
# 2 = Zone, 3 = Pièce). start_type is not, so it is still shown next to its raw
# code. Same rule as the state table: never present an unverified code as though
# it were confirmed for this model.
_CLEAN_TYPE_LABELS = {1: "Full clean", 2: "Zone", 3: "Room"}
_START_TYPE_LABELS = {2: "App", 13: "Robot button", 12: "Dock", 1: "Schedule"}


async def _clean_record_raw(props: typing.Any, record_id: int) -> dict | None:
    """One clean record, unabridged.

    `clean_summary.get_clean_record()` maps onto a dataclass that silently drops
    fields it does not declare -- which is most of them. The interesting parts of
    a run (avoid_count, wash_count, clean_type, finish_reason) only survive if we
    read the raw payload.
    """
    try:
        result = await props.command.send(RoborockCommand.GET_CLEAN_RECORD, [int(record_id)])
    except Exception as exc:  # noqa: BLE001
        _LOGGER.info("clean record %s failed: %s", record_id, exc)
        return None
    if isinstance(result, list):
        result = result[0] if result else None
    return result if isinstance(result, dict) else None


# A finished clean record never changes: `begin` is its id and every other field
# is written when the run ends. Each one costs a get_clean_record round trip over
# MQTT, so drawing the 20-run history list was 21 commands to the vacuum *every
# time the sheet was opened* -- which is what made it slow. Cached by id, on the
# server and therefore for every client, and kept across a restart (see
# _hcache_save), so a load fetches only ids nothing has seen yet: normally the one
# run that has happened since, or nothing at all.
_CLEAN_RECORD_CACHE: dict[int, dict] = {}
_CLEAN_RECORD_KEEP = 240
# How long after a run's `end` its record is trusted to be final.
#
# `end` on its own is NOT that signal: the firmware fills a record's fields
# progressively, so it can carry an end timestamp while `complete` is still 0.
# Caching then froze "this run was interrupted" for a run that went on to finish
# perfectly -- and everything derived from it (the resume gate, the rooms
# outstanding) with it. A completed zone clean kept offering to resume nine rooms
# it had never been asked to touch, until the next run or a restart.
#
# The run's saved MAP has the same property, for the same reason, and 60s was
# already the window used there.
_RECORD_SETTLE = 60.0


def _record_settled(rec: dict | None) -> bool:
    end = int((rec or {}).get("end") or 0)
    return bool(end) and (time.time() - end) > _RECORD_SETTLE


# --- keeping the history caches across a restart --------------------------
# Everything above is immutable per run, so throwing it away when the process
# restarts means the next person to open the history pays for all of it again.
# One small file holds the lot; it is written from a dirty flag by the warmer
# rather than on every insert, so a cold history load does not also become twenty
# file writes. The event logs are the bulky part, so only the newest few are kept
# on disk -- the warmer refills the rest in the background anyway.
HISTORY_CACHE_PATH = os.environ.get("RR_HISTORY_CACHE", str(HERE / "history_cache.json"))
_HCACHE_EVENTS_PERSIST = 12
_hcache_dirty = False


def _hcache_touch() -> None:
    global _hcache_dirty
    _hcache_dirty = True


def _hcache_load() -> None:
    try:
        data = json.loads(Path(HISTORY_CACHE_PATH).read_text())
    except FileNotFoundError:
        return
    except Exception as exc:  # noqa: BLE001 -- a cache is never worth failing over
        _LOGGER.warning("history cache unreadable (%s); starting cold", exc)
        return
    for key, target in (("records", _CLEAN_RECORD_CACHE),
                        ("events", _RUN_EVENTS_CACHE),
                        ("obstacles", _RECORD_MAP_CACHE)):
        for begin, value in (data.get(key) or {}).items():
            try:
                target[int(begin)] = value
            except (TypeError, ValueError):
                continue
    # The trace the map draws while nothing is running. Kept so a restart does not
    # spend a ~400 KB read re-deriving a path that has not changed.
    # Only a trace in the ordered form (pts): an older one has the drive legs drawn
    # as cleaning, so it is dropped and re-read from the run map.
    trace = data.get("trace") or {}
    if trace.get("pts"):
        _LAST_TRACE.update(begin=trace.get("begin"), stale=bool(trace.get("stale")),
                           pts=[tuple(p) for p in trace["pts"]],
                           goto=[[tuple(p) for p in seg] for seg in (trace.get("goto") or [])])
    _LOGGER.info("history cache: %d records, %d logs, %d detection sets from %s",
                 len(_CLEAN_RECORD_CACHE), len(_RUN_EVENTS_CACHE),
                 len(_RECORD_MAP_CACHE), HISTORY_CACHE_PATH)


def _hcache_save() -> None:
    global _hcache_dirty
    if not _hcache_dirty:
        return
    _hcache_dirty = False
    newest = sorted(_RUN_EVENTS_CACHE, reverse=True)[:_HCACHE_EVENTS_PERSIST]
    payload = {
        "records": {str(k): v for k, v in _CLEAN_RECORD_CACHE.items()},
        "events": {str(k): _RUN_EVENTS_CACHE[k] for k in newest},
        "obstacles": {str(k): v for k, v in _RECORD_MAP_CACHE.items()},
        "trace": dict(_LAST_TRACE),
    }
    try:
        path = Path(HISTORY_CACHE_PATH)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False))
        tmp.replace(path)
    except Exception as exc:  # noqa: BLE001
        _LOGGER.warning("could not write history cache: %s", exc)


async def _clean_record(props: typing.Any, record_id: int) -> dict | None:
    """One clean record, from the cache where the run has already finished."""
    rid = int(record_id)
    hit = _CLEAN_RECORD_CACHE.get(rid)
    if hit is not None:
        return hit
    rec = await _clean_record_raw(props, rid)
    # Only a SETTLED record is final -- see _RECORD_SETTLE. A record still being
    # written is re-read next time rather than frozen with a zero duration, no
    # outcome, or a `complete` the firmware has not got round to setting.
    if rec and _record_settled(rec):
        _CLEAN_RECORD_CACHE[rid] = rec
        _hcache_touch()
        if len(_CLEAN_RECORD_CACHE) > _CLEAN_RECORD_KEEP:
            # Ids are begin timestamps, so "smallest first" is "oldest first".
            for stale in sorted(_CLEAN_RECORD_CACHE)[: len(_CLEAN_RECORD_CACHE) - _CLEAN_RECORD_KEEP]:
                _CLEAN_RECORD_CACHE.pop(stale, None)
    return rec


def _history_entry(rec: dict) -> dict:
    begin = int(rec.get("begin") or 0)
    end = int(rec.get("end") or 0)
    duration = int(rec.get("duration") or 0)
    # Areas are reported in mm^2. `area` is what was covered; `cleaned_area`
    # counts repeat passes, so it exceeds `area` on a two-pass run.
    area = (rec.get("area") or 0) / 1_000_000
    cleaned = (rec.get("cleaned_area") or 0) / 1_000_000
    complete = int(rec.get("complete") or 0)
    error = int(rec.get("error") or 0)

    finish = jobs_mod.describe_finish(_int_or_none(rec.get("finish_reason")), complete)
    if error:
        outcome = "error"
    else:
        # The phone picks the green tick purely on `complete`; a run the user
        # stopped is reported as stopped rather than as a failure.
        outcome = finish["outcome"]

    return {
        "begin": begin,
        "end": end,
        "duration_sec": duration,
        "duration_min": round(duration / 60),
        "area_m2": round(area, 1),
        "cleaned_area_m2": round(cleaned, 1),
        "outcome": outcome,
        "complete": complete,
        "error": error,
        "error_text": _enum_label_safe("RoborockErrorCode", error) if error else None,
        "error_info": jobs_mod.describe_error(error) if error else None,
        "avoid_count": rec.get("avoid_count"),
        "wash_count": rec.get("wash_count"),
        "clean_times": rec.get("clean_times"),
        "extra_time_sec": rec.get("extra_time"),
        # Despite the name this is a COUNT of dock empties, not a status: the
        # phone renders it as "N recueil(s) de la poussière par la station" and
        # it matches the number of dust-collection events in the timeline.
        "dust_collections": _int_or_none(rec.get("dust_collection_status")) or 0,
        # Raw codes plus a best-effort label; the UI shows the code too.
        "clean_type": rec.get("clean_type"),
        "clean_type_label": _CLEAN_TYPE_LABELS.get(rec.get("clean_type")),
        "start_type": rec.get("start_type"),
        "start_type_label": _START_TYPE_LABELS.get(rec.get("start_type")),
        "finish_reason": rec.get("finish_reason"),
        "finish_reason_label": _enum_label_safe("RoborockFinishReason", rec.get("finish_reason")),
        "finish": finish,
        "map_flag": rec.get("map_flag"),
        # The record keeps no suction/water setting, but a run that washed its
        # pads was unambiguously mopping. Same inference _last_run relies on.
        "mopped": (rec.get("wash_count") or 0) > 0,
    }


async def handle_history(request: web.Request) -> web.Response:
    """Run history plus lifetime totals, for the overview tab.

    Reads raw records so the per-run detail the app shows (obstacles avoided,
    mop washes, pass count) is preserved.

    The record list itself is always re-read from the vacuum (one command), so a
    run that has happened since the last call is never missed; only the records
    themselves come from _CLEAN_RECORD_CACHE. `fresh=1` empties that cache first,
    which is what the reload button in the history sheet is for.
    """
    try:
        limit = max(1, min(50, int(request.query.get("limit", "20"))))
    except ValueError:
        limit = 20
    if request.query.get("fresh"):
        # The records and the event logs, which are a handful of cheap commands to
        # rebuild. NOT the record maps: those are ~400 KB each over MQTT, and a
        # finished run's detections genuinely cannot change -- re-reading twenty of
        # them would make the reload button the slowest thing in the panel.
        _CLEAN_RECORD_CACHE.clear()
        _RUN_EVENTS_CACHE.clear()
        _SEGMENT_NAMES_CACHE.update(at=0.0, names=None)

    # STATE._lock is the one MQTT queue: everything in the panel -- the 1s status
    # poll, the map, a tap on a room -- waits behind whoever holds it. Holding it
    # across twenty get_clean_record calls froze the whole page for as long as they
    # took, which is what made a cold history load feel like the app had hung. So
    # it is taken for the record LIST, released, and then taken again per record
    # that is actually missing; a warm cache takes it once and reads nothing.
    async with STATE._lock:
        props = await STATE.props()
        await props.clean_summary.refresh()
        cs = props.clean_summary
        ids = list(cs.records or [])[:limit]
        totals = {
            "clean_time_hours": round((cs.clean_time or 0) / 3600, 1),
            "area_m2": round((cs.clean_area or 0) / 1_000_000, 1),
            "count": cs.clean_count,
            "dust_collection_count": cs.dust_collection_count,
        }

    runs = []
    for rid in ids:
        rec = _CLEAN_RECORD_CACHE.get(int(rid))
        if rec is None:
            async with STATE._lock:
                rec = await _clean_record(await STATE.props(), rid)
        if rec:
            runs.append(_history_entry(rec))

    # Aggregates over the window we actually fetched, so the UI need not re-derive.
    window = {
        "runs": len(runs),
        "area_m2": round(sum(r["area_m2"] for r in runs), 1),
        "duration_min": sum(r["duration_min"] for r in runs),
        "avoided": sum(r["avoid_count"] or 0 for r in runs),
        "washes": sum(r["wash_count"] or 0 for r in runs),
        "interrupted": sum(1 for r in runs if r["outcome"] != "completed"),
    }
    return web.json_response({"totals": totals, "window": window, "runs": runs})


# --------------------------------------------------------------------------- #
# Job event timeline
# --------------------------------------------------------------------------- #
# Event vocabulary for get_clean_detail / get_rt_clean_detail. Every entry below
# was confirmed by opening the same run in the phone app over adb and lining the
# events up against what it rendered, event by event -- see the table in
# NOTES.md. Kinds and fields that the phone does not surface are decoded only as
# far as is defensible and marked hidden, so they stay available behind the
# "all events" toggle without being given invented names.
#
# key      -- stable machine name; the UI maps it to display text
# vis      -- shown by default, as the phone does
_EV_KINDS: dict[int, tuple[str, bool]] = {
    25: ("job_start", True),        # 251=start_type 252=clean_type + job modes
    12: ("job_end", True),          # 121=finish code (own vocabulary, != finish_reason)
    41: ("room_start", False),      # 2=segment; the phone uses room_settings instead
    42: ("room_settings", True),    # 2=segment 421=scope 422=room-type id + modes
    43: ("room_end", False),        # 2=segment; not always emitted
    55: ("mop_installed", True),    # 7
    56: ("mop_removed", True),      # 7
    52: ("wash_start", True),       # 521=wash mode
    53: ("wash_end", False),        # 531
    51: ("dock_return_wash", False),  # 511,512
    54: ("dust_collect", True),     # 541=mode 542=-1
    22: ("mode_change", False),     # 3+221 (suction+reason) or 4+222 (water+reason)
    13: ("clean_start", True),      # 131: 16=start, 14=resume after dock
    31: ("error", True),            # 311=major 312=minor
    11: ("state", False),           # 111=from 112=to 1=battery
    23: ("run_stats", False),       # 234 ~= clean time
    24: ("map_info", False),        # 244/245 map ids, 243
    21: ("position", False),
    44: ("mop_detail", False),
}

# Inside i=25 / i=42 (and i=22 for whichever one changed) the mode fields are
# offsets from the RPC code space, not the codes themselves.
_EV_FAN, _EV_WATER, _EV_MOP, _EV_PASSES = "3", "4", "5", "6"
_EV_SEGMENT, _EV_BATTERY = "2", "1"

# Mirrors the option tables /api/state exposes (which are library-derived for
# this model). Codes outside these render as their raw number.
_FAN_NAMES = {101: "quiet", 102: "balanced", 103: "turbo", 104: "max", 108: "max_plus",
              109: "off_raise_main_brush", 110: "smart_mode", 106: "custom"}
_MOP_NAMES = {300: "standard", 301: "deep", 303: "deep_plus", 304: "fast",
              306: "smart_mode", 302: "custom"}
# The phone renders water on a 1-30 scale: the plugin ships ic_water_flow_1 ..
# ic_water_flow_30 and the slider codes are 220 + level. Only the two legacy
# discrete codes below have been seen on screen, so the rest stay unresolved
# rather than being mapped by guesswork.
_WATER_LEVELS_CONFIRMED = {200: 0, 202: 15}
# 521 on a wash event / 541 on a dust event. Confirmed from the phone: 2 is
# "En profondeur", 10 is "Intelligent", 0 is "Léger", 4 (dust) is "Max.".
# Codes seen on a wash_start's field 521. 0/2/10 match WashTowelModes in the
# library (light / deep / smart).
#
# 3 is `balanced` -- the official app shows it as Normal, confirmed by the owner.
# Worth recording that it was guessed wrong first: the library's table has no 3 (its
# balanced is 1, its super_deep is 8) and the plugin ships icon_wash_mode_0..3, from
# which "3 = the thorough end of a 0..3 scale" looked reasonable and was not. The
# codes on this field are simply not WashTowelModes' codes, so nothing about them
# should be inferred from that table again.
_WASH_MODE_NAMES = {0: "light", 2: "deep", 3: "balanced", 10: "smart"}
_DUST_MODE_NAMES = {1: "light", 2: "balanced", 4: "max", 10: "smart"}
# 221 on an i=22 suction change. 4 is the one the phone surfaces, as
# "La puissance d'aspiration diminuera en mode NPD".
_MODE_CHANGE_REASONS = {0: None, 2: "carpet", 4: "dnd"}


def _water_level(code: int | None) -> int | None:
    """Water flow on the phone's 1-30 scale, or None when it cannot be derived."""
    if code is None:
        return None
    if code in _WATER_LEVELS_CONFIRMED:
        return _WATER_LEVELS_CONFIRMED[code]
    if 221 <= code <= 250:
        return code - 220
    return None


def _ev_settings(fields: dict) -> dict | None:
    """The fan / water / mop / passes block carried by i=25, i=42 and i=22.

    Returns None when the event carries none of them. Each value is reported as
    the real RPC code (offset restored) plus a name where one is known, so the
    UI never has to know about the offsets.
    """
    out: dict[str, typing.Any] = {}
    fan = _int_or_none(fields.get(_EV_FAN))
    water = _int_or_none(fields.get(_EV_WATER))
    mop = _int_or_none(fields.get(_EV_MOP))
    passes = _int_or_none(fields.get(_EV_PASSES))
    if fan is not None:
        out["fan"] = 100 + fan
        out["fan_name"] = _FAN_NAMES.get(100 + fan)
    if water is not None:
        out["water"] = 200 + water
        out["water_level"] = _water_level(200 + water)
    if mop is not None:
        out["mop"] = 300 + mop
        out["mop_name"] = _MOP_NAMES.get(300 + mop)
    if passes is not None:
        out["passes"] = passes
    if not out:
        return None
    # What the phone puts on the card. Derived from water and the brush, not
    # from a field of its own: "Personnalisé" whenever the job is running
    # per-room settings, otherwise vacuum / mop / both.
    if out.get("fan") == 106 or out.get("mop") == 302:
        out["mode"] = "custom"
    elif out.get("water") == 200:
        out["mode"] = "vacuum"
    elif out.get("fan") == 109:
        out["mode"] = "mop"
    elif out.get("water") is not None:
        out["mode"] = "vac_and_mop"
    return out


def _timeline_event(raw: dict, rooms: dict[int, str]) -> dict:
    """One timeline entry, decoded as far as is defensible.

    `e` is a JSON *string* of numbered fields, so it needs a second parse. The
    raw fields are always passed through under `fields` so nothing is hidden
    behind the interpretation.
    """
    kind = _int_or_none(raw.get("i"))
    try:
        fields = json.loads(raw.get("e") or "{}")
    except (json.JSONDecodeError, TypeError):
        fields = {}

    key, visible = _EV_KINDS.get(kind, (f"event_{kind}", False))
    entry: dict[str, typing.Any] = {
        "t": _int_or_none(raw.get("t")),
        "kind": kind,
        "key": key,
        "visible": visible,
        "fields": fields,
    }

    settings = _ev_settings(fields)
    if settings and key in ("job_start", "room_settings"):
        entry["settings"] = settings

    if key == "job_start":
        entry["start_type"] = _int_or_none(fields.get("251"))
        entry["clean_type"] = _int_or_none(fields.get("252"))
        # The first of the two i=25 events carries only {"8": 0} and would
        # otherwise render as a second, empty "job started" row.
        if entry["clean_type"] is None and entry["start_type"] is None:
            entry["visible"] = False
    elif key == "job_end":
        entry["finish_code"] = _int_or_none(fields.get("121"))
    elif key in ("room_start", "room_settings", "room_end"):
        segment = _int_or_none(fields.get(_EV_SEGMENT))
        if segment is not None:
            entry["segment"] = segment
            scope = _int_or_none(fields.get("421"))
            # 421=3 marks a zone, where `2` is a pseudo segment -- 41 for the
            # first zone of the job, counting up from there -- rather than a real
            # room, and the phone labels the row "Zone".
            if scope == 3 or (segment == 41 and segment not in rooms):
                entry["zone"] = True
                entry["room"] = None
            else:
                entry["room"] = rooms.get(segment)
            if scope is not None:
                entry["per_room_settings"] = scope == 1
    elif key == "clean_start":
        # 16 starts the job and the phone shows it as "Nettoyer"; 14 is the
        # robot picking the job back up after a dock excursion, which the phone
        # does not show at all (the 08/01 run fires it three times and none of
        # them appear on screen).
        code = _int_or_none(fields.get("131"))
        entry["resumed"] = code == 14
        if code != 16:
            entry["key"] = key = "clean_resume"
            entry["visible"] = False
    elif key in ("mop_installed", "mop_removed"):
        entry["mop_pad_state"] = _int_or_none(fields.get("7"))
    elif key == "wash_start":
        mode = _int_or_none(fields.get("521"))
        entry["wash_mode"] = mode
        entry["wash_mode_name"] = _WASH_MODE_NAMES.get(mode)
    elif key == "dust_collect":
        mode = _int_or_none(fields.get("541"))
        entry["dust_mode"] = mode
        entry["dust_mode_name"] = _DUST_MODE_NAMES.get(mode)
    elif key == "mode_change":
        reason_code = _int_or_none(fields.get("221"))
        if reason_code is None:
            reason_code = _int_or_none(fields.get("222"))
        entry["reason_code"] = reason_code
        entry["reason"] = _MODE_CHANGE_REASONS.get(reason_code)
        if settings:
            entry["settings"] = settings
        # Only the do-not-disturb clamp is worth a row of its own; carpet
        # lift/drop fires dozens of times in a single run. Whether *this* one is
        # shown is decided in _decode_timeline, which keeps just the first --
        # the clamp fires on every suction change while the window is open, and
        # the phone shows the notice once.
        if entry["reason"] == "dnd":
            entry["visible"] = None
    elif key == "error":
        major = _int_or_none(fields.get("311")) or 0
        minor = _int_or_none(fields.get("312"))
        entry["error"] = jobs_mod.describe_error(major, minor)
    elif key == "state":
        entry["state_from"] = _int_or_none(fields.get("111"))
        entry["state_to"] = _int_or_none(fields.get("112"))
        battery = _int_or_none(fields.get(_EV_BATTERY))
        if battery is not None:
            entry["battery"] = battery
    return entry


def _decode_timeline(events: list, rooms: dict[int, str], *,
                     interrupted: bool = False, running: bool = False) -> list[dict]:
    """Decode, order and cross-link a run's events.

    Cross-linking is what the per-room durations need: `i=43` is not emitted for
    every room, so a room's end is the earliest of its own room_end, the next
    room's start, or the end of the job.

    The last room a run entered usually has no `room_end` of its own -- the
    firmware emits that one unreliably -- so "no room_end" on its own says
    nothing at all. What it means depends entirely on the run, which only the
    caller knows, so it is asked rather than guessed:

      interrupted -- the run is known to have stopped short. Its last room is
                     where it stopped: `unfinished`.
      running     -- a run is under way right now. Its last room is the one the
                     robot is in: `in_progress`.

    Neither, and the last room gets no marker. That is the right answer for a
    run that finished normally, which was previously being told it had been
    interrupted in whichever room it happened to end in -- and, once the resume
    plan started reading this flag, being offered up for a resume it did not
    need.
    """
    decoded = [_timeline_event(e, rooms) for e in events if isinstance(e, dict)]
    decoded.sort(key=lambda e: e["t"] or 0)

    # `visible is None` means "shown, but only the first of its kind": the
    # do-not-disturb clamp re-fires on every suction change while the window is
    # open, and six identical rows say nothing the first one did not.
    seen_once: set[str] = set()
    for e in decoded:
        if e["visible"] is None:
            once_key = f'{e["key"]}:{e.get("reason")}'
            e["visible"] = once_key not in seen_once
            seen_once.add(once_key)

    # The phone times a room from its room_settings row, and rooms it never
    # reached have no row at all -- so the only rooms here are ones it entered.
    room_rows = [e for e in decoded if e["key"] == "room_settings"]
    job_end = next((e["t"] for e in reversed(decoded) if e["key"] == "job_end"), None)
    # Moments the robot stopped cleaning, from its own state transitions. Only
    # the LAST room needs these: every other one is already bounded by the next
    # room's start, and clamping those too would dock them the mop washes and
    # dock trips they legitimately span. The last room has nothing after it but
    # job_end, which is paperwork and can land hours late -- the run of 09/08
    # 13:01 cleaned Entrée for two minutes, paused, sat there all afternoon and
    # only closed the job at 19:16, so the row read "375 min" against a record
    # that says 2. Measured across the last dozen runs, this is identical to the
    # old figure on every healthy one and only bites on that tail.
    stopped_cleaning = [e["t"] for e in decoded
                        if e["key"] == "state" and e.get("state_to") not in _CLEANING_STATES]
    # A job_end settles it whatever the caller believed: the robot has closed the
    # run out, so nothing in it is still in progress.
    if job_end is not None:
        running = False
    for i, row in enumerate(room_rows):
        seg = row.get("segment")
        ends = [e["t"] for e in decoded
                if e["key"] == "room_end" and e.get("segment") == seg and (e["t"] or 0) >= (row["t"] or 0)]
        if i + 1 < len(room_rows):
            ends.append(room_rows[i + 1]["t"])
        # An error stops the room there. Without this the last room of a run
        # that sat faulted until someone noticed absorbs all six of those hours.
        ends += [e["t"] for e in decoded if e["key"] == "error" and (e["t"] or 0) >= (row["t"] or 0)]
        if i == len(room_rows) - 1:
            ends += [t for t in stopped_cleaning if t and t > (row["t"] or 0)]
        if job_end:
            ends.append(job_end)
        ends = [t for t in ends if t and row["t"] and t >= row["t"]]
        if ends:
            row["duration_sec"] = min(ends) - row["t"]
        # Where the run stopped: the LAST room it entered, if that room never
        # reported finishing. Only the last one, because `room_end` is not
        # emitted reliably -- two rooms of the 08/01 run have none and were
        # nonetheless cleaned, so treating every missing room_end as an
        # interruption marked three rooms as the single point of failure.
        #
        # And only when the caller says the run stopped short. Which room that
        # was needs no further test: if the run did not finish, the last room it
        # entered is as far as it got. Whether that room also logged a room_end
        # is beside the point -- the reference run of 18/08 closed Room B out
        # after one second, got stuck, and never went anywhere else, so asking
        # for a missing room_end found no interruption in a run that plainly had
        # one. Only the last row is ever marked: room_end is emitted too
        # unreliably to accuse any earlier room of being where it stopped.
        last = i == len(room_rows) - 1
        row["unfinished"] = bool(interrupted and last)
        row["in_progress"] = bool(running and last)
    return decoded


# --------------------------------------------------------------------------- #
# What a run left undone, read off its own timeline
# --------------------------------------------------------------------------- #
# A room the robot was inside for less than this did not get cleaned; it was
# entered and abandoned. Rooms that genuinely take seconds do not exist -- even
# the smallest one in the reference flat takes two minutes -- so this only ever
# catches a room the run bailed out of.
TODO_MIN_ROOM_SEC = 25

# How close to "done" each verdict is. A room visited twice keeps the verdict of
# its BEST visit: what is still missing after the better of the two attempts is
# what a resume has to make up.
_TODO_RANK = ["never_reached", "interrupted", "too_short", "no_suction", "no_water"]

_TODO_PHRASE = {
    "never_reached": "never reached",
    "interrupted": "interrupted part-way",
    "too_short": "barely entered",
    "no_suction": "cleaned without suction",
    "no_water": "cleaned without water",
}


def _run_interrupted(record: dict | None) -> bool:
    """Did this run stop short of what it set out to do?

    Straight off the record's own two unambiguous fields. Everything else about
    a run is inference; these are not.
    """
    if not record:
        return False
    return not record.get("complete") or bool(record.get("error"))


def _todo_from_timeline(decoded: list[dict], all_segments: set[int], clean_type: int | None,
                        completed: bool = False) -> dict:
    """Which rooms of a decoded run still need the clean that was asked for.

    The map's path trace can only answer "did the robot drive through here".
    The event log also knows what it was *doing* there, which is the difference
    between a room that got its clean and one that only looks like it did:

      * never reached  -- a whole-home run that stopped before getting there
      * interrupted    -- the run ended inside the room, or barely entered it
      * degraded       -- the room ran without the water (or the suction) the
                          job was started with. That is what a clean-water tank
                          fault mid-run looks like: the robot carries on
                          vacuum-only, finishes the house, and the record calls
                          every one of those rooms cleaned.

    Returns {"reasons": {seg: why}, "order": [seg...], "visited": [seg...]}.
    """
    rows = [e for e in decoded
            if e["key"] == "room_settings" and e.get("segment") is not None and not e.get("zone")]
    visited = [r["segment"] for r in rows]

    # What the job was started with, which is the only statement of intent a run
    # makes. A per-room-custom job (fan 106 / mop 302 are markers, not settings)
    # has no single intent, so nothing can be called degraded against it.
    job = next((e.get("settings") for e in decoded
                if e["key"] == "job_start" and e.get("settings")), None) or {}
    custom = job.get("fan") == 106 or job.get("mop") == 302
    want_water = not custom and job.get("water") is not None and job.get("water") != WATER_OFF
    want_suction = not custom and job.get("fan") is not None and job.get("fan") != 109

    def verdict(row: dict) -> str | None:
        """Why this one visit fell short, or None if it did the job."""
        if row.get("unfinished"):
            return "interrupted"
        if (row.get("duration_sec") or 0) < TODO_MIN_ROOM_SEC:
            return "too_short"
        s = row.get("settings") or {}
        if want_water and s.get("water") == WATER_OFF:
            return "no_water"
        if want_suction and s.get("fan") == 109:
            return "no_suction"
        return None

    reasons: dict[int, str] = {}
    order: list[int] = []

    # A run that reached the end of its own plan did not skip a room and did not
    # stop in one. Whatever the event log's shape suggests -- and the last room
    # of every run, healthy or not, looks "open" because room_end is unreliable
    # -- the only thing such a run can still owe is work it did in a lesser mode
    # than it was asked for. Restrict it to that rather than trusting a signal
    # that cannot distinguish the two cases.
    allowed = {"no_water", "no_suction"} if completed else None

    # Rooms with no row at all were never entered. Only a whole-home run was
    # asked to go everywhere -- a room or zone clean going nowhere near the rest
    # of the house is it doing exactly what it was told.
    if clean_type == 1 and not completed:
        for seg in sorted(all_segments - set(visited)):
            reasons[seg] = "never_reached"
            order.append(seg)

    by_seg: dict[int, list[dict]] = {}
    for row in rows:
        by_seg.setdefault(row["segment"], []).append(row)
    for seg, visits in by_seg.items():
        verdicts = [verdict(v) for v in visits]
        if None in verdicts:
            continue  # one of the visits did what was asked; nothing outstanding
        best = max(verdicts, key=_TODO_RANK.index)
        if allowed is not None and best not in allowed:
            continue
        reasons[seg] = best
        order.append(seg)

    # Zone rows were filtered out above because a zone is not a room and none of
    # the room reasoning applies to it. They are still the only record of which
    # zones of a zone job got done, so they are analysed on their own terms and
    # carried alongside: pseudo segment (41, 42, ...) -> why it fell short.
    zrows = [e for e in decoded
             if e["key"] == "room_settings" and e.get("segment") is not None and e.get("zone")]
    zbase = min((r["segment"] for r in zrows), default=None)
    zones: dict[int, str] = {}
    for r in zrows:
        why = ("interrupted" if r.get("unfinished")
               else "too_short" if (r.get("duration_sec") or 0) < TODO_MIN_ROOM_SEC
               else None)
        if why:
            zones[int(r["segment"])] = why
    return {"reasons": reasons, "order": order, "visited": visited,
            "zones": zones, "zone_base": zbase,
            "zones_seen": sorted({int(r["segment"]) for r in zrows})}


async def _run_todo(props: typing.Any, last: dict | None, all_segments: set[int]) -> dict | None:
    """Fetch and analyse the last run's timeline, memoised on its `begin`.

    One get_clean_detail per run, from /api/state, so the 1s status poll can
    read the answer without an RPC of its own.
    """
    begin = _int_or_none((last or {}).get("begin"))
    if begin is None:
        return None
    if begin in _RUN_TODO_CACHE:
        return _RUN_TODO_CACHE[begin]
    # A run that has not ended has nothing outstanding to go back for -- it is
    # still doing it. Deriving a verdict now would read every room it has not
    # reached yet as missed (nine of them, for a zone clean), and the line below
    # would then cache that for the life of the process.
    if not _record_settled(_CLEAN_RECORD_CACHE.get(begin)):
        return None
    try:
        # The run sheet's "déroulement" is the same event log, so this goes through
        # the same cache -- which is on disk, so a cold /api/state normally costs
        # nothing here instead of up to twenty paged commands.
        events = await _run_events(props, begin, _CLEAN_RECORD_CACHE.get(begin))
        if not events:
            return None
        interrupted = _run_interrupted(last)
        decoded = _decode_timeline(events, _map_room_names(props), interrupted=interrupted)
    except Exception:  # noqa: BLE001 -- the coverage fallback still stands
        _LOGGER.debug("run-todo timeline unavailable", exc_info=True)
        return None
    out = _todo_from_timeline(decoded, all_segments, _int_or_none(last.get("clean_type")),
                              completed=not interrupted)
    _RUN_TODO_CACHE.clear()
    _RUN_TODO_CACHE[begin] = out
    _LOGGER.info("run %s todo: %d room(s) outstanding (interrupted=%s, type=%s)",
                 begin, len(out.get("order") or ()), interrupted, last.get("clean_type"))
    return out


def _todo_rooms(todo: dict, scope: set[int] | None) -> tuple[list[int], dict[str, str]]:
    """The outstanding rooms, narrowed to what the run was actually asked for."""
    reasons: dict[int, str] = dict(todo.get("reasons") or {})
    order: list[int] = list(todo.get("order") or [])
    if scope:
        order = [s for s in order if s in scope]
        # A segment clean we launched ourselves knows its own room list, so a
        # room it asked for and never entered is outstanding whatever the
        # record's clean_type says.
        for seg in sorted(scope - set(todo.get("visited") or ())):
            if seg not in order:
                reasons[seg] = "never_reached"
                order.append(seg)
    return order, {str(s): reasons[s] for s in order if s in reasons}


def _todo_reason(notes: dict[str, str]) -> str:
    """One sentence for the logs / API, in the same voice as the coverage one."""
    if not notes:
        return ""
    counts: dict[str, int] = {}
    for why in notes.values():
        counts[why] = counts.get(why, 0) + 1
    bits = [f"{n} {_TODO_PHRASE.get(why, why)}" for why, n in
            sorted(counts.items(), key=lambda kv: _TODO_RANK.index(kv[0]) if kv[0] in _TODO_RANK else 99)]
    n = len(notes)
    return f"{n} room{'' if n == 1 else 's'} did not get the clean that was asked for: " + ", ".join(bits) + "."


# Room names are read with get_multi_maps_list, one command, and every run sheet
# needs them to label its rows. They change only when a room is renamed or the map
# is rebuilt, so they are held briefly rather than re-asked for on every open;
# handle_room_rename drops this so a rename shows up at once rather than in five
# minutes.
_SEGMENT_NAMES_CACHE: dict[str, typing.Any] = {"at": 0.0, "names": None}
_SEGMENT_NAMES_TTL = 300.0


async def _segment_names(props: typing.Any) -> dict[int, str]:
    """{segment_id: name} for labelling timeline rows.

    Prefer get_multi_maps_list: it carries `iot_name` for every segment. The
    rooms trait resolves names through the account room list, which comes back
    empty for some segments here and would render "Room 9" for a room that is
    plainly called Entrée.

    These are the room's names *now*, which is deliberate -- the phone relabels
    historical runs when a room is renamed too, and the run's own `422` field is
    a room-type id, not a name.
    """
    hit = _SEGMENT_NAMES_CACHE["names"]
    if hit is not None and time.monotonic() - _SEGMENT_NAMES_CACHE["at"] < _SEGMENT_NAMES_TTL:
        return dict(hit)
    rooms: dict[int, str] = {}
    try:
        maps = await props.command.send("get_multi_maps_list")
        entries = maps[0] if isinstance(maps, list) and maps else maps
        for info in (entries or {}).get("map_info", []) or []:
            for room in info.get("rooms", []) or []:
                seg = _int_or_none(room.get("id"))
                name = str(room.get("iot_name") or "").strip()
                if seg is not None and name:
                    rooms[seg] = name
            break  # active map only
    except Exception:  # noqa: BLE001 -- names are a nicety
        _LOGGER.debug("room names unavailable for timeline", exc_info=True)
    # A failed read is not cached: it would label every run sheet "Pièce 9" for
    # the next five minutes over one dropped command.
    if rooms:
        _SEGMENT_NAMES_CACHE.update(at=time.monotonic(), names=dict(rooms))
    return rooms


async def _paged_events(props: typing.Any, method: str, params: typing.Callable[[int], typing.Any]) -> tuple[list, int | None]:
    """Walk a `from` / `more` cursor, returning (events, begin).

    Both detail RPCs page the same way. The loop is capped so a firmware that
    never clears `more` cannot spin here forever.
    """
    events: list = []
    begin: int | None = None
    cursor = 0
    for _ in range(20):
        try:
            page = await props.command.send(method, params(cursor))
        except Exception as exc:  # noqa: BLE001
            _LOGGER.info("%s failed at cursor %s: %s", method, cursor, exc)
            break
        if isinstance(page, list):
            page = page[0] if page else None
        if not isinstance(page, dict) or not page.get("events"):
            break
        if begin is None:
            begin = _int_or_none(page.get("begin"))
        events.extend(page["events"])
        if not _int_or_none(page.get("more")):
            break
        cursor += _int_or_none(page.get("num")) or len(page["events"])
    return events, begin


async def handle_timeline(_request: web.Request) -> web.Response:
    """Event timeline of the job in progress.

    get_rt_clean_detail covers only the current run and returns nothing once the
    robot has cleared it. For a finished run use /api/run_detail, which reads
    get_clean_detail and works for runs from weeks ago.
    """
    async with STATE._lock:
        props = await STATE.props()
        rooms = await _segment_names(props)
        events, begin = await _paged_events(
            props, "get_rt_clean_detail", lambda c: {"from": c, "type": 0})
        # A live run: the last room is the one being cleaned, if the robot is
        # still cleaning it. Whether a run that has since STOPPED was interrupted
        # is the clean record's call, not this endpoint's -- that is answered in
        # the history. Read under the same lock rather than taking it twice.
        st = props.status
        running = _is_running(_int_or_none(st.state), st.in_cleaning)

    decoded = _name_zone_rows(_decode_timeline(events, rooms, running=running), begin)
    _note_requested(decoded, begin, rooms)
    return web.json_response({
        "begin": begin,
        "events": decoded,
        "available": bool(decoded),
        "note": "" if decoded else "No job is in progress. Open a run in the history for its timeline.",
    })


# The `déroulement` of a finished run -- the raw get_clean_detail event pages.
# Immutable once the run has ended, and read over a `from`/`more` cursor, so
# opening a run sheet was up to twenty commands for a log that cannot have
# changed since the last time it was read. The RAW events are what is kept, not
# the decoded timeline: decoding folds in the rooms' names *as they are now*, and
# a rename has to reach an already-cached run.
_RUN_EVENTS_CACHE: dict[int, list] = {}
_RUN_EVENTS_KEEP = 60


async def _run_events(props: typing.Any, begin: int, record: dict | None) -> list:
    """A finished run's raw event pages, cached. Caller holds STATE._lock."""
    hit = _RUN_EVENTS_CACHE.get(begin)
    if hit is not None:
        return hit
    events, _ = await _paged_events(
        props, "get_clean_detail", lambda c: {"begin": begin, "from": c, "id": 0, "type": 0})
    # Only a run that has ENDED, and only a log that actually came back: an empty
    # answer is as likely to be a dropped command as a run the robot kept no
    # detail for, and freezing that would make the sheet permanently say so.
    if events and record and int(record.get("end") or 0):
        _RUN_EVENTS_CACHE[begin] = events
        for stale in list(_RUN_EVENTS_CACHE)[:-_RUN_EVENTS_KEEP]:
            del _RUN_EVENTS_CACHE[stale]
        _hcache_touch()
    return events


def _zone_names_for(begin: float | None) -> list[str]:
    """What the zones of the run that began at `begin` were called.

    Empty when we did not launch it, or when it was not a zone run -- both of
    which are the honest answer rather than a guess.
    """
    logged = RUNS.for_run(STATE.selected_duid or "", float(begin or 0))
    if not logged or logged.get("kind") != "zone":
        return []
    # An unnamed zone still counts: dropping it would make a three-zone job read
    # as two. "Zone N" is what the panel calls an unnamed one anyway, and N is its
    # position in the job, which is what the panel numbers them by too.
    return [(z.get("name") or "").strip() or f"Zone {i + 1}"
            for i, z in enumerate(logged.get("requested_zones") or [])]


def _zone_seg_names(logged: dict, names: list[str], begin: float | None) -> dict[int, str]:
    """Pseudo segment -> zone name, from the ids the zones were SENT with.

    The robot drives the zones in `data` order and reports each as the id it was
    given, so this is a lookup rather than an inference. Path traces prove it (see
    ZONE_ORDER_REVERSE). Runs launched since the launch note recorded the id carry
    it on each zone; older ones are rebuilt from the order they went out in: listed
    order, except during _ZONE_REVERSED_SPAN when the list went out reversed.
    """
    zones = logged.get("requested_zones") or []
    out: dict[int, str] = {}
    if any(z.get("seg") is not None for z in zones):
        for i, z in enumerate(zones):
            seg = _int_or_none(z.get("seg"))
            if seg is not None and i < len(names):
                out[seg] = names[i]
        return out
    n = len(names)
    if n == 1:
        return {ZONE_PSEUDO_BASE: names[0]}
    lo, hi = _ZONE_REVERSED_SPAN
    if float(begin or 0) >= hi:
        return out          # launched since ids are recorded, so it fell back: unknown
    reversed_then = lo <= float(begin or 0) < hi
    for i, nm in enumerate(names):
        out[ZONE_PSEUDO_BASE + ((n - 1 - i) if reversed_then else i)] = nm
    return out


def _name_zone_rows(decoded: list[dict], begin: float | None) -> list[dict]:
    """Put our own zone names on the rows the robot could only call "Zone".

    Pseudo segment 41 is the first entry of the `data` we sent, 42 the second --
    the robot drives them in that order and reports the ids it was given. The
    earlier belief that it renumbers zones in its own driving order came from runs
    whose two zones overlap by half, judged by eye; the run map's path trace says
    otherwise (run 1789793563: sink strip, heating-plate strip, entrance, against
    segments 41, 42, 43 exactly as sent). So the id we sent names the row.

    Where the id is not known -- a multi-zone job that fell back to app_zoned_clean,
    which numbers the zones itself -- the reported settings are matched against the
    zones we launched instead, each match consuming its zone.
    """
    names = _zone_names_for(begin)
    if not names:
        return decoded
    logged = RUNS.for_run(STATE.selected_duid or "", float(begin or 0)) or {}
    rows = [e for e in decoded if e.get("zone") and e.get("segment") is not None]
    named_seg = _zone_seg_names(logged, names, begin)

    unnamed = sorted({int(e["segment"]) for e in rows} - set(named_seg))
    if unnamed:
        job = logged.get("requested") or logged.get("modes") or {}

        def want(z: dict) -> tuple:
            return tuple(_int_or_none(z.get(k) if z.get(k) is not None else job.get(k))
                         for k in ("fan", "water", "mop", "repeat"))

        def got(e: dict) -> tuple:
            st = e.get("settings") or {}
            return tuple(_int_or_none(st.get(k)) for k in ("fan", "water", "mop", "repeat"))

        taken = set(named_seg.values())
        pool = [(i, z) for i, z in enumerate(logged.get("requested_zones") or [])
                if i < len(names) and names[i] not in taken]
        for e in sorted(rows, key=lambda r: (r.get("t") or 0)):
            seg = int(e["segment"])
            if seg in named_seg or not e.get("settings"):
                continue
            fp = got(e)
            hits = [(i, z) for i, z in pool
                    if all(a is None or b is None or a == b for a, b in zip(want(z), fp))]
            if len(hits) == 1:
                pool.remove(hits[0])
                named_seg[seg] = names[hits[0][0]]

    for e in decoded:
        if not e.get("zone"):
            continue
        seg = e.get("segment")
        nm = named_seg.get(int(seg)) if seg is not None else None
        e["zone_names"] = [nm] if nm else names
    return decoded


def _note_requested(decoded: list[dict], begin: float | None,
                    rooms: dict[int, str]) -> list[dict]:
    """Put what the job was ASKED for on its job_start row.

    A per-target job sets the device's globals to the custom markers -- fan 106,
    route 302 -- because that is how the firmware is told "each target carries its
    own". The robot then reports exactly that at job_start, and the panel rendered
    it as the one word it means: "Personnalisé". True, and useless: the settings
    that were actually used are per target, and the only record of them is ours.

    So the row gets the launch note: every target, its name, and the suction, water,
    route and passes it was given. Absent for a job we did not launch, which is the
    honest answer -- there is nothing to report and the robot's own word stands.
    """
    logged = RUNS.for_run(STATE.selected_duid or "", float(begin or 0))
    if not logged:
        return decoded
    targets: list[dict] = []
    for i, z in enumerate(logged.get("requested_zones") or []):
        targets.append({
            "name": (z.get("name") or "").strip() or f"Zone {i + 1}",
            **{k: z.get(k) for k in ("fan", "water", "mop", "repeat")},
        })
    for r in (logged.get("requested_per_room") or []):
        seg = _int_or_none(r.get("id"))
        targets.append({
            "name": rooms.get(seg) or (f"pièce {seg}" if seg is not None else "?"),
            "segment": seg,
            **{k: r.get(k) for k in ("fan", "water", "mop", "repeat")},
        })
    # A room clean with no PER-ROOM settings keeps its rooms in `requested_rooms`
    # and one set of modes in `requested`, and that was the gap: neither of the two
    # lists above has anything in it, so the row fell back to the plain settings card
    # and never said WHICH rooms. Those rooms are just as much a target list; they
    # simply share the job's modes, which is what the card's own fallback is for.
    if not targets:
        for seg in (logged.get("requested_rooms") or []):
            seg = _int_or_none(seg)
            targets.append({
                "name": rooms.get(seg) or (f"pièce {seg}" if seg is not None else "?"),
                "segment": seg,
                **{k: None for k in ("fan", "water", "mop", "repeat")},
            })
    req = {k: v for k, v in (logged.get("requested") or {}).items() if v is not None}
    if not targets and not req:
        return decoded
    for e in decoded:
        if e.get("key") == "job_start" and e.get("visible"):
            e["requested"] = {"targets": targets, "global": req,
                              "kind": logged.get("kind")}
            break
    return decoded


async def _read_timeline(props: typing.Any) -> dict | None:
    """The running job's event list, for pushing while somebody is watching it.

    Same read the page used to make every 5 seconds on its own; made once here for
    however many tabs are open. Caller holds STATE._lock -- and only for this: the
    frame is SENT outside it, because a wedged browser socket must not be able to
    hold up the vacuum's command queue.
    """
    try:
        rooms = await _segment_names(props)
        events, begin = await _paged_events(
            props, "get_rt_clean_detail", lambda c: {"from": c, "type": 0})
        st = props.status
        decoded = _name_zone_rows(_decode_timeline(
            events, rooms, running=_is_running(_int_or_none(st.state), st.in_cleaning)), begin)
        _note_requested(decoded, begin, rooms)
    except Exception:  # noqa: BLE001 -- a dropped read retries on the next tick
        _LOGGER.debug("timeline read failed", exc_info=True)
        return None
    return {"begin": begin, "events": decoded, "available": bool(decoded), "note": ""}


async def handle_run_detail(request: web.Request) -> web.Response:
    """Full detail of one past run: its record plus its event timeline.

    `begin` is the record's own begin timestamp, which is also its id in
    get_clean_summary. get_clean_detail keeps this for runs that finished weeks
    ago -- it is what the phone's history detail screen renders.
    """
    try:
        begin = int(request.query["begin"])
    except (KeyError, ValueError):
        raise web.HTTPBadRequest(reason="begin=<record timestamp> required") from None

    # One short critical section each, rather than one long one: STATE._lock is the
    # single MQTT queue and the 1s status poll is waiting behind it. Both of these
    # take the lock only when the answer is not already cached.
    events = _RUN_EVENTS_CACHE.get(begin)
    async with STATE._lock:
        props = await STATE.props()
        rooms = await _segment_names(props)
        record = await _clean_record(props, begin)
        if events is None:
            events = await _run_events(props, begin, record)

    decoded = _name_zone_rows(
        _decode_timeline(events, rooms, interrupted=_run_interrupted(record)), begin)
    _note_requested(decoded, begin, rooms)
    entry = _history_entry(record) if record else None
    # What this run was ASKED for. Neither the record nor the event log can say:
    # the record has no mode fields, and the log only has settings for rooms the
    # robot actually entered. For the rooms an interrupted run never reached,
    # our own note at launch time is the only source there is.
    logged = RUNS.for_run(STATE.selected_duid or "", begin)
    requested = None
    if logged and (logged.get("requested") or logged.get("requested_per_room")):
        requested = {
            "kind": logged.get("kind"),
            "rooms": logged.get("requested_rooms") or [],
            "modes": logged.get("requested") or {},
            "per_room": logged.get("requested_per_room") or [],
            "observed": logged.get("modes") or {},
        }
    return web.json_response({
        "begin": begin,
        "run": entry,
        "requested": requested,
        "events": decoded,
        "available": bool(decoded),
        "rooms": [
            {"segment": e.get("segment"), "room": e.get("room"), "zone": bool(e.get("zone")),
             "t": e.get("t"), "duration_sec": e.get("duration_sec"),
             "unfinished": e.get("unfinished"), "settings": e.get("settings")}
            for e in decoded if e["key"] == "room_settings"
        ],
        "note": "" if decoded else "The robot kept no event detail for this run.",
    })


def _map_room_names(props: typing.Any) -> dict[int, str]:
    """{segment_id: name} from the active map's own room list.

    get_multi_maps_list carries an `iot_name` per segment that survives even
    when get_room_mapping's second field is corrupt.
    """
    out: dict[int, str] = {}
    try:
        for info in (props.maps.map_info or []):
            for room in (getattr(info, "rooms", None) or []):
                seg = _int_or_none(getattr(room, "id", None))
                name = str(getattr(room, "iot_name", "") or "").strip()
                if seg is not None and name and not _looks_like_junk(name):
                    out[seg] = name
            break  # active map only
    except Exception:  # noqa: BLE001
        _LOGGER.debug("map room names unavailable", exc_info=True)
    return out


# get_room_mapping's rows are [segment_id, iot_id, tag]. The third field is the
# room TYPE the vacuum stores -- 14 kitchen, 15 bathroom, 8 corridor and so on,
# the same numbering the app's roomtag_bubble_tag_<N> artwork is named for. The
# rooms trait drops it, so it is read from the raw command.
#
# Cached because it only changes when someone re-types a room in the official
# app, and /api/state is polled every 30s. The TTL is there so that a re-tag
# shows up eventually without needing a restart.
_ROOM_TAGS: dict[int, int] = {}
_ROOM_TAGS_AT = 0.0
_ROOM_TAGS_TTL = 600.0


async def _room_tags(props: typing.Any) -> dict[int, int]:
    """{segment_id: room type tag} from the vacuum."""
    global _ROOM_TAGS_AT
    if _ROOM_TAGS and (time.time() - _ROOM_TAGS_AT) < _ROOM_TAGS_TTL:
        return _ROOM_TAGS
    try:
        raw = await props.command.send(RoborockCommand.GET_ROOM_MAPPING)
    except Exception:  # noqa: BLE001 -- the panel falls back to a generic icon
        _LOGGER.debug("room tags unavailable", exc_info=True)
        return _ROOM_TAGS
    tags: dict[int, int] = {}
    for row in raw or []:
        if isinstance(row, (list, tuple)) and len(row) >= 3:
            seg, tag = _int_or_none(row[0]), _int_or_none(row[2])
            if seg is not None and tag is not None:
                tags[seg] = tag
    if tags:
        _ROOM_TAGS.clear()
        _ROOM_TAGS.update(tags)
        _ROOM_TAGS_AT = time.time()
    return _ROOM_TAGS


# The order rooms are listed in, everywhere: the quick-select strip, the
# per-room settings rows, and -- because app_segment_clean uses clean_order_mode
# =1 -- the order a multi-room job actually cleans in. Segment ids are what the
# vacuum sorts by otherwise, and that is the order the rooms happened to be
# carved out in, which is nobody's route through their own home.
#
# Accepts segment ids or room names, comma separated. Names are matched
# accent- and case-insensitively; ids are steadier, since a rename cannot break
# them. Anything not listed keeps its place at the end, in segment order, so a
# newly split room appears rather than vanishing.
ROOM_ORDER = os.environ.get("RR_ROOM_ORDER", "")


def _fold(text: typing.Any) -> str:
    """Lowercase, accent-stripped, for matching a name the user typed."""
    return "".join(c for c in unicodedata.normalize("NFD", str(text).strip().lower())
                   if unicodedata.category(c) != "Mn")


def _order_rooms(rooms: list[dict]) -> list[dict]:
    """`rooms` in the order RR_ROOM_ORDER asks for."""
    rank = {_fold(item): i for i, item in enumerate(
        x.strip() for x in ROOM_ORDER.split(",") if x.strip())}
    if not rank:
        return rooms

    def key(r: dict) -> tuple:
        pos = rank.get(_fold(r["segment_id"]))
        if pos is None:
            pos = rank.get(_fold(r["name"]))
        return (0, pos) if pos is not None else (1, int(r["segment_id"]))

    return sorted(rooms, key=key)


def _room_name(rm: typing.Any, seg: int) -> str:
    """Human name for a segment, tolerating both room-mapping dialects.

    get_room_mapping returns [segment_id, <second>, ...]. python-roborock always
    reads <second> as an iot id and resolves the name through the account's room
    list. This stack puts the NAME there instead, so that lookup misses and
    raw_name comes back None -- which previously rendered every room as
    "Room N" and looked like the names had been wiped. They had not.

    So: prefer the resolved name, else use the raw second field when it is not
    an id-shaped value, else fall back to the segment number.
    """
    name = getattr(rm, "raw_name", None)
    if name:
        return str(name)
    iot = getattr(rm, "iot_id", None)
    if iot is not None:
        text = str(iot).strip()
        # Account iot ids are all-numeric; anything else is already the name --
        # unless the vacuum stored junk there. Segment 4 on this robot holds a
        # truncated JSON fragment ('{"created":true,"homeId'), so anything that
        # looks like structured data is rejected rather than shown as a name.
        if text and not text.isdigit() and not _looks_like_junk(text):
            return text
    return f"Pièce {seg}"


def _looks_like_junk(text: str) -> bool:
    """True when a room 'name' is corrupt data or a bare placeholder.

    Segment 4 on this robot stored a truncated JSON fragment in the room
    mapping, and its map entry was the unnamed-room placeholder "Room" -- so
    neither was a name a person chose.

    "NaN" is the same thing from a different direction: renaming a room in the
    official app writes the new room's id into the mapping, and for segments 3
    and 7 it wrote the JavaScript string "NaN" instead. The name is then simply
    gone from the device -- the id points at nothing in the account's room list,
    so there is nothing left to look it up by. Rendering it verbatim put "NaN"
    on the map and in every room list; falling back to "Pièce <n>" at least says
    what is true. Renaming from the Map tab writes a usable value again.
    """
    stripped = text.strip()
    if stripped.startswith(("{", "[")) or '"' in stripped or len(stripped) > 40:
        return True
    return stripped.lower() in {"room", "piece", "pièce", "unnamed", "nan", "null", "none", "undefined"}


async def handle_runs(_request: web.Request) -> web.Response:
    """The run log: what each recent run was set to, and how it ended."""
    duid = STATE.selected_duid or ""
    return web.json_response({"runs": RUNS.recent(duid) if duid else []})


async def handle_rooms_raw(_request: web.Request) -> web.Response:
    """Diagnostic: the raw segment->room mapping plus the account's room list.

    Room NAMES do not live on the vacuum. The robot only stores segment_id ->
    iot_id; the human name for an iot_id comes from the account/home data. This
    endpoint shows both halves so a broken rename can be told apart from a
    broken name lookup.
    """
    async with STATE._lock:
        props = await STATE.props()
        raw = await props.command.send(RoborockCommand.GET_ROOM_MAPPING)
        await props.rooms.refresh()
        trait = [
            {"segment_id": r.segment_id, "iot_id": r.iot_id, "raw_name": r.raw_name}
            for r in (props.rooms.rooms or [])
        ]
    home_rooms = []
    try:
        hd = STATE.manager._home_data if STATE.manager is not None else None  # noqa: SLF001
        home_rooms = [{"id": r.id, "name": r.name} for r in (getattr(hd, "rooms", None) or [])]
    except Exception as exc:  # noqa: BLE001
        home_rooms = [{"error": str(exc)}]
    # The active map's own room list. It carries a `tag` of its own, which is
    # not always the same one get_room_mapping reports -- worth seeing side by
    # side when a re-type in the app does not appear to have landed.
    map_rooms = []
    try:
        async with STATE._lock:
            props = await STATE.props()
            await props.maps.refresh()
            for mi in (props.maps.map_info or []):
                for r in (getattr(mi, "rooms", None) or []):
                    map_rooms.append({"id": getattr(r, "id", None), "tag": getattr(r, "tag", None),
                                      "iot_name": getattr(r, "iot_name", None),
                                      "iot_name_id": getattr(r, "iot_name_id", None)})
                break  # active map only
    except Exception as exc:  # noqa: BLE001
        map_rooms = [{"error": str(exc)}]
    return web.json_response({"get_room_mapping": raw, "trait": trait,
                              "map_info_rooms": map_rooms, "home_data_rooms": home_rooms})


async def handle_routines(_request: web.Request) -> web.Response:
    async with STATE._lock:
        props = await STATE.props()
        try:
            scenes = await props.routines.get_routines()
            out = [{"id": s.id, "name": s.name} for s in scenes]
        except Exception as exc:  # noqa: BLE001
            return web.json_response({"routines": [], "error": str(exc)})
    return web.json_response({"routines": out})


# --------------------------------------------------------------------------- #
# Action endpoints
# --------------------------------------------------------------------------- #
async def handle_select(request: web.Request) -> web.Response:
    _patrol_available()
    body = await request.json()
    duid = str(body.get("duid", "")).strip()
    if not any(d.duid == duid for d in STATE.devices):
        raise web.HTTPBadRequest(reason="unknown duid")
    STATE.selected_duid = duid
    return web.json_response({"ok": True, "selected": duid})


def _current_modes(st: typing.Any) -> dict:
    return {"fan": _int_or_none(st.fan_power), "water": _int_or_none(st.water_box_mode),
            "mop": _int_or_none(st.mop_mode), "repeat": _int_or_none(st.repeat) or 1}


async def handle_command(request: web.Request) -> web.Response:
    body = await request.json()
    action = str(body.get("action", "")).strip()
    command = SIMPLE_ACTIONS.get(action)
    if PATROL.active:
        if action in ("stop", "pause", "dock"):
            await PATROL.stop()
        else:
            _patrol_available()
    # `resume` is not one command: which one depends on what was paused, and only
    # the robot knows that. in_cleaning says WHICH kind of job is outstanding (2 for
    # a zone run, 1 for rooms), so the pairing comes from the device rather than from
    # whatever the browser last saw -- which is what makes a resume from a second
    # phone work as well as from the one that pressed pause.
    if action != "resume" and command is None:
        raise web.HTTPBadRequest(reason=f"unknown action {action!r}")
    async with STATE._lock:
        props = await STATE.props()
        if action == "resume":
            kind = _int_or_none(getattr(props.status, "in_cleaning", None)) or 0
            ladder = ([RoborockCommand.RESUME_ZONED_CLEAN] if kind == 2
                      else [RoborockCommand.RESUME_SEGMENT_CLEAN] if kind == 1
                      else [])
            # APP_START last: it resumes a paused job on this firmware and is the
            # only thing left for a paused run that reports no kind at all.
            ladder.append(RoborockCommand.APP_START)
            result, err = None, None
            for cmd in ladder:
                try:
                    result = await props.command.send(cmd)
                    _LOGGER.info("resume via %s", cmd)
                    break
                except Exception as exc:  # noqa: BLE001 -- try the next rung
                    _LOGGER.info("resume: %s refused (%s)", cmd, exc)
                    err = exc
            else:
                raise web.HTTPBadGateway(reason=f"resume refused: {err}")
        else:
            result = await props.command.send(command)
        duid = STATE.selected_duid or ""
        modes = _current_modes(props.status) if action == "start" else {}
    # Keep the job ledger honest: a bare "start" is a whole-house run we did not
    # compose, and stop/dock end whatever was running.
    if duid:
        if action == "start":
            TRACKER.note_launch(duid, "full", params=modes)
            # The settings it goes out with are whatever the device holds right
            # now, and those drift the moment anything else touches them. Write
            # them down while they are still what this run is using.
            RUNS.note_request(duid, "full", (), modes)
        elif action in ("stop", "dock"):
            TRACKER.clear_launch(duid)
    _journal("command_action", {"action": action, "result": result})
    _LOGGER.info("command %s -> %s", action, result)
    return web.json_response({"ok": True, "result": result})


async def handle_errors(_request: web.Request) -> web.Response:
    """Error history for the selected device, most recent first."""
    duid = STATE.selected_duid or ""
    return web.json_response({"errors": TRACKER.recent_errors(duid) if duid else []})


# Washing a set of pads takes a couple of minutes; cap the wait well above that
# so a dock that never reports "done" cannot leave the resume hanging forever.
WASH_TIMEOUT = 420.0
WASH_POLL = 5.0
# The legacy state table's wash codes, used as a hint only -- like every other
# code in that enum they are unverified for this model, so the wait below keys
# off "left the dock state and came back" rather than trusting these.
WASH_STATES = {
    RoborockStateCode.washing_the_mop.value,
    RoborockStateCode.washing_the_mop_2.value,
    RoborockStateCode.going_to_wash_the_mop.value,
}
DOCKED_STATES = {RoborockStateCode.idle.value, RoborockStateCode.charging.value}


async def _wash_and_wait(duid: str) -> bool:
    """Wash the mop pads at the dock, waiting for the cycle to finish.

    Returns whether a wash actually ran. Resuming the moment the wash is issued
    would cancel it -- the robot cannot drive off and wash at the same time --
    so this has to wait. It takes STATE._lock only for the individual commands,
    never across the sleeps, so status polling keeps working throughout.
    """
    async with STATE._lock:
        props = await STATE.props()
        try:
            await props.command.send(RoborockCommand.APP_START_WASH)
        except RoborockException as exc:
            _LOGGER.warning("mop wash refused, resuming without it: %s", exc)
            return False

    deadline = time.time() + WASH_TIMEOUT
    started = False
    while time.time() < deadline:
        await asyncio.sleep(WASH_POLL)
        try:
            async with STATE._lock:
                props = await STATE.props()
                await props.status.refresh()
                state = _int_or_none(props.status.state)
        except RoborockException:
            continue
        if state in WASH_STATES or state not in DOCKED_STATES:
            started = True
            TRACKER.set_pending(duid, "washing", "washing the mop pads")
        elif started:
            _LOGGER.info("mop wash finished")
            return True
    _LOGGER.warning("mop wash did not report completion within %.0fs; continuing", WASH_TIMEOUT)
    return started


async def _apply_modes(cmd: typing.Any, values: dict) -> None:
    """Push fan / water / mop-route so a resumed run cleans like the original."""
    for key, command, field in (
        ("fan", RoborockCommand.SET_CUSTOM_MODE, "fan_power"),
        ("water", RoborockCommand.SET_WATER_BOX_CUSTOM_MODE, "water_box_mode"),
        ("mop", RoborockCommand.SET_MOP_MODE, "mop_mode"),
    ):
        if values.get(key) is not None:
            await _apply_setting(cmd, command, field, int(values[key]))


def _subdivision_data(per_room: typing.Sequence[dict], segments: typing.Sequence[int],
                      fallback: dict) -> list[dict]:
    """The `data` block of app_segment_clean_subdivision, one entry per room.

    Schema captured off the wire (RR_MQTT_TRACE=1) from a real configured room
    clean; see NOTES.md. Rooms not in `segments` are dropped, so a resume can
    hand this the settings of the whole original job and get back only the part
    it still has to do.
    """
    want = {int(s) for s in segments}
    data = []
    for item in per_room:
        seg = _int_or_none(item.get("id"))
        if seg is None or seg not in want:
            continue
        data.append({
            "id": seg,
            "fan_power": int(item.get("fan") if item.get("fan") is not None else (fallback.get("fan") or 102)),
            "water_box_mode": int(item.get("water") if item.get("water") is not None else (fallback.get("water") or 200)),
            "mop_mode": int(item.get("mop") if item.get("mop") is not None else (fallback.get("mop") or 300)),
            "repeat": max(1, min(3, int(item.get("repeat") or 1))),
            # Fixed fields observed in every captured job.
            "mop_power": 0, "mop_type": -1,
            "need_wash": 1, "wash_mode": 2, "seq_type": 0,
        })
    return data


# Off: zones go out in the order they are listed, and the robot drives them in
# that order. The 2026-09-14 conclusion that it takes the END of `data` first was
# drawn from watching two zones that overlap by half, and it was wrong: the run
# maps' own path traces settle it. Run 1789793563 sent [Zone A 41, Zone B 42,
# Entrée étendu 43] -- the user's [Entrée, Zone B, Zone A] reversed -- and the
# trace goes kitchen-sink strip, then the heating-plate strip, then the entrance,
# 246 s / 178 s / 781 s against segments 41 / 42 / 43. Run 1791651577 likewise did
# id 41 (Zone A) first. So the reversal cleaned every multi-zone job backwards from
# 2026-09-14 14:45 UTC until it was turned off here.
ZONE_ORDER_REVERSE = os.environ.get("RR_ZONE_ORDER_REVERSE", "0").strip() == "1"
# The window in which zone jobs went out reversed, for naming the rows of runs
# launched then that did not record their ids (see _zone_seg_names). Its end is
# also when launches started recording them.
_ZONE_REVERSED_SPAN = (1789396900, 1792877000)
ZONE_PSEUDO_BASE = 41
"""First pseudo segment id of a zone job. The official app numbers its zones from
41 upwards -- captured off the wire, and the same ids the clean record's event log
reports back, which is how _name_zone_rows pairs a row with a zone."""


def _zone_subdivision_data(zones: list[dict], fallback: dict) -> list[dict]:
    """The `data` block of app_zoned_clean_subdivision, one entry per zone.

    Schema captured off the wire (RR_MQTT_TRACE=1, tools/find_command.py) from a
    real two-zone job started in the official app with DIFFERENT suction and passes
    per zone -- fan_power 101/repeat 1 and fan_power 102/repeat 2. That capture is
    what disproved this panel's earlier claim that zones could not carry their own
    modes; the claim came from app_zoned_clean's five-element form having nowhere to
    put them, which said nothing about the richer command existing.

    `zones` is 8 numbers, the rectangle's four corners in perimeter order starting
    at (x0, ymax): x0,y1, x1,y1, x1,y0, x0,y0. Each entry takes {rect(vacuum mm as
    x0,y0,x1,y1), fan, water, mop, repeat}.
    """
    # In listed order: the robot cleans data[0] first and reports it as segment 41
    # (see ZONE_ORDER_REVERSE for the path traces that show it). The switch stays
    # so the order can be flipped without a code change if a firmware disagrees.
    if ZONE_ORDER_REVERSE:
        zones = list(reversed(zones))
    data = []
    for i, z in enumerate(zones):
        x0, y0, x1, y1 = z["rect"]
        x0, x1 = min(x0, x1), max(x0, x1)
        y0, y1 = min(y0, y1), max(y0, y1)
        data.append({
            "id": ZONE_PSEUDO_BASE + i,
            "zones": [x0, y1, x1, y1, x1, y0, x0, y0],
            "fan_power": int(z.get("fan") if z.get("fan") is not None else (fallback.get("fan") or 102)),
            "water_box_mode": int(z.get("water") if z.get("water") is not None else (fallback.get("water") or 200)),
            "mop_mode": int(z.get("mop") if z.get("mop") is not None else (fallback.get("mop") or 300)),
            "repeat": max(1, min(3, int(z.get("repeat") or 1))),
            # Fixed fields, exactly as captured. mop_type is 3 here where the room
            # command sends -1; carried across rather than harmonised, because the
            # capture is the only thing that knows.
            "mop_power": 0, "mop_type": 3,
            "need_wash": 1, "wash_mode": 2, "seq_type": 0,
        })
    return data


async def _zone_subdivision_clean(cmd: typing.Any, data: list[dict], repeat: int) -> typing.Any:
    """Start a zone clean whose zones carry their own settings.

    The payload is logged in full before it goes: a rejection comes back as bare
    `{'code': -10007, 'message': 'invalid params'}` with no hint which field the
    robot objected to, so the only way to work backwards is to have the exact thing
    that was sent.
    """
    payload = {
        "auto_dry": 1, "auto_dustCollection": 1, "clean_order_mode": 1,
        "is_new_clean": 1, "repeat": repeat, "data": data,
    }
    _LOGGER.info("app_zoned_clean_subdivision <- %s",
                 json.dumps(payload, separators=(",", ":")))
    _journal("zone_payload", payload)
    # A BARE DICT, not [dict]. The two subdivision commands disagree about this and
    # the difference is the whole bug: app_segment_clean_subdivision (rooms) takes a
    # one-element list, this one takes the object itself, and wrapping it comes back
    # as `{'code': -10007, 'message': 'invalid params'}` with nothing to say which
    # field was wrong. Proved by replaying the captured call both ways against the
    # robot: dict accepted, [dict] rejected. The capture had it as a dict all along
    # -- tools/find_command.py printed it unwrapped and this code copied the rooms
    # command's shape instead of the trace's.
    return await cmd.send("app_zoned_clean_subdivision", payload)


async def _subdivision_clean(cmd: typing.Any, data: list[dict], repeat: int) -> typing.Any:
    """Start a room clean whose rooms carry their own settings.

    Order follows the order the rooms are listed, as clean_order_mode=1 asks.
    """
    payload = {
        "auto_dry": 1, "auto_dustCollection": 1, "clean_order_mode": 1,
        "is_new_clean": 1, "repeat": repeat, "data": data,
    }
    _LOGGER.info("app_segment_clean_subdivision <- %s",
                 json.dumps(payload, separators=(",", ":")))
    _journal("clean_payload", payload)
    # A BARE DICT, like its zone twin -- see _zone_subdivision_clean. This was
    # wrapped in a list and had therefore never once worked: the run log shows zero
    # runs ever launched with per-room settings, so nothing exercised it until the
    # preset merge started producing per_room bodies. Both shapes were replayed
    # against the robot: [dict] rejected with invalid params, dict accepted.
    return await cmd.send("app_segment_clean_subdivision", payload)


async def _segment_clean(duid: str, rooms: list[int], repeat: int, modes: dict) -> typing.Any:
    async with STATE._lock:
        props = await STATE.props()
        await _apply_modes(props.command, modes)
        result = await props.command.send(
            RoborockCommand.APP_SEGMENT_CLEAN,
            [{"segments": rooms, "repeat": repeat, "clean_order_mode": 1}],
        )
    params = {"repeat": repeat, "fan": modes.get("fan"),
              "water": modes.get("water"), "mop": modes.get("mop")}
    TRACKER.note_launch(duid, "segment", segments=rooms, params=params)
    RUNS.note_request(duid, "segment", rooms, params)
    return result


async def _resume_clean(duid: str, rooms: list[int], repeat: int, modes: dict,
                        subdivision: list[dict] | None = None) -> typing.Any:
    """Re-issue the outstanding rooms, per-room settings included where there are any."""
    if not subdivision:
        return await _segment_clean(duid, rooms, repeat, modes)
    async with STATE._lock:
        props = await STATE.props()
        result = await _subdivision_clean(props.command, subdivision, repeat)
    per_room = [{"id": d["id"], "fan": d["fan_power"], "water": d["water_box_mode"],
                 "mop": d["mop_mode"], "repeat": d["repeat"]} for d in subdivision]
    params = {"repeat": repeat, **{k: modes.get(k) for k in ("fan", "water", "mop")}}
    TRACKER.note_launch(duid, "segment", segments=rooms, params=params)
    RUNS.note_request(duid, "segment", rooms, params, per_room=per_room)
    return result


async def _wash_then_resume(duid: str, rooms: list[int], repeat: int, modes: dict,
                            subdivision: list[dict] | None = None) -> None:
    """Background half of a wash-then-resume, so the request can return at once."""
    try:
        TRACKER.set_pending(duid, "washing", "washing the mop pads before resuming")
        await _wash_and_wait(duid)
        TRACKER.set_pending(duid, "starting", "starting the resumed clean")
        await _resume_clean(duid, rooms, repeat, modes, subdivision)
    except Exception as exc:  # noqa: BLE001 -- background task, report and stop
        _LOGGER.error("wash-then-resume failed: %s", exc)
        TRACKER.set_pending(duid, "failed", str(exc))
        await asyncio.sleep(20)
    finally:
        if (TRACKER.pending(duid) or {}).get("phase") != "failed":
            TRACKER.clear_pending(duid)


# --- looking after the mop once the run is over ---------------------------
#
# The dock does not always wash and dry after a mopping job. Whether it decides to
# is not something this panel can read -- the wash and the dry both happen after
# job_end, outside the run's own event log, and `dry_status` only says whether one
# is happening NOW. So instead of predicting it, this watches: once the robot has
# been parked long enough to be sure it is finished, it looks at whether a wash and
# then a dry actually happened since the run ended, and asks for whichever is
# missing. Wash first, then dry, because drying a dirty pad is pointless.
#
# Deliberately conservative. It acts once per run and never while anything is
# moving, and one press of Arrêter in the Entretien sheet stops the current cycle
# and takes that run off the list for good.
MOP_CARE = os.environ.get("RR_MOP_CARE", "1").strip().lower() in ("1", "true", "yes")
MOP_CARE_PATH = os.environ.get("RR_MOP_CARE_DB", str(HERE / "mop_care.json"))
MOP_CARE_SETTLE = float(os.environ.get("RR_MOP_CARE_SETTLE", "240"))
MOP_CARE_TICK = 30.0
MOP_CARE_MAX_AGE = float(os.environ.get("RR_MOP_CARE_MAX_AGE", "10800"))
# How many times to ask for the same cycle before giving up. Without this, a dock
# that silently refuses the command (no pad fitted, tank empty) would be asked
# again every tick, for ever -- the start/stop loop, in slow motion.
MOP_CARE_TRIES = 2
# Named apart from _DOCKED_STATES, which is a set of state LABELS -- reusing that
# name here shadowed it and quietly made at_dock always false.
_PARKED_STATES = {
    RoborockStateCode.charging.value,
    RoborockStateCode.charging_complete.value,
    RoborockStateCode.idle.value,
}

_mop_care: dict = {
    "run": None,        # the run key we are looking after
    "phase": "idle",    # idle | wash | dry | done | off
    "washed": False,
    "dried": False,
    "since": 0.0,       # when the robot became parked
    "tries": {},        # cycle -> how many times we have asked for it
    "note": "",
}


# On/off is a preference, not a session detail: switched off from the phone it has
# to stay off across a restart, or the next deploy quietly turns it back on.
#
# `armed_at` is what makes "on" mean the NEXT run rather than the one just
# stopped. Switching on stamps the clock, and the watcher ignores any run that
# had already ended by then -- so turning it off mid-cycle and back on later
# cannot re-trigger the wash that was interrupted, without needing to remember
# which run that was.
_care_cfg: dict = {"enabled": True, "armed_at": 0.0}


def _care_cfg_load() -> None:
    try:
        with open(MOP_CARE_PATH, encoding="utf-8") as fh:
            saved = json.load(fh)
        if isinstance(saved, dict):
            _care_cfg["enabled"] = bool(saved.get("enabled", True))
            _care_cfg["armed_at"] = float(saved.get("armed_at") or 0)
    except (OSError, ValueError):
        pass


def _care_cfg_save() -> None:
    try:
        with open(MOP_CARE_PATH, "w", encoding="utf-8") as fh:
            json.dump(_care_cfg, fh)
    except OSError as exc:
        _LOGGER.warning("cannot save mop care setting: %s", exc)


def _mop_care_state() -> dict:
    """What the watcher is doing, for the Entretien sheet."""
    return {
        "enabled": MOP_CARE and _care_cfg["enabled"],
        "available": MOP_CARE,
        "phase": _mop_care["phase"],
        "run": _mop_care["run"],
        "washed": _mop_care["washed"],
        "dried": _mop_care["dried"],
        "note": _mop_care["note"],
        "settle": MOP_CARE_SETTLE,
    }


def _run_used_mop(run: dict | None) -> bool:
    """Did the job we launched ASK to put water down anywhere?"""
    if not run:
        return False
    wet = []
    # What we ASKED for, only. The device-observed `modes` used to count too, for
    # runs started elsewhere -- but since jobs park the device on the custom
    # markers, what it reports afterwards is a leftover: run 1791827550 was sent
    # with water 200 and its observed modes said 235. A run we have no request for
    # is therefore "unknown", which reads as no mop: nothing is done to a pad on
    # a guess.
    wet.append((run.get("requested") or {}).get("water"))
    for r in (run.get("requested_per_room") or []):
        wet.append(r.get("water"))
    for z in (run.get("requested_zones") or []):
        wet.append(z.get("water"))
    for code in wet:
        if code is None:
            continue
        if (_water_level(_int_or_none(code)) or 0) > 0:
            return True
    return False


def _care_evidence(since: float) -> tuple[bool, bool]:
    """Has the pad been washed, and dried since that wash, in this run's window?

    Read out of the journal rather than kept in memory, because the panel gets
    restarted and the dock's own cycle can easily fall in the gap -- and a watcher
    that forgets an already-done wash re-runs it. `dock_busy` rows are written on
    every transition, which is exactly the trail needed.

    `since` is the run's START, not its end, and the test is "was there a dry after
    the LAST wash". Both details are the fix for a redundant wash the watcher asked
    for on 2026-09-09: the dock washed the pads at 17:28 while the robot was still
    docked mid-run and dried them at 17:33 as the run closed, so a window that
    began at the run's end saw a dry with no wash before it, decided the pad had
    never been washed, and started a second wash three seconds after the user had
    stopped the first one. Whether the dock's wash falls just inside or just
    outside the end of the run is not a distinction worth acting on.

    The one case this reads generously: a dock that washes on the way OUT and never
    again is taken as having washed. Better than washing a clean pad twice.
    """
    last_wash = 0.0
    dried_at = 0.0
    try:
        with open(LAUNCH_JOURNAL, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if '"dock_busy"' not in line:
                    continue
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                when = float(row.get("t") or 0)
                if when < since:
                    continue
                to = (row.get("payload") or {}).get("to") or []
                if "washing" in to:
                    last_wash = when
                if "drying" in to:
                    dried_at = when
    except OSError:
        pass
    return bool(last_wash), bool(last_wash and dried_at >= last_wash)


async def _mop_care_watch(app: web.Application) -> None:
    while True:
        await asyncio.sleep(MOP_CARE_TICK)
        if not MOP_CARE or not _care_cfg["enabled"]:
            continue
        try:
            async with STATE._lock:
                if not STATE.devices:
                    continue
                props = await STATE.props()
                st = props.status
                state = _int_or_none(getattr(st, "state", None))
                busy = _dock_activity(st)
                parked = (state in _PARKED_STATES
                          and not _is_running(state, getattr(st, "in_cleaning", None))
                          and not getattr(st, "in_cleaning", None))
                if not parked:
                    # Anything moving resets the clock: "docked for a while" has to
                    # mean uninterrupted, or a dock excursion mid-run would count.
                    _mop_care["since"] = 0.0
                    if _mop_care["phase"] in ("wash", "dry"):
                        _mop_care["note"] = "en cours"
                    continue
                now = time.time()
                if not _mop_care["since"]:
                    _mop_care["since"] = now
                # A cycle we asked for counts as observed the moment we see it.
                if "washing" in busy:
                    _mop_care["washed"] = True
                if "drying" in busy:
                    _mop_care["dried"] = True
                if now - _mop_care["since"] < MOP_CARE_SETTLE:
                    continue

                # The ROBOT's record of its last run, not our run log. Two wrong
                # cycles in October came from trusting the log: on 10-12 last_run()
                # returned a synthetic "unknown" duplicate the sampler had written a
                # few seconds after our launch, whose device-observed modes claimed
                # water 235 for a job sent dry; on 10-06 it returned a 10-01 run
                # whose ended_at the sampler had kept fresh for five days, so the
                # age guard never tripped. The clean record has neither problem:
                # one entry per run, a real end time, and its own verdict.
                rec = _last_run_cached()
                if not rec:
                    continue
                key = _int_or_none(rec.get("begin"))
                if key != _mop_care["run"]:
                    _mop_care.update(run=key, phase="idle", washed=False,
                                     dried=False, tries={}, note="")
                if _mop_care["phase"] == "done":
                    continue
                # A run that went to the end is the dock's to look after. It washes
                # and dries on its own after a completed mopping job, and stepping
                # in there is exactly the unwanted wash-then-dry. Only a run that
                # failed or was cut short can have missed its care.
                if rec.get("complete") and not rec.get("error"):
                    _mop_care.update(phase="done", note="terminé normalement")
                    continue
                run = RUNS.for_run(STATE.selected_duid or "", float(key or 0))
                if not _run_used_mop(run):
                    _mop_care.update(phase="done", note="pas de serpillière")
                    continue
                ended = float(rec.get("end") or 0)
                if ended and ended < _care_cfg["armed_at"]:
                    # It was switched off while this run was waiting to be looked
                    # after. Switching back on is for the next one.
                    _mop_care.update(phase="done", note="hors service")
                    continue
                if not ended or now - ended > MOP_CARE_MAX_AGE:
                    # Old news. Washing a pad hours after the fact is not looking
                    # after it, and on a fresh start the last run in the log can
                    # be days old -- that must not fire a cycle.
                    _mop_care.update(phase="done", note="trop ancien")
                    continue
                seen_wash, seen_dry = _care_evidence(float(key or ended))
                _mop_care["washed"] = _mop_care["washed"] or seen_wash
                _mop_care["dried"] = _mop_care["dried"] or seen_dry

                tries = _mop_care["tries"]
                for cycle, seen, phase, note in (
                        ("wash", _mop_care["washed"], "wash", "lavage"),
                        ("dry", _mop_care["dried"], "dry", "séchage")):
                    if seen:
                        continue
                    if tries.get(cycle, 0) >= MOP_CARE_TRIES:
                        # Asked for and never observed. Stop rather than keep at it,
                        # and say so: something at the dock is refusing, and this
                        # row is the only place that would ever tell you.
                        _mop_care.update(phase="done", note=note + " sans effet")
                        _journal("mop_care", {"action": "gave_up", "cycle": cycle,
                                              "run": key})
                        _LOGGER.info("mop care: %s never happened, giving up "
                                     "(run %s)", cycle, key)
                        break
                    tries[cycle] = tries.get(cycle, 0) + 1
                    _mop_care.update(phase=phase, note=note + " demandé")
                    try:
                        if cycle == "wash":
                            await _raw(props.command, RoborockCommand.APP_START_WASH)
                        else:
                            await _set_dryer(props.command, 1)
                    except Exception as exc:  # noqa: BLE001
                        _LOGGER.info("mop care: %s refused (%s)", cycle, exc)
                    _journal("mop_care", {"action": cycle, "run": key,
                                          "try": tries[cycle]})
                    _LOGGER.info("mop care: asked the dock to %s (run %s, try %d)",
                                 cycle, key, tries[cycle])
                    break
                else:
                    _mop_care.update(phase="done", note="lavage et séchage faits")
                    _journal("mop_care", {"action": "done", "run": key})
                continue
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 -- a watcher must not die
            _LOGGER.debug("mop care tick failed: %s", exc)


# --- queued jobs ----------------------------------------------------------
#
# A job queued while another is running starts the moment the first one heads for
# the dock, so the robot never makes the trip. That is the whole point: the dock
# visit between two jobs costs several minutes and leaves the pads to be washed
# and dried in between.
#
# An entry is the REQUEST the panel would have posted to launch it by hand -- the
# endpoint and the body, verbatim. So a queued job is byte-identical to a launched
# one, and presets, fine-tuning, merges and per-room settings all work with no
# machinery of their own. It is replayed by posting it back to this very server,
# which keeps one launch path rather than two.
QUEUE_PATH = os.environ.get("RR_QUEUE_DB", str(HERE / "queue.json"))
QUEUE_TICK = 5.0
_QUEUE_ENDPOINTS = {"/api/clean", "/api/zone"}
_queue: list[dict] = []
_queue_fired: float = 0.0     # the run we already cut in on, so we do it once


def _queue_load() -> None:
    try:
        with open(QUEUE_PATH, encoding="utf-8") as fh:
            saved = json.load(fh)
        if isinstance(saved, list):
            _queue[:] = [e for e in saved if isinstance(e, dict)]
    except (OSError, ValueError):
        pass


def _queue_save() -> None:
    try:
        with open(QUEUE_PATH, "w", encoding="utf-8") as fh:
            json.dump(_queue, fh)
    except OSError as exc:
        _LOGGER.warning("cannot save the job queue: %s", exc)


async def _job_targets_done(props: typing.Any) -> bool:
    """Has the running job been everywhere it was asked to go?

    The trigger is the robot heading home with nothing left on its list, not
    merely heading home: it also goes back mid-job to empty the bin, wash the
    pads or charge, and cutting in there would kill a run that meant to come
    back out. The live event log is the only thing that knows the difference --
    a room it is still in reports `unfinished`.

    Unknowable (no rows yet, a job we did not launch) counts as NOT done, so an
    unreadable run is left alone rather than cut short.
    """
    frame = await _read_timeline(props)
    rows = [e for e in ((frame or {}).get("events") or [])
            if e.get("key") == "room_settings"]
    if not rows:
        return False
    return not any(e.get("unfinished") for e in rows)


async def _queue_launch(entry: dict) -> bool:
    """Replay one entry through the panel's own HTTP API."""
    import aiohttp   # only this path needs the client half of the library

    url = "http://127.0.0.1:8099" + entry["endpoint"]
    try:
        async with aiohttp.ClientSession() as sess:
            async with sess.post(url, json=entry["body"],
                                 timeout=aiohttp.ClientTimeout(total=120)) as resp:
                body = await resp.text()
                ok = resp.status == 200
    except Exception as exc:  # noqa: BLE001
        _LOGGER.warning("queued job could not be launched: %s", exc)
        _journal("queue", {"action": "failed", "entry": entry, "error": str(exc)})
        return False
    _journal("queue", {"action": "launched" if ok else "refused",
                       "entry": entry, "status": resp.status, "body": body[:300]})
    _LOGGER.info("queued job %s -> %s", entry.get("label"), resp.status)
    return ok


async def _queue_watch(app: web.Application) -> None:
    global _queue_fired
    while True:
        await asyncio.sleep(QUEUE_TICK)
        if not _queue:
            continue
        try:
            async with STATE._lock:
                if not STATE.devices:
                    continue
                props = await STATE.props()
                st = props.status
                state = _int_or_none(getattr(st, "state", None))
                if state != RoborockStateCode.returning_home.value:
                    continue
                begin = float((_last_run_cached() or {}).get("begin") or 0)
                if begin and begin == _queue_fired:
                    continue
                if not await _job_targets_done(props):
                    continue
                # Stop where it stands rather than letting it reach the dock.
                await _raw(props.command, RoborockCommand.APP_STOP)
                _queue_fired = begin
            entry = _queue.pop(0)
            _queue_save()
            _journal("queue", {"action": "cut_in", "entry": entry, "run": begin})
            _LOGGER.info("queue: cutting in on the dock trip for %s",
                         entry.get("label"))
            # Outside the lock: the launch takes it itself, and the robot needs a
            # moment to actually come to rest before it will accept a new job.
            await asyncio.sleep(3)
            if not await _queue_launch(entry):
                _queue.insert(0, entry)
                _queue_save()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 -- a watcher must not die
            _LOGGER.debug("queue tick failed: %s", exc)


def _queue_state() -> dict:
    """The queue, with enough of each job to draw it like any other job row.

    The settings come out of the stored request rather than being kept twice: for
    a uniform job they are the job-wide values, and for a per-target one there is
    no single answer, so `mixed` says so instead of picking one target's.
    """
    out = []
    for e in _queue:
        body = e.get("body") or {}
        targets = body.get("segments") or body.get("zones") or []
        per = body.get("per_room") or [z for z in (body.get("zones") or [])
                                       if isinstance(z, dict) and z.get("fan") is not None]
        modes = {k: body.get(k) for k in ("fan", "water", "mop")}
        modes["repeat"] = body.get("repeat")
        out.append({
            "id": e.get("id"), "label": e.get("label"), "at": e.get("at"),
            "endpoint": e.get("endpoint"),
            "kind": "zone" if e.get("endpoint") == "/api/zone" else "segment",
            "targets": len(targets),
            "modes": None if any(v is None for v in
                                 (modes["fan"], modes["mop"])) else modes,
            "mixed": bool(per) and len(per) > 1,
        })
    return {"jobs": out, "count": len(_queue)}


async def handle_queue(request: web.Request) -> web.Response:
    """Read the queue, add to it, or take something out of it.

    POST {endpoint, body, label}  -> append
    POST {remove: id}             -> drop that one
    POST {clear: true}            -> drop all
    """
    if request.method == "GET":
        return web.json_response(_queue_state())
    body = await request.json() if request.can_read_body else {}
    if body.get("clear"):
        _queue.clear()
    elif body.get("remove"):
        _queue[:] = [e for e in _queue if e.get("id") != body["remove"]]
    else:
        endpoint = str(body.get("endpoint") or "")
        if endpoint not in _QUEUE_ENDPOINTS:
            raise web.HTTPBadRequest(reason="endpoint must be /api/clean or /api/zone")
        if not isinstance(body.get("body"), dict):
            raise web.HTTPBadRequest(reason="body must be the launch request object")
        _queue.append({
            "id": f"q{int(time.time() * 1000)}",
            "at": time.time(),
            "endpoint": endpoint,
            "label": str(body.get("label") or "")[:120],
            "body": body["body"],
        })
    _queue_save()
    _journal("queue", {"action": "edit", "count": len(_queue)})
    return web.json_response(_queue_state())


async def handle_mop_care(request: web.Request) -> web.Response:
    """Read the watcher, or switch it on and off.

    POST {"enable": false} switches it off AND stops whatever it has running --
    off has to mean the dock goes quiet now, not merely that it will not be asked
    again. Only the cycle actually in progress is stopped: sending stop-wash to an
    idle dock is harmless, but clearing dry_status when nothing is drying would
    disable the next dry rather than this one.

    POST {"enable": true} arms it for the next run only. It does not pick the
    interrupted one back up -- see `armed_at`.
    """
    if request.method == "GET":
        return web.json_response(_mop_care_state())
    body = await request.json() if request.can_read_body else {}
    want = body.get("enable")
    if want is None:
        return web.json_response({"error": "enable required"}, status=400)
    want = bool(want)
    if not want:
        async with STATE._lock:
            props = await STATE.props()
            busy = _dock_activity(props.status)
            if "washing" in busy:
                await _raw(props.command, RoborockCommand.APP_STOP_WASH)
            if "drying" in busy:
                try:
                    await _set_dryer(props.command, 0)
                except Exception as exc:  # noqa: BLE001
                    _LOGGER.info("mop care off: dryer refused (%s)", exc)
        _mop_care.update(phase="done", washed=True, dried=True, tries={},
                         note="désactivé")
    else:
        _mop_care.update(phase="idle", washed=False, dried=False, tries={},
                         note="")
    _care_cfg.update(enabled=want, armed_at=time.time())
    _care_cfg_save()
    _journal("mop_care", {"action": "on" if want else "off"})
    _LOGGER.info("mop care: switched %s", "on" if want else "off")
    return web.json_response(_mop_care_state())


async def handle_resume_dismiss(request: web.Request) -> web.Response:
    """Stop offering to finish one run, for good and for everybody.

    Kept in the run log rather than in the browser: the offer is advertised with a
    dot on the history button and on the run's own row, and a local flag would leave
    the second phone still nagging about a run the first one had already dealt with.

    Body: {"begin": <record timestamp>}
    """
    body = await request.json()
    begin = _int_or_none(body.get("begin"))
    if begin is None:
        raise web.HTTPBadRequest(reason="begin required")
    ok = RUNS.dismiss_resume(STATE.selected_duid or "", float(begin))
    _RUN_TODO_CACHE.clear()      # the offer is computed from this
    _journal("resume_dismissed", {"begin": begin, "matched": ok})
    _LOGGER.info("resume offer dismissed for %s (matched=%s)", begin, ok)
    await push_status()
    return web.json_response({"ok": True, "matched": ok})


async def handle_resume(request: web.Request) -> web.Response:
    """Resume an interrupted run -- see _resume_plan for the two meanings.

    Refuses while a blocking fault is still reported, because re-issuing the
    job just drives the robot back into the same fault (and, for a jam, spins
    the motor against the obstruction). Pass force:true to override.

    Body: {force, wash_first}. wash_first washes the mop pads at the dock first
    and only sets off once that finishes -- pads that sat on a stalled robot are
    dry, and a dry pad pushes dirt around instead of lifting it. That path runs
    in the background and reports progress through status.resume.pending.
    """
    body = await request.json() if request.can_read_body else {}
    force = bool(body.get("force"))
    wash_first = bool(body.get("wash_first"))
    async with STATE._lock:
        props = await STATE.props()
        await props.status.refresh()
        # Coverage is read off the path trace, so the map must be current or we
        # would re-clean rooms that have been finished since the last refresh.
        try:
            await props.map_content.refresh()
        except Exception:  # noqa: BLE001 -- fall back to the cached map
            _LOGGER.debug("map refresh before resume failed", exc_info=True)
        st = props.status
        duid = STATE.selected_duid or ""
        dev = STATE.device()
        plan = _resume_plan(duid, st, props, _status_class(dev) if dev else None)
        err = _int_or_none(st.error_code) or 0
        if err and plan["blocked_by_error"] and not force:
            info = jobs_mod.describe_error(err)
            raise web.HTTPConflict(reason=f"{info['title']} — clear it first, or resume with force")
        if not plan["available"]:
            raise web.HTTPBadRequest(reason="nothing to resume")
        if TRACKER.pending(duid):
            raise web.HTTPConflict(reason="a resume is already in progress")

        modes = plan["modes"]
        launch = TRACKER.last_launch(duid)
        params = (launch or {}).get("params", {})
        # Precedence: what the caller explicitly asked for (the Clean tab's
        # selectors) > what we recorded when we launched the run > what the
        # device reports now. The device cannot tell us what an older run used,
        # so the user's explicit choice has to be able to win.
        values = {}
        for k in ("fan", "water", "mop"):
            v = body.get(k)
            if v is None:
                v = params.get(k)
            if v is None:
                v = modes.get(k)
            values[k] = v
        chosen_water = _int_or_none(values.get("water"))
        mopping = chosen_water is not None and chosen_water != WATER_OFF
        repeat = max(1, min(3, int(body.get("repeat") or params.get("repeat") or 1)))

        if plan["kind"] == "vacuum":
            # The robot still holds the job, so its own modes are already right.
            result = await props.command.send(RoborockCommand.APP_START)
            _LOGGER.info("resume: vacuum resume (app_start) forced=%s", force)
            return web.json_response({"ok": True, "kind": "vacuum", "rooms": [], "modes": modes,
                                      "detail": "vacuum resume", "result": result})

        rooms = plan["rooms"]
        if not rooms:
            raise web.HTTPBadRequest(reason="no rooms left to clean")

        # A job whose rooms each had their own settings has to be resumed the
        # same way, or the rooms it never reached get whatever single set of
        # values happened to be lying around. The caller sends back the original
        # per-room block; _subdivision_data keeps only the rooms still owed.
        per_room = body.get("per_room") or []
        subdivision = _subdivision_data(per_room, rooms, values) if per_room else []
        if per_room and not subdivision:
            raise web.HTTPBadRequest(reason="per_room covered none of the rooms left to clean")
        if subdivision:
            mopping = any(d["water_box_mode"] != WATER_OFF for d in subdivision)

    if wash_first and mopping:
        asyncio.create_task(_wash_then_resume(duid, rooms, repeat, values, subdivision))
        return web.json_response({"ok": True, "kind": "program", "rooms": rooms, "modes": modes,
                                  "per_room": bool(subdivision),
                                  "pending": "washing", "detail": "washing pads, then resuming"})

    result = await _resume_clean(duid, rooms, repeat, values, subdivision)
    _LOGGER.info("resume: rooms %s modes %s per_room=%s forced=%s",
                 rooms, values, len(subdivision), force)
    return web.json_response({"ok": True, "kind": "program", "rooms": rooms, "modes": modes,
                              "per_room": bool(subdivision),
                              "detail": f"resuming {len(rooms)} room(s)", "result": result})


async def handle_clean(request: web.Request) -> web.Response:
    _patrol_available()
    body = await request.json()
    _LOGGER.info("/api/clean <- %s", json.dumps(body, separators=(",", ":"), default=str))
    _journal("clean_request", body)
    segments = [int(s) for s in body.get("segments", [])]
    if not segments:
        raise web.HTTPBadRequest(reason="no rooms selected")
    repeat = max(1, min(3, int(body.get("repeat", 1))))
    fan, water, mop = body.get("fan"), body.get("water"), body.get("mop")
    applied: list[str] = []
    skipped: list[str] = []
    async with STATE._lock:
        props = await STATE.props()
        cmd = props.command
        # The global setters are gone from this path. They were how a room job's
        # suction, water and route were applied, and they are refused outright once
        # the device sits on the custom markers -- so they were the reason a job
        # could be launched with settings that had silently not taken. The values go
        # in the job itself below instead.
        #
        # Per-room settings are not stored on the vacuum (get_customize_clean_mode
        # returns [] even after a configured run) -- they travel WITH the job, one
        # object per room. Schema captured off the wire (RR_MQTT_TRACE=1) from a
        # real configured room clean; see NOTES.md.
        per_room = body.get("per_room") or []
        try:
            # Every room job carries its settings per room now, configured or not.
            #
            # The plain form cannot: app_segment_clean takes ids and a repeat, so
            # suction, water and route have to be set with the global setters first
            # -- and those get REFUSED (-10005 'param error!') once the device has
            # been parked on the custom markers, which every zone job does to it and
            # leaves it that way. On 2026-09-09 17:13 a three-room job asked for
            # water 200 with the setters refused, ran with the 235 the device still
            # held, and the dock then washed and dried pads the job was never
            # supposed to wet. The job was configured dry and went out wet, and
            # nothing in the panel said so.
            #
            # So the settings travel WITH the job, where they cannot be refused, and
            # a uniform job just gets the same values on every room.
            if not per_room:
                per_room = [{"id": s, "fan": fan, "water": water, "mop": mop,
                             "repeat": repeat} for s in segments]
                uniform = True
            else:
                uniform = False
            data = _subdivision_data(per_room, segments, {"fan": fan, "water": water, "mop": mop})
            if not data:
                raise web.HTTPBadRequest(reason="per_room had no rooms matching segments")
            # The markers, exactly as the app sends them before a subdivision call.
            await _raw(cmd, "set_clean_motor_mode",
                       [{"fan_power": 106, "mop_mode": 302, "water_box_mode": 204}])
            result = await _subdivision_clean(cmd, data, repeat)
            applied.append(f"rooms×{len(data)}" if uniform else f"per_room×{len(data)}")
        except web.HTTPException:
            raise
        except Exception as exc:  # noqa: BLE001
            _LOGGER.warning("room clean refused by the robot: %s", exc)
            raise web.HTTPBadRequest(reason=f"l'aspirateur a refusé : {exc}") from exc
        duid = STATE.selected_duid or ""
    if duid:
        params = {"repeat": repeat, "fan": fan, "water": water, "mop": mop}
        TRACKER.note_launch(duid, "segment", segments=segments, params=params)
        RUNS.note_request(duid, "segment", segments, params, per_room=body.get("per_room") or ())
    _LOGGER.info("segment clean segments=%s repeat=%s applied=%s skipped=%s", segments, repeat, applied, skipped)
    return web.json_response({"ok": True, "result": result, "applied": applied, "skipped": skipped})


LAUNCH_JOURNAL = os.environ.get("RR_LAUNCH_JOURNAL", "/data/launches.jsonl")
LAUNCH_JOURNAL_MAX = 4 * 1024 * 1024
"""Every launch, start to finish, on disk.

`docker logs` was where this lived and it is the wrong place: it dies with the
container, and three separate diagnoses in a row lost their evidence to a
`compose up -d`. A job launch is a rare event and the whole chain is small, so it
goes to /data where it survives everything: what the browser posted, what was
decided from it, the exact command payload the robot was sent, and the note kept
for the timeline. That is the full path from a tap to the wire, decodable after
the fact without asking anyone to reproduce anything.

One generation of rotation at 4MB, the same rule the jsonl traces use.
"""


def _journal(kind: str, payload: typing.Any) -> None:
    try:
        now = time.time()
        line = json.dumps({"t": now, "kind": kind,
                           "at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
                           "payload": payload},
                          separators=(",", ":"), ensure_ascii=False, default=str)
        path = Path(LAUNCH_JOURNAL)
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            if path.stat().st_size + len(line) + 1 > LAUNCH_JOURNAL_MAX:
                path.replace(path.with_name(path.name + ".1"))
        except FileNotFoundError:
            pass
        with path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except Exception as exc:  # noqa: BLE001 -- a journal must never break a launch
        _LOGGER.debug("journal write failed: %s", exc)


JOURNAL_PARAM_MAX = 4000


def _journal_commands(props: typing.Any) -> None:
    """Keep a durable record of every command that CHANGES something.

    Reads are left out on purpose: get_status alone runs every second or so, and a
    journal that drowns in polling is one nobody can read. What is left is the few
    dozen commands a day that start, stop, configure or save something -- which is
    exactly the set worth having when a job misbehaves, with the params that were
    sent and what came back or what it was refused with.

    Wrapped once per device, on the same pass that subscribes to pushed updates.
    Failing to wrap is not worth failing a request over: the panel works without
    the journal, it just becomes harder to debug.
    """
    cmd = getattr(props, "command", None)
    if cmd is None or getattr(cmd, "_rr_journalled", False):
        return
    inner = cmd.send

    def _clip(value: typing.Any) -> typing.Any:
        try:
            text = json.dumps(value, ensure_ascii=False, default=str)
        except Exception:  # noqa: BLE001
            return repr(value)[:JOURNAL_PARAM_MAX]
        return value if len(text) <= JOURNAL_PARAM_MAX else text[:JOURNAL_PARAM_MAX] + "…"

    async def send(command: typing.Any, params: typing.Any = None) -> typing.Any:
        name = str(getattr(command, "value", None) or command)
        # `get_` covers the reads; app_get_* are reads too despite the prefix.
        watch = not (name.startswith("get_") or name.startswith("app_get"))
        try:
            result = await (inner(command, params) if params is not None else inner(command))
        except Exception as exc:
            if watch:
                _journal("command", {"method": name, "params": _clip(params),
                                     "error": str(exc)})
            raise
        if watch:
            _journal("command", {"method": name, "params": _clip(params),
                                 "result": _clip(result)})
        return result

    try:
        cmd.send = send            # type: ignore[method-assign]
        cmd._rr_journalled = True  # type: ignore[attr-defined]
        _LOGGER.info("journalling write commands to %s", LAUNCH_JOURNAL)
    except Exception as exc:  # noqa: BLE001
        _LOGGER.warning("could not journal commands: %s", exc)


async def handle_launch_journal(request: web.Request) -> web.Response:
    """Read the launch journal back, newest last. ?limit=N, ?kind=zone_payload."""
    limit = max(1, min(500, int(request.query.get("limit", "40"))))
    want = (request.query.get("kind") or "").strip()
    rows: list[dict] = []
    for path in (Path(LAUNCH_JOURNAL + ".1"), Path(LAUNCH_JOURNAL)):
        if not path.exists():
            continue
        for line in path.read_text(errors="replace").splitlines():
            try:
                row = json.loads(line)
            except Exception:  # noqa: BLE001
                continue
            if want and row.get("kind") != want:
                continue
            rows.append(row)
    return web.json_response({"entries": rows[-limit:], "path": LAUNCH_JOURNAL},
                             dumps=lambda o: json.dumps(o, ensure_ascii=False, default=str))


async def handle_zone(request: web.Request) -> web.Response:
    """Clean one or more zones.

    app_zoned_clean is sent here as [[x1, y1, x2, y2, repeat], ...] in vacuum mm,
    with a per-zone repeat. In THIS form suction/water/route have nowhere to go, so
    a zone's fan/water/mop are accepted and applied globally (last zone wins).

    That is a limit of this payload, not of the device: the official app does set
    modes per zone, so a richer form exists and we have not captured it yet. Do not
    read the code below as evidence that it cannot be done -- see
    tools/find_command.py for how to catch the real one off the wire.

    Evidence that this matches the official app rather than merely standing in
    for it: a zone clean started from the phone (record 1786619169, 2026-08-13)
    reports exactly one `i=42` settings event in get_clean_detail, for the
    pseudo segment 41 with scope 3, carrying a single fan/water/route/passes set
    for the whole job -- see NOTES.md. That was a one-zone job, and a two-zone one
    since (record 1787563753) reports TWO settings events, pseudo segments 41 and
    42, one per zone with its own duration -- so zones are reported individually
    after all. Both carried the same modes, which is still consistent with the
    modes being job-wide; the per-zone values collected below stay plumbed through
    for the capture that shows otherwise.

    Body: {zones:[{rect:[x0,y0,x1,y1] img px, repeat, fan?, water?, mop?}],
           fan, water, mop}
          (also accepts the legacy {rect, repeat} single-zone form).
    """
    _patrol_available()
    body = await request.json()
    _LOGGER.info("/api/zone <- %s", json.dumps(body, separators=(",", ":"), default=str))
    _journal("zone_request", body)
    zones_in = body.get("zones")
    if not zones_in:  # legacy single-zone
        rect = body.get("rect")
        if not rect or len(rect) != 4:
            raise web.HTTPBadRequest(reason="provide zones:[{rect,repeat}]")
        zones_in = [{"rect": rect, "repeat": body.get("repeat", 1)}]
    fan, water, mop = body.get("fan"), body.get("water"), body.get("mop")
    # Placeholder: per-zone modes collapse to the last zone that specifies one.
    # Logged so it is obvious in the trace that this is not truly per-zone.
    for z in zones_in:
        for key, val in (("fan", z.get("fan")), ("water", z.get("water")), ("mop", z.get("mop"))):
            if val is not None:
                if key == "fan":
                    fan = val
                elif key == "water":
                    water = val
                else:
                    mop = val
    # Do the zones actually differ? A job where every zone wants the same thing has
    # nothing to gain from the richer command, and the plain form is the one with
    # years of use behind it, so it keeps that path.
    def _modes_of(z: dict) -> tuple:
        return (z.get("fan"), z.get("water"), z.get("mop"), z.get("repeat", 1))
    per_zone = len(zones_in) > 1 and len({_modes_of(z) for z in zones_in}) > 1 \
        and any(z.get(k) is not None for z in zones_in for k in ("fan", "water", "mop"))
    # Any job with more than one zone goes through the subdivision command, even when
    # every zone carries the same settings and the simple form would do.
    #
    # The reason is ordering. app_zoned_clean takes a bare array of
    # [x0,y0,x1,y1,repeat] rectangles and nothing else -- there is no envelope, so
    # there is nowhere to say "do them in this order", and the firmware routes as it
    # pleases. The subdivision call is the one with the envelope, and the official
    # app's own capture puts clean_order_mode:1 in it. Rooms have always had this
    # (app_segment_clean takes an object and already gets clean_order_mode), so
    # zones were the only targets whose order the panel could not honour.
    ordered = len(zones_in) > 1
    applied: list[str] = []
    skipped: list[str] = []
    async with STATE._lock:
        props = await STATE.props()
        await props.map_content.refresh()
        to_vac = _image_to_vacuum(props.map_content.map_data)
        if to_vac is None:
            raise web.HTTPBadRequest(reason="no map calibration; cannot map coordinates")
        cmd = props.command
        if per_zone or ordered:
            # What the app does immediately before EVERY subdivision call: park the
            # globals on the "custom" markers, which is how the run then reports
            # itself (fan 106 / mop 302) and how _todo_from_timeline already knows
            # to judge no room against a single job-wide intent.
            #
            # Sent for a uniform multi-zone job too, not just a per-zone one. That
            # is the difference between the capture of the app's own two-zone job
            # (2026-09-07 10:38 UTC) and what this panel was sending: identical
            # command, identical clean_order_mode, identical fixed fields -- but the
            # app sets the markers first and we only did when the zones disagreed.
            # Its job then drove the zones in the order of the array (quiet kitchen
            # first, turbo entrance second, watched and confirmed) where ours
            # reordered them every time. The read: without the markers the firmware
            # takes the call as an ordinary zone job, optimises the route, and
            # ignores the per-entry settings -- which is also how a job configured
            # dry went out with the water on, and why the global setters it fell
            # back to were refused with -10005.
            await _raw(cmd, "set_clean_motor_mode",
                       [{"fan_power": 106, "mop_mode": 302, "water_box_mode": 204}])
            applied.append("per_zone_modes" if per_zone else "custom_modes")
        else:
            if fan is not None:
                (applied if await _apply_setting(cmd, RoborockCommand.SET_CUSTOM_MODE, "fan_power", int(fan)) else skipped).append("fan")
            if water is not None:
                (applied if await _apply_setting(cmd, RoborockCommand.SET_WATER_BOX_CUSTOM_MODE, "water_box_mode", int(water)) else skipped).append("water")
            if mop is not None:
                (applied if await _apply_setting(cmd, RoborockCommand.SET_MOP_MODE, "mop_mode", int(mop)) else skipped).append("mop")

        zones: list[list[int]] = []
        vac: list[dict] = []
        for z in zones_in:
            r = z["rect"]
            repeat = max(1, min(3, int(z.get("repeat", 1))))
            (ax, ay), (bx, by) = to_vac(r[0], r[1]), to_vac(r[2], r[3])
            box = [min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)]
            zones.append(box + [repeat])
            vac.append({"rect": box, "repeat": repeat,
                        "fan": z.get("fan"), "water": z.get("water"), "mop": z.get("mop")})
        try:
            if per_zone or ordered:
                # With uniform settings the fallback fills every entry with the same
                # job-wide values, which is what the globals set above already say --
                # so the two agree and the run reports itself the ordinary way.
                data = _zone_subdivision_data(vac, {"fan": fan, "water": water, "mop": mop})
                try:
                    result = await _zone_subdivision_clean(
                        cmd, data, max(1, min(3, int(body.get("repeat") or 1))))
                    if not per_zone:
                        applied.append("ordered")
                except Exception as exc:  # noqa: BLE001
                    # Ordering is an improvement, not a requirement, so it must not
                    # be able to cost you the job. app_zoned_clean refuses more than
                    # five rectangles and this command's own ceiling is unknown, so
                    # a job that used to start has to still start -- unordered, and
                    # saying so -- rather than fail with the robot's complaint.
                    if per_zone:
                        raise
                    _LOGGER.warning("ordered zone clean refused (%s); falling back "
                                    "to app_zoned_clean", exc)
                    _journal("zone_order_fallback", {"error": str(exc),
                                                     "zones": len(zones)})
                    result = await cmd.send(RoborockCommand.APP_ZONED_CLEAN, zones)
                    skipped.append("ordered")
            else:
                result = await cmd.send(RoborockCommand.APP_ZONED_CLEAN, zones)
        except Exception as exc:  # noqa: BLE001
            # The robot's own words rather than a 500: it says useful things here --
            # "too many region params" for more than five zones -- and a bare
            # Internal Server Error in a toast tells nobody anything.
            _LOGGER.warning("zone clean refused by the robot: %s", exc)
            raise web.HTTPBadRequest(reason=f"l'aspirateur a refusé : {exc}") from exc
    # A zone run was never logged, which is why its history could only ever say
    # "Zone": the robot reports every zone of the job as the single pseudo segment
    # 41, so unless we write down what we sent, the names are gone. Named zones
    # only -- an unnamed one has nothing to infer.
    duid = STATE.selected_duid or ""
    # The EFFECTIVE settings, not just the explicit ones: a zone that carries none
    # of its own is cleaned with the job-wide values, and those are what the device
    # was told. Recording only the explicit ones made the launch note on the
    # timeline disagree with what actually happened -- it had to guess at the
    # fallback, and a note that guesses is worse than no note.
    def _eff(z: dict) -> dict:
        return {
            "fan": z.get("fan") if z.get("fan") is not None else fan,
            "water": z.get("water") if z.get("water") is not None else water,
            "mop": z.get("mop") if z.get("mop") is not None else mop,
            "repeat": max(1, min(3, int(z.get("repeat") or 1))),
        }
    # The pseudo segment each zone went out as, which is what the event log will
    # report it as. Known only when we chose the ids: the subdivision command, or a
    # lone zone (always 41). A multi-zone app_zoned_clean fallback numbers them
    # itself, so those rows are left to the settings fingerprint.
    sent_ids = (per_zone or ordered) and "ordered" not in skipped
    n_in = len(zones_in)

    def _seg(i: int) -> dict:
        if n_in == 1:
            return {"seg": ZONE_PSEUDO_BASE}
        if not sent_ids:
            return {}
        return {"seg": ZONE_PSEUDO_BASE + ((n_in - 1 - i) if ZONE_ORDER_REVERSE else i)}

    named = [{"name": (z.get("name") or "").strip(), "rect": list(z.get("rect") or []),
              **_eff(z), **_seg(i)}
             for i, z in enumerate(zones_in)]
    _LOGGER.info("zone launch note: per_zone=%s zones=%s", per_zone,
                 json.dumps([{k: v for k, v in n.items() if k != "rect"} for n in named],
                            ensure_ascii=False))
    _journal("zone_decision", {
        "per_zone": per_zone,
        "ordered": ordered,
        "job_wide": {"fan": fan, "water": water, "mop": mop, "repeat": body.get("repeat")},
        # What arrived per zone, before any fallback: this is the field that has been
        # disagreeing with the robot's own report and the one to read first.
        "zones_in": [{k: z.get(k) for k in ("name", "fan", "water", "mop", "repeat")}
                     for z in zones_in],
        "note_written": [{k: v for k, v in n.items() if k != "rect"} for n in named],
        "applied": applied, "skipped": skipped,
    })
    TRACKER.note_launch(duid, "zone", params={"fan": fan, "water": water, "mop": mop})
    RUNS.note_request(duid, "zone", (), {"fan": fan, "water": water, "mop": mop,
                                         "repeat": body.get("repeat")}, zones=named)
    _LOGGER.info("zone clean %d zone(s)%s=%s applied=%s skipped=%s",
                 len(zones), " per-zone" if per_zone else "", zones, applied, skipped)
    return web.json_response({"ok": True, "result": result, "zones": zones, "applied": applied, "skipped": skipped})


async def handle_goto(request: web.Request) -> web.Response:
    _patrol_available()
    body = await request.json()
    async with STATE._lock:
        props = await STATE.props()
        await props.map_content.refresh()
        to_vac = _image_to_vacuum(props.map_content.map_data)
        if to_vac is None:
            raise web.HTTPBadRequest(reason="no map calibration; cannot map coordinates")
        (x, y) = to_vac(float(body["x"]), float(body["y"]))
        result = await props.command.send(RoborockCommand.APP_GOTO_TARGET, [x, y])
    _LOGGER.info("goto %s,%s", x, y)
    return web.json_response({"ok": True, "result": result, "target": [x, y]})


async def handle_barriers_save(request: web.Request) -> web.Response:
    """Replace ALL map barriers via one save_map call (Valetudo-confirmed format).

    Body carries the COMPLETE desired set in IMAGE PIXELS (so nothing pre-existing
    is lost — the frontend seeds from what /api/state reported):
      { walls:  [[x0,y0,x1,y1], ...],                 # line endpoints
        no_go:  [[x0,y0,x1,y1,x2,y2,x3,y3], ...],     # 4 corners, perimeter order
        no_mop: [[x0,y0,x1,y1,x2,y2,x3,y3], ...] }
    Each point is converted image-px -> vacuum-mm via the map calibration. Records:
      wall  -> [1, mx0,my0, mx1,my1]
      no_go -> [0, 8 mm coords]
      no_mop-> [2, 8 mm coords]
    save_map is a full REPLACE; an empty set clears all barriers. Vertex budget 68.
    """
    body = await request.json()
    walls = body.get("walls", []) or []
    no_go = body.get("no_go", []) or []
    no_mop = body.get("no_mop", []) or []
    async with STATE._lock:
        props = await STATE.props()
        await props.map_content.refresh()
        to_vac = _image_to_vacuum(props.map_content.map_data)
        if to_vac is None:
            raise web.HTTPBadRequest(reason="no map calibration; cannot map coordinates")

        vertices = len(walls) * 2 + (len(no_go) + len(no_mop)) * 4
        if vertices > 68:
            raise web.HTTPBadRequest(reason=f"too many barrier vertices ({vertices} > 68)")

        records: list[list[int]] = []
        for w in walls:
            (a0, a1), (b0, b1) = to_vac(w[0], w[1]), to_vac(w[2], w[3])
            records.append([1, a0, a1, b0, b1])
        for kind, zones in ((0, no_go), (2, no_mop)):
            for z in zones:
                rec = [kind]
                for i in range(0, 8, 2):
                    mx, my = to_vac(z[i], z[i + 1])
                    rec += [mx, my]
                records.append(rec)
        result = await props.command.send(RoborockCommand.SAVE_MAP, records)
    _LOGGER.info("save_map barriers: %d walls, %d no-go, %d no-mop (%d vertices)",
                 len(walls), len(no_go), len(no_mop), vertices)
    _drop_map_ttl()          # walls and no-go zones live in the map we just rewrote
    return web.json_response({"ok": True, "result": result, "records": records})


async def handle_thresholds_save(request: web.Request) -> web.Response:
    """Replace threshold / door-sill areas (map block 28) via app_set_door_sill_blocks.
    EXPERIMENTAL: the write payload is inferred from the block format (a list of
    quads, 4 corners in vacuum mm). Body: {quads:[[x0,y0,...x3,y3] image px, ...]}."""
    body = await request.json()
    quads_px = body.get("quads", []) or []
    async with STATE._lock:
        props = await STATE.props()
        await props.map_content.refresh()
        to_vac = _image_to_vacuum(props.map_content.map_data)
        if to_vac is None:
            raise web.HTTPBadRequest(reason="no map calibration")
        records: list[list[int]] = []
        for q in quads_px:
            rec: list[int] = []
            for i in range(0, 8, 2):
                mx, my = to_vac(q[i], q[i + 1])
                rec += [mx, my]
            records.append(rec)
        cmd = props.command
        for params in (records, {"data": records}):  # try bare list, then {"data":...}
            try:
                result = await cmd.send(RoborockCommand.APP_SET_DOOR_SILL_BLOCKS, params)
                _LOGGER.info("door_sill_blocks set: %d quad(s) %r", len(records), params)
                return web.json_response({"ok": True, "result": result, "records": records})
            except RoborockException as exc:
                _LOGGER.info("door_sill_blocks %r rejected: %s", params, exc)
    raise web.HTTPBadRequest(reason="app_set_door_sill_blocks rejected (payload guess wrong)")


async def handle_modes(request: web.Request) -> web.Response:
    body = await request.json()
    applied, skipped = [], []
    async with STATE._lock:
        props = await STATE.props()
        cmd = props.command
        if body.get("fan") is not None:
            (applied if await _apply_setting(cmd, RoborockCommand.SET_CUSTOM_MODE, "fan_power", int(body["fan"])) else skipped).append("fan")
        if body.get("water") is not None:
            (applied if await _apply_setting(cmd, RoborockCommand.SET_WATER_BOX_CUSTOM_MODE, "water_box_mode", int(body["water"])) else skipped).append("water")
        if body.get("mop") is not None:
            (applied if await _apply_setting(cmd, RoborockCommand.SET_MOP_MODE, "mop_mode", int(body["mop"])) else skipped).append("mop")
        if body.get("repeat") is not None:
            try:
                await cmd.send(RoborockCommand.SET_CLEAN_REPEAT_TIMES, [max(1, min(3, int(body["repeat"])))])
                applied.append("repeat")
            except RoborockException as exc:
                _LOGGER.info("repeat rejected: %s", exc)
                skipped.append("repeat")
    return web.json_response({"ok": True, "applied": applied, "skipped": skipped})


async def handle_dock_set(request: web.Request) -> web.Response:
    body = await request.json()
    applied, skipped = [], []
    async with STATE._lock:
        props = await STATE.props()
        cmd = props.command
        if body.get("dust_collection") is not None:
            (applied if await _apply_setting(cmd, RoborockCommand.SET_DUST_COLLECTION_MODE, "mode", int(body["dust_collection"])) else skipped).append("dust_collection")
        if body.get("wash_towel") is not None:
            (applied if await _apply_setting(cmd, RoborockCommand.SET_WASH_TOWEL_MODE, "wash_mode", int(body["wash_towel"])) else skipped).append("wash_towel")
        if body.get("smart_wash") is not None or body.get("wash_interval") is not None:
            params = {}
            if body.get("smart_wash") is not None:
                params["smart_wash"] = int(body["smart_wash"])
            if body.get("wash_interval") is not None:
                params["wash_interval"] = int(body["wash_interval"])
            try:
                await cmd.send(RoborockCommand.SET_SMART_WASH_PARAMS, [params])
                applied.append("smart_wash")
            except RoborockException as exc:
                _LOGGER.info("smart_wash rejected: %s", exc)
                skipped.append("smart_wash")
    return web.json_response({"ok": True, "applied": applied, "skipped": skipped})


# Dock tasks are actions, not settings: start/stop the dock doing a job now.
# Each runs at whatever mode the dock is configured for, which /api/dock reports.
_DOCK_ACTIONS = {
    "dust":  (RoborockCommand.APP_START_COLLECT_DUST, RoborockCommand.APP_STOP_COLLECT_DUST),
    "wash":  (RoborockCommand.APP_START_WASH,         RoborockCommand.APP_STOP_WASH),
}


# app_set_dryer_status is the one dock command with no capture behind it, and
# [{"status": n}] -- the shape every other dock command uses -- comes back
# -10007 invalid params on this dock. So the plausible shapes are tried in turn
# and the one that works is remembered for the rest of the process; -10007 means
# "wrong shape", so it is the only error worth moving on from. Anything else
# (busy, no dock, offline) is a real answer and stops the ladder.
_DRYER_PARAM_SHAPES = (
    ("bare-dict", lambda on: {"status": on}),
    ("bare-int", lambda on: [on]),
    ("dict-in-list", lambda on: [{"status": on}]),
    ("on-off-dict", lambda on: {"status": on, "dry_time": 0}),
)
_INVALID_PARAMS = "-10007"
_dryer_shape: str | None = None


async def _set_dryer(cmd: typing.Any, on: int) -> typing.Any:
    shapes = [s for s in _DRYER_PARAM_SHAPES if _dryer_shape in (None, s[0])]
    last: Exception | None = None
    for name, build in shapes:
        try:
            result = await cmd.send(RoborockCommand.APP_SET_DRYER_STATUS, build(on))
        except RoborockException as exc:
            if _INVALID_PARAMS not in str(exc):
                raise
            _LOGGER.info("dryer: %s params rejected as invalid, trying the next shape", name)
            last = exc
            continue
        globals()["_dryer_shape"] = name
        _LOGGER.info("dryer: %s params accepted", name)
        return result
    raise last if last else RoborockException("no dryer parameter shape accepted")


# --------------------------------------------------------------------------- #
# Camera (live view) and remote control.
#
# The live view is WebRTC between the BROWSER and the robot; this panel only
# carries the signalling, which is ordinary robot RPCs over the MQTT link we
# already hold. Same sequence as go2rtc's roborock source (pkg/roborock/
# client.go), which is the reference implementation: clear any stale viewer,
# start_camera_preview, get_turn_server for the ICE server, send the offer as
# base64 JSON, poll get_device_sdp until it stops saying "retry", then trade
# candidates both ways. Media goes through the TURN relay the robot was handed
# by /fwapi/createca -- the local server's coturn -- so neither end needs a
# route to the other.
#
# The remote-view PIN is the one set in the Roborock app (a drawn pattern reads
# as digits on a 1-9 keypad, row by row). It travels to the robot md5-hashed,
# never in clear, and is only kept in the environment (RR_CAMERA_PIN), so it is
# never in a URL or a log line.

CAMERA_PIN = os.environ.get("RR_CAMERA_PIN", "").strip()
_CAMERA_CLIENT_ID = "676f32727463"      # what go2rtc sends; the robot echoes it back


def _camera_pin_md5(pin: str | None = None) -> str:
    raw = (pin if pin is not None else CAMERA_PIN).strip()
    return hashlib.md5(raw.encode()).hexdigest() if raw else ""


def _cam_obj(value: typing.Any) -> typing.Any:
    """Camera replies come as a dict, a [dict], or a JSON string of one."""
    value = _first(value)
    if isinstance(value, (bytes, bytearray)):
        value = value.decode("utf-8", "replace")
    if isinstance(value, str):
        try:
            return json.loads(value)
        except ValueError:
            return value
    return value


def _b64_json(value: typing.Any) -> typing.Any:
    """dev_sdp / dev_ice entries: base64 of a JSON document (Go []byte fields)."""
    if isinstance(value, (dict, list)):
        return value
    try:
        return json.loads(base64.b64decode(str(value)).decode("utf-8"))
    except Exception:  # noqa: BLE001 -- some firmwares send it unwrapped
        try:
            return json.loads(str(value))
        except ValueError:
            return None


async def _cam_send(method: str, params: typing.Any = None) -> typing.Any:
    async with STATE._lock:
        if (PATROL.active and PATROL.state.get("camera_close_requested")
                and method in ("start_camera_preview", "start_voice_chat", "send_sdp_to_robot", "send_ice_to_robot")):
            raise ValueError("La vidéo est fermée pour le retour à la station.")
        props = await STATE.props()
        cmd = props.command
        return await (cmd.send(method, params) if params is not None else cmd.send(method))


async def handle_camera_status(_request: web.Request) -> web.Response:
    """Read-only: is a viewer connected, what TURN relay the robot holds, camera flags."""
    out: dict[str, typing.Any] = {"pin_configured": bool(CAMERA_PIN)}
    for key, method in (("connect", "get_homesec_connect_status"),
                        ("turn", "get_turn_server"),
                        ("camera", "get_camera_status")):
        try:
            out[key] = _cam_obj(await _cam_send(method))
        except Exception as exc:  # noqa: BLE001 -- the refusal is the answer
            out[key] = {"error": str(exc)}
    turn = out.get("turn")
    if isinstance(turn, dict) and turn.get("pwd"):
        turn["pwd"] = "<redacted>"
    return web.json_response(out)


async def handle_camera_start(request: web.Request) -> web.Response:
    """Open a preview and hand the browser the ICE server to build its peer with."""
    body = await request.json() if request.can_read_body else {}
    pin = _camera_pin_md5(body.get("pin")) if body.get("pin") else _camera_pin_md5()
    if not pin:
        raise web.HTTPBadRequest(reason="code de vue à distance manquant (RR_CAMERA_PIN)")
    try:
        ok = _cam_obj(await _cam_send("check_homesec_password", {"password": pin}))
    except Exception as exc:  # noqa: BLE001
        raise web.HTTPBadGateway(reason=f"vérification du code refusée : {exc}") from exc
    if ok in (False, 0, "false") or (isinstance(ok, dict) and ok.get("status") in (0, False)):
        raise web.HTTPForbidden(reason="code de vue à distance incorrect")
    # A viewer left over from a closed tab holds the camera; go2rtc clears it the same way.
    for _ in range(5):
        st = _cam_obj(await _cam_send("get_homesec_connect_status"))
        cid = st.get("client_id") if isinstance(st, dict) else None
        if not cid or cid == "none":
            break
        await _raw_cam("stop_camera_preview", {"client_id": cid})
        await asyncio.sleep(1)
    try:
        await _cam_send("start_camera_preview", {"client_id": _CAMERA_CLIENT_ID,
                                                 "quality": str(body.get("quality") or "HD"),
                                                 "password": pin})
    except Exception as exc:  # noqa: BLE001
        raise web.HTTPBadGateway(reason=f"la caméra a refusé : {exc}") from exc
    # The robot fetches its relay (/fwapi/createca) only once a preview is asked
    # for, so the first answers are "retry" -- give it a few seconds.
    turn = None
    for _ in range(10):
        turn = _cam_obj(await _raw_cam("get_turn_server"))
        if isinstance(turn, dict) and turn.get("url") and turn.get("url") != "retry":
            break
        await asyncio.sleep(1)
    ice = []
    if isinstance(turn, dict) and turn.get("url") and turn.get("url") != "retry":
        urls = [turn["url"]]
        # The browser may sit where the robot's address is unreachable (UDP blocked,
        # or only 443 open); a TLS form of the same relay, same login, covers it.
        extra = os.environ.get("RR_CAMERA_TURNS", "").strip()
        if extra and extra not in urls:
            urls.append(extra)
        ice.append({"urls": urls, "username": turn.get("user") or "",
                    "credential": turn.get("pwd") or ""})
    _journal("camera", {"step": "start", "turn": (turn or {}).get("url") if isinstance(turn, dict) else None})
    return web.json_response({"ok": True, "ice_servers": ice})


async def _raw_cam(method: str, params: typing.Any = None) -> typing.Any:
    try:
        return await _cam_send(method, params)
    except Exception as exc:  # noqa: BLE001
        _LOGGER.info("camera %s rejected: %s", method, exc)
        return None


# Codecs the robot never answers with, dropped (with their rtx) from the copy of
# the offer it is sent. What it DID answer with -- H.264 + rtx, video red/ulpfec,
# opus -- stays. The browser keeps its full offer: an answer that picks a subset of
# it is valid as-is.
_SDP_DROP_CODECS = {"av1", "vp9", "h265", "g722", "telephone-event", "cn", "flexfec-03",
                    "isac", "ilbc"}


def _slim_sdp(sdp: str) -> str:
    """Shrink an offer below what the firmware will answer.

    Evidence (2026-10-25, Saros 10R): Firefox's 5.4 KB receive-only offer was
    answered, its 5.5 KB one with a microphone and Chrome's 9.3 KB one were not,
    and offers trimmed WITHOUT rtx/red/fec went unanswered at any size. So keep
    the codecs it uses and cut what it never reads: header extensions and the
    codecs it never picks.
    """
    out: list[str] = []
    sections: list[list[str]] = [[]]
    for line in sdp.replace("\r\n", "\n").split("\n"):
        if line.startswith("m="):
            sections.append([])
        if line:
            sections[-1].append(line)
    out += sections[0]
    for sec in sections[1:]:
        names: dict[str, str] = {}
        apt: dict[str, str] = {}
        for ln in sec:
            if ln.startswith("a=rtpmap:"):
                pt, rest = ln[len("a=rtpmap:"):].split(" ", 1)
                names[pt] = rest.split("/", 1)[0].lower()
            elif ln.startswith("a=fmtp:") and "apt=" in ln:
                pt = ln[len("a=fmtp:"):].split(" ", 1)[0]
                apt[pt] = ln.split("apt=", 1)[1].split(";", 1)[0].strip()
        kind = sec[0].split(" ", 1)[0][2:]
        drop = {pt for pt, n in names.items()
                if n in _SDP_DROP_CODECS or (kind == "audio" and n == "red")}
        drop |= {pt for pt, a in apt.items() if a in drop}
        for ln in sec:
            if ln.startswith("m="):
                head = ln.split(" ")
                keep = [pt for pt in head[3:] if pt not in drop]
                out.append(" ".join(head[:3] + keep))
            elif ln.startswith("a=extmap"):
                continue
            elif any(ln.startswith(f"{p}{pt} ") for pt in drop
                     for p in ("a=rtpmap:", "a=fmtp:", "a=rtcp-fb:")):
                continue
            else:
                out.append(ln)
    return "\r\n".join(out) + "\r\n"


async def handle_camera_offer(request: web.Request) -> web.Response:
    """Pass the browser's offer on; wait for the robot's answer."""
    body = await request.json()
    offer = {"type": "offer", "sdp": str(body.get("sdp") or "")}
    if not offer["sdp"]:
        raise web.HTTPBadRequest(reason="sdp required")
    full_len = len(offer["sdp"])
    offer["sdp"] = _slim_sdp(offer["sdp"])
    encoded = base64.b64encode(json.dumps(offer, separators=(",", ":")).encode()).decode()
    _journal("camera", {"step": "offer", "bytes": len(encoded), "sdp_full": full_len,
                        "sdp_slim": len(offer["sdp"]),
                        "agent": request.headers.get("User-Agent", "")[:80],
                        "codecs": sorted({ln.split()[1] for ln in offer["sdp"].splitlines()
                                          if ln.startswith("a=rtpmap:")})})
    try:
        await _cam_send("send_sdp_to_robot", {"app_sdp": encoded})
    except Exception as exc:  # noqa: BLE001
        raise web.HTTPBadGateway(reason=f"offre refusée : {exc}") from exc
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        await asyncio.sleep(1)
        res = _cam_obj(await _raw_cam("get_device_sdp"))
        sdp = res.get("dev_sdp") if isinstance(res, dict) else None
        if not sdp or sdp == "retry":
            continue
        answer = _b64_json(sdp)
        if isinstance(answer, dict) and answer.get("sdp"):
            # The candidate lines say how the robot means to be reached (host /
            # srflx / relay) -- the first thing to read when ICE fails.
            _journal("camera", {"step": "answer",
                                "codecs": [ln.split()[1] for ln in answer["sdp"].splitlines()
                                           if ln.startswith("a=rtpmap:")],
                                "candidates": [
                ln.split(" typ ")[1].split()[0] + " " + " ".join(ln.split()[4:6])
                for ln in answer["sdp"].splitlines() if ln.startswith("a=candidate") and " typ " in ln]})
            return web.json_response({"type": answer.get("type") or "answer", "sdp": answer["sdp"]})
    raise web.HTTPGatewayTimeout(reason="pas de réponse SDP de l'aspirateur")


async def handle_camera_ice(request: web.Request) -> web.Response:
    """POST: one local candidate to the robot. GET: the robot's candidates so far."""
    if request.method == "POST":
        body = await request.json()
        cand = {"candidate": str(body.get("candidate") or ""),
                "sdpMLineIndex": int(body.get("sdpMLineIndex") or 0),
                "sdpMid": str(body.get("sdpMid") if body.get("sdpMid") is not None else "0")}
        if not cand["candidate"]:
            return web.json_response({"ok": True})
        encoded = base64.b64encode(json.dumps(cand, separators=(",", ":")).encode()).decode()
        _journal("camera", {"step": "browser_ice", "candidate": cand["candidate"][:120]})
        await _raw_cam("send_ice_to_robot", {"app_ice": encoded})
        return web.json_response({"ok": True})
    res = _cam_obj(await _raw_cam("get_device_ice"))
    raw = res.get("dev_ice") if isinstance(res, dict) else None
    out = []
    if isinstance(raw, list) and raw:
        _journal("camera", {"step": "device_ice", "candidates": [
            str((_b64_json(i) or {}).get("candidate", ""))[:120] for i in raw]})
    if isinstance(raw, list):
        for item in raw:
            c = _b64_json(item)
            if isinstance(c, dict) and c.get("candidate"):
                out.append({"candidate": c["candidate"],
                            "sdpMid": c.get("sdpMid"), "sdpMLineIndex": c.get("sdpMLineIndex")})
    return web.json_response({"candidates": out})


async def handle_camera_voice(request: web.Request) -> web.Response:
    """Two-way audio on a live preview: `record` = hear the robot's microphone,
    `play` = play the browser's microphone on the robot's speaker. Same calls as
    go2rtc's roborock backchannel: start_voice_chat, then the speaker volume."""
    body = await request.json()
    record, play = bool(body.get("record")), bool(body.get("play"))
    try:
        await _cam_send("start_voice_chat", {"record": record, "play": play})
    except Exception as exc:  # noqa: BLE001
        raise web.HTTPBadGateway(reason=f"audio refusé : {exc}") from exc
    if play:
        volume = max(0, min(100, int(body.get("volume") or 80)))
        await _raw_cam("set_voice_chat_volume", {"volume": volume})
    _journal("camera", {"step": "voice", "record": record, "play": play})
    return web.json_response({"ok": True})


# --------------------------------------------------------------------------- #
# Soundboard: short clips played through the robot's speaker over the live
# camera session. Stored as uploaded (the browser decodes them and sends them as
# the session's audio track), next to the other panel data, with a small index.
SOUNDS_DIR = Path(os.environ.get("RR_SOUNDS_DIR", str(Path(PRESETS_PATH).parent / "sounds")))
SOUND_MAX_BYTES = 8 * 1024 * 1024
_SOUND_TYPES = {"audio/mpeg": "mp3", "audio/mp3": "mp3", "audio/ogg": "ogg", "audio/opus": "opus",
                "audio/wav": "wav", "audio/x-wav": "wav", "audio/wave": "wav", "audio/webm": "webm",
                "audio/mp4": "m4a", "audio/x-m4a": "m4a", "audio/aac": "aac", "audio/flac": "flac"}


def _sounds_index() -> list[dict]:
    try:
        data = json.loads((SOUNDS_DIR / "index.json").read_text(encoding="utf-8"))
        return data if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def _sounds_save(items: list[dict]) -> None:
    SOUNDS_DIR.mkdir(parents=True, exist_ok=True)
    tmp = SOUNDS_DIR / "index.json.tmp"
    tmp.write_text(json.dumps(items, ensure_ascii=False, indent=1), encoding="utf-8")
    tmp.replace(SOUNDS_DIR / "index.json")


def _sound_public(item: dict) -> dict:
    out = {k: item.get(k) for k in ("id", "name", "size", "type")}
    out["gain"] = float(item.get("gain") or 1.0)
    return out


async def handle_sounds(_request: web.Request) -> web.Response:
    return web.json_response({"sounds": [_sound_public(i) for i in _sounds_index()]})


async def handle_sound_upload(request: web.Request) -> web.Response:
    """multipart: `file` (an audio file) and `name`; with `replace` (a sound id)
    the file takes that sound's place -- same name, gain and position."""
    reader = await request.multipart()
    name, blob, ctype, replace = "", b"", "", ""
    async for part in reader:
        if part.name == "name":
            name = (await part.text()).strip()[:40]
        elif part.name == "replace":
            replace = (await part.text()).strip()
        elif part.name == "file":
            ctype = (part.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            blob = bytes(await part.read())
    if not blob:
        raise web.HTTPBadRequest(reason="fichier manquant")
    if len(blob) > SOUND_MAX_BYTES:
        raise web.HTTPRequestEntityTooLarge(max_size=SOUND_MAX_BYTES, actual_size=len(blob))
    ext = _SOUND_TYPES.get(ctype)
    if ext is None:
        raise web.HTTPUnsupportedMediaType(reason=f"type audio non pris en charge : {ctype or '?'}")
    sid = hashlib.blake2b(blob, digest_size=6).hexdigest()
    items = _sounds_index()
    old = next((i for i in items if i.get("id") == replace), None) if replace else None
    SOUNDS_DIR.mkdir(parents=True, exist_ok=True)
    (SOUNDS_DIR / f"{sid}.{ext}").write_bytes(blob)
    entry = {"id": sid, "name": name or (old or {}).get("name") or "Son", "file": f"{sid}.{ext}",
             "type": ctype, "size": len(blob), "gain": (old or {}).get("gain", 1.0),
             "added": time.time()}
    if old is not None:
        items[items.index(old)] = entry
        if old.get("file") != entry["file"]:
            try:
                (SOUNDS_DIR / str(old.get("file"))).unlink()
            except OSError:
                pass
    else:
        items = [i for i in items if i.get("id") != sid] + [entry]
    _sounds_save(items)
    return web.json_response({"ok": True, "sound": _sound_public(entry)})


async def handle_sound_edit(request: web.Request) -> web.Response:
    """{id, name?, gain?} edits; {id, delete: true} removes."""
    body = await request.json()
    sid = str(body.get("id") or "")
    items = _sounds_index()
    hit = next((i for i in items if i.get("id") == sid), None)
    if hit is None:
        raise web.HTTPNotFound(reason="son inconnu")
    if body.get("delete"):
        items.remove(hit)
        try:
            (SOUNDS_DIR / str(hit.get("file"))).unlink()
        except OSError:
            pass
    else:
        if "name" in body:
            hit["name"] = str(body.get("name") or "").strip()[:40] or hit["name"]
        if "gain" in body:
            # 0..3: up to three times louder for a quiet clip, never so loud that
            # the robot's small speaker turns it into distortion by default.
            hit["gain"] = max(0.0, min(3.0, float(body.get("gain") or 0)))
    _sounds_save(items)
    return web.json_response({"ok": True})


async def handle_sound_file(request: web.Request) -> web.Response:
    sid = request.query.get("id", "")
    hit = next((i for i in _sounds_index() if i.get("id") == sid), None)
    if hit is None:
        raise web.HTTPNotFound()
    path = SOUNDS_DIR / str(hit.get("file"))
    if not path.is_file():
        raise web.HTTPNotFound()
    # Content-addressed (the id is the file's hash), so it never changes.
    return web.FileResponse(path, headers={"Content-Type": hit.get("type") or "application/octet-stream",
                                           "Cache-Control": "public, max-age=31536000, immutable"})


# Choreographies: a text program of timed moves and sound cues (parsed and run by
# the browser; see the Chorégraphies sheet). Stored as written, so the text the
# user edited is exactly what comes back.
CHOREO_PATH = Path(os.environ.get("RR_CHOREO_DB", str(Path(PRESETS_PATH).parent / "choreographies.json")))
CHOREO_MAX_TEXT = 64 * 1024


def _choreos() -> list[dict]:
    try:
        data = json.loads(CHOREO_PATH.read_text(encoding="utf-8"))
        return data if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def _choreos_save(items: list[dict]) -> None:
    CHOREO_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = CHOREO_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(items, ensure_ascii=False, indent=1), encoding="utf-8")
    tmp.replace(CHOREO_PATH)


async def handle_choreos(request: web.Request) -> web.Response:
    """GET: the list. POST {id?, name, text}: create or update. POST {id, delete}."""
    if request.method == "GET":
        return web.json_response({"choreos": _choreos()})
    body = await request.json()
    items = _choreos()
    cid = str(body.get("id") or "")
    hit = next((c for c in items if c.get("id") == cid), None) if cid else None
    if body.get("delete"):
        if hit is not None:
            items.remove(hit)
            _choreos_save(items)
        return web.json_response({"ok": True})
    text = str(body.get("text") or "")
    if len(text) > CHOREO_MAX_TEXT:
        raise web.HTTPRequestEntityTooLarge(max_size=CHOREO_MAX_TEXT, actual_size=len(text))
    name = str(body.get("name") or "").strip()[:60] or "Chorégraphie"
    if hit is None:
        hit = {"id": hashlib.blake2b(f"{time.time()}{name}".encode(), digest_size=6).hexdigest()}
        items.append(hit)
    hit.update(name=name, text=text, updated=time.time())
    _choreos_save(items)
    return web.json_response({"ok": True, "choreo": hit})


async def handle_camera_stop(_request: web.Request) -> web.Response:
    await _raw_cam("stop_camera_preview", {"client_id": _CAMERA_CLIENT_ID})
    _journal("camera", {"step": "stop"})
    return web.json_response({"ok": True})


# Remote control: app_rc_start puts the robot in manual mode, app_rc_move drives
# it for `duration` ms (sent repeatedly while a direction is held), app_rc_end
# leaves manual mode. Speeds are clamped to the ranges the official clients use:
# velocity +-0.3 m/s, omega +-pi rad/s.
_RC_SEQ = 0


async def handle_rc(request: web.Request) -> web.Response:
    global _RC_SEQ
    body = await request.json()
    action = str(body.get("action") or "")
    _patrol_available()
    if action == "start":
        _RC_SEQ = 0
        method, params = "app_rc_start", None
    elif action == "stop":
        method, params = "app_rc_end", None
    elif action == "move":
        _RC_SEQ += 1
        velocity = max(-0.3, min(0.3, float(body.get("velocity") or 0)))
        omega = max(-3.1, min(3.1, float(body.get("omega") or 0)))
        duration = max(100, min(1500, int(body.get("duration") or 500)))
        method, params = "app_rc_move", [{"omega": round(omega, 3), "velocity": round(velocity, 3),
                                          "seqnum": _RC_SEQ, "duration": duration}]
    else:
        raise web.HTTPBadRequest(reason="action must be start, move or stop")
    try:
        result = await _cam_send(method, params)
    except Exception as exc:  # noqa: BLE001
        raise web.HTTPBadGateway(reason=f"télécommande refusée : {exc}") from exc
    if action != "move":
        _journal("remote_control", {"action": action, "result": result})
    return web.json_response({"ok": True, "result": result})


# Saved remote-view routes. Coordinates are stored in the robot's map frame so
# changing the image size does not shift the saved destinations.
PATROL_ROUTES = patrol_mod.RouteStore(Path(PRESETS_PATH).parent / "patrol_routes.json")


async def _patrol_send(duid, method, params):
    async with STATE._lock:
        if STATE.selected_duid != duid:
            raise ValueError("Le robot sélectionné a changé.")
        props = await STATE.props()
        result = await props.command.send(method, params) if params is not None else await props.command.send(method)
        if isinstance(result, dict) and (result.get("error") or result.get("code", 0)):
            raise ValueError(f"Commande refusée : {method}")
        return result


async def _patrol_snapshot(duid, position):
    async with STATE._lock:
        if STATE.selected_duid != duid:
            raise ValueError("Le robot sélectionné a changé.")
        props = await STATE.props()
        await props.status.refresh()
        st = props.status
        out = {"state": _int_or_none(st.state), "error": _int_or_none(st.error_code),
               "in_cleaning": _int_or_none(st.in_cleaning), "map_flag": props.maps.current_map,
               "home_sec_status": _int_or_none(getattr(st, "home_sec_status", None))}
        if position:
            await props.map_content.refresh()
            md = props.map_content.map_data
            point = getattr(md, "vacuum_position", None) or getattr(md, "robot_position", None)
            proj, convert = _map_projector(md), _image_to_vacuum(md)
            if point and proj and convert:
                pct = _point_pct(proj[0], point)
                if pct:
                    out["position"] = convert(pct[0]*proj[1]/100, pct[1]*proj[2]/100)
            target = patrol_mod.station_approach(getattr(md, "charger", None), proj, convert)
            if target:
                out["dock_approach"] = target
        return out


PATROL = patrol_mod.Patrol(_patrol_send, _patrol_snapshot)


def _patrol_available():
    if PATROL.active:
        raise web.HTTPConflict(reason="Arrêtez le parcours avant cette action.")


async def handle_patrol_map(_request):
    async with STATE._lock:
        props = await STATE.props()
        await props.maps.refresh()
        await props.map_content.refresh()
        md = props.map_content.map_data
        proj, convert = _map_projector(md), _image_to_vacuum(md)
        if not proj or not convert or props.maps.current_map is None:
            raise web.HTTPBadRequest(reason="Carte indisponible.")
        _, w, h = proj
        return web.json_response({"w": w, "h": h, "map_flag": props.maps.current_map,
                                  "duid": STATE.selected_duid, "origin": convert(0, 0), "x_axis": convert(w, 0),
                                  "y_axis": convert(0, h), "bg": _map_bg_hex()})


async def handle_patrol_routes(request):
    try:
        duid = STATE.selected_duid
        if not duid:
            raise ValueError("Sélectionnez un robot.")
        if request.method == "GET":
            return web.json_response({"routes": PATROL_ROUTES.list(duid), "status": PATROL.status()})
        body = await request.json()
        async with STATE._lock:
            props = await STATE.props()
            flag = props.maps.current_map
            if not body.get("delete") and (flag is None or body.get("map_flag") != flag):
                raise ValueError("La carte a changé. Rouvrez Tap & Go.")
            route = PATROL_ROUTES.update(duid, body, flag)
        return web.json_response({"ok": True, "route": route})
    except (ValueError, TypeError, KeyError) as exc:
        raise web.HTTPBadRequest(reason=str(exc)) from exc


async def handle_patrol_session(request):
    if request.method == "GET":
        return web.json_response(PATROL.status())
    try:
        body = await request.json()
        action = body.get("action")
        if action == "start":
            if not STATE.selected_duid:
                raise ValueError("Sélectionnez un robot.")
            steps, flag, name = body.get("steps"), body.get("map_flag"), "Tap & Go"
            if body.get("id"):
                route = next((r for r in PATROL_ROUTES.list(STATE.selected_duid) if r["id"] == body["id"]), None)
                if not route:
                    raise ValueError("Parcours introuvable.")
                steps, flag, name = route["steps"], route["map_flag"], route["name"]
            token = await PATROL.start(STATE.selected_duid, flag, steps, name,
                                       end_session=bool(body.get("id") or body.get("end_session", True)))
            _journal("patrol", {"action": "start", "name": name, "steps": steps})
            return web.json_response({"token": token, **PATROL.status()})
        if action == "heartbeat":
            return web.json_response(PATROL.touch(body.get("token")))
        if action == "camera_closed":
            return web.json_response(PATROL.camera_closed(body.get("token")))
        if action == "stop":
            result = await PATROL.stop(body.get("token"))
            _journal("patrol", {"action": "stop"})
            return web.json_response(result)
        raise ValueError("Action de parcours inconnue.")
    except (ValueError, TypeError, KeyError) as exc:
        raise web.HTTPBadRequest(reason=str(exc)) from exc


async def _stop_patrol(_app):
    await PATROL.stop()


async def handle_dock_action(request: web.Request) -> web.Response:
    body = await request.json()
    action = str(body.get("action") or "")
    stop = bool(body.get("stop"))
    async with STATE._lock:
        props = await STATE.props()
        cmd = props.command
        if action == "dry":
            # Drying has no start/stop pair -- it takes a status flag.
            try:
                result = await _set_dryer(cmd, 0 if stop else 1)
            except RoborockException as exc:
                raise web.HTTPBadGateway(reason=f"dryer rejected: {exc}")
            return web.json_response({"ok": True, "action": "dry", "stop": stop, "result": result})
        pair = _DOCK_ACTIONS.get(action)
        if not pair:
            raise web.HTTPBadRequest(reason="action must be dust, wash or dry")
        try:
            await cmd.send(pair[1] if stop else pair[0])
        except RoborockException as exc:
            raise web.HTTPBadGateway(reason=f"{action} rejected: {exc}")
    return web.json_response({"ok": True, "action": action, "stop": stop})


async def handle_setting(request: web.Request) -> web.Response:
    body = await request.json()
    key = str(body.get("key", ""))
    value = body.get("value")
    async with STATE._lock:
        props = await STATE.props()
        if key == "volume":
            await props.sound_volume.set_volume(int(value))
        elif key == "dnd":
            await (props.dnd.enable() if value else props.dnd.disable())
        elif key == "child_lock" and props.child_lock is not None:
            await (props.child_lock.enable() if value else props.child_lock.disable())
        elif key == "led" and props.led_status is not None:
            await (props.led_status.enable() if value else props.led_status.disable())
        elif key == "flow_led" and props.flow_led_status is not None:
            await (props.flow_led_status.enable() if value else props.flow_led_status.disable())
        else:
            raise web.HTTPBadRequest(reason=f"unknown/unsupported setting {key!r}")
    return web.json_response({"ok": True})


async def handle_dnd(request: web.Request) -> web.Response:
    body = await request.json()
    timer = DnDTimer(
        start_hour=int(body["start_hour"]), start_minute=int(body["start_minute"]),
        end_hour=int(body["end_hour"]), end_minute=int(body["end_minute"]), enabled=True,
    )
    async with STATE._lock:
        props = await STATE.props()
        await props.dnd.set_dnd_timer(timer)
    return web.json_response({"ok": True})


async def handle_map_select(request: web.Request) -> web.Response:
    _patrol_available()
    body = await request.json()
    flag = int(body["map_flag"])
    async with STATE._lock:
        props = await STATE.props()
        await props.maps.set_current_map(flag)
    _drop_map_ttl()          # a different floor is not the map we are holding
    return web.json_response({"ok": True})


async def handle_map_reset(_request: web.Request) -> web.Response:
    _patrol_available()
    async with STATE._lock:
        props = await STATE.props()
        result = await props.command.send(RoborockCommand.RESET_MAP)
    _drop_map_ttl()          # there is no map now; do not serve the old one
    return web.json_response({"ok": True, "result": result})


async def handle_room_rename(request: web.Request) -> web.Response:
    """Rename a room. name_segment expects ALL segments resent as objects
    {miRoomId: <name string>, robotRoomId: <id int>} (Valetudo-confirmed);
    the ones not being renamed keep their current names."""
    body = await request.json()
    seg = int(body["segment_id"])
    name = str(body["name"]).strip()[:23]
    async with STATE._lock:
        props = await STATE.props()
        await props.rooms.refresh()
        payload = []
        for s, rm in props.rooms.room_map.items():
            # Resend every other room's CURRENT name. Reading this back through
            # _room_name matters: raw_name is None on this stack, and sending ""
            # for the untouched rooms would blank them all.
            nm = name if int(s) == seg else _room_name(rm, int(s))
            payload.append({"miRoomId": nm, "robotRoomId": int(s)})
        result = await props.command.send(RoborockCommand.NAME_SEGMENT, payload)
    # The timeline labels rooms by their name NOW, so the held copy is stale the
    # moment this returns -- and the names ride along in the map.
    _SEGMENT_NAMES_CACHE.update(at=0.0, names=None)
    _drop_map_ttl()
    return web.json_response({"ok": True, "result": result})


async def handle_room_merge(request: web.Request) -> web.Response:
    body = await request.json()
    a, b = int(body["a"]), int(body["b"])
    async with STATE._lock:
        props = await STATE.props()
        result = await props.command.send(RoborockCommand.MERGE_SEGMENT, [a, b])
    _LOGGER.info("merge_segment %s+%s", a, b)
    return web.json_response({"ok": True, "result": result})


async def handle_room_split(request: web.Request) -> web.Response:
    """split_segment [seg_id, x1,y1, x2,y2] in vacuum mm; line drawn in image px.
    The dividing line must terminate on walls or the robot rejects it."""
    body = await request.json()
    seg = int(body["segment_id"])
    line = body["line"]  # [x0,y0,x1,y1] image px
    async with STATE._lock:
        props = await STATE.props()
        await props.map_content.refresh()
        to_vac = _image_to_vacuum(props.map_content.map_data)
        if to_vac is None:
            raise web.HTTPBadRequest(reason="no map calibration")
        (ax, ay), (bx, by) = to_vac(line[0], line[1]), to_vac(line[2], line[3])
        result = await props.command.send(RoborockCommand.SPLIT_SEGMENT, [seg, ax, ay, bx, by])
    _LOGGER.info("split_segment %s along %s", seg, [ax, ay, bx, by])
    return web.json_response({"ok": True, "result": result})


async def handle_room_material(request: web.Request) -> web.Response:
    """set_segment_ground_material: {"data":[[seg, material, dir]]} (fallback to bare)."""
    body = await request.json()
    seg = int(body["segment_id"])
    material = int(body["material"])
    direction = int(body.get("direction", 0))
    triple = [seg, material, direction]
    async with STATE._lock:
        props = await STATE.props()
        cmd = props.command
        for params in ({"data": [triple]}, [triple]):
            try:
                result = await cmd.send(RoborockCommand.SET_SEGMENT_GROUND_MATERIAL, params)
                return web.json_response({"ok": True, "result": result})
            except RoborockException as exc:
                _LOGGER.info("ground_material %r rejected: %s", params, exc)
    raise web.HTTPBadRequest(reason="set_segment_ground_material rejected in both forms")


async def handle_carpet_get(_request: web.Request) -> web.Response:
    """Query the vacuum's CURRENT carpet clean mode (get_carpet_clean_mode)."""
    async with STATE._lock:
        props = await STATE.props()
        mode = None
        err = None
        try:
            resp = await props.command.send(RoborockCommand.GET_CARPET_CLEAN_MODE)
            obj = resp[0] if isinstance(resp, list) and resp else resp
            if isinstance(obj, dict):
                mode = obj.get("carpet_clean_mode")
        except Exception as exc:  # noqa: BLE001
            err = str(exc)
    return web.json_response({"mode": mode, "options": CARPET_CLEAN_MODES, "error": err})


async def handle_carpet_mode(request: web.Request) -> web.Response:
    """set_carpet_clean_mode {"carpet_clean_mode": n}. 0 avoid / 1,3 lift / 2 off."""
    body = await request.json()
    mode = int(body["mode"])
    async with STATE._lock:
        props = await STATE.props()
        result = await props.command.send(RoborockCommand.SET_CARPET_CLEAN_MODE, {"carpet_clean_mode": mode})
    return web.json_response({"ok": True, "result": result})


async def handle_experimental(request: web.Request) -> web.Response:
    """Best-effort toggles whose exact payloads are undocumented (door sill / cliff).
    Params are a guess (on/off); tries [flag] then [{"enable":flag}]. Clearly
    labelled experimental in the UI."""
    body = await request.json()
    which = str(body.get("which", ""))
    flag = 1 if body.get("enabled") else 0
    cmd_map = {
        "smart_door_sill": RoborockCommand.APP_SET_SMART_DOOR_SILL,
        "smart_cliff": RoborockCommand.APP_SET_SMART_CLIFF_FORBIDDEN,
    }
    command = cmd_map.get(which)
    if command is None:
        raise web.HTTPBadRequest(reason=f"unknown experimental toggle {which!r}")
    async with STATE._lock:
        props = await STATE.props()
        if await _apply_setting(props.command, command, "enable", flag):
            return web.json_response({"ok": True})
    raise web.HTTPBadRequest(reason=f"{which} rejected (payload guess wrong)")


async def handle_consumable_reset(request: web.Request) -> web.Response:
    body = await request.json()
    key = str(body.get("key", ""))
    entry = CONSUMABLES.get(key)
    if entry is None:
        raise web.HTTPBadRequest(reason=f"unknown consumable {key!r}")
    async with STATE._lock:
        props = await STATE.props()
        await props.consumables.reset_consumable(entry[0])
    return web.json_response({"ok": True})


async def handle_routine_run(request: web.Request) -> web.Response:
    body = await request.json()
    rid = int(body["id"])
    async with STATE._lock:
        props = await STATE.props()
        await props.routines.execute_routine(rid)
    return web.json_response({"ok": True})


# --------------------------------------------------------------------------- #
# Wiring
# --------------------------------------------------------------------------- #
async def _login_task(app: web.Application) -> None:
    """Log in and discover devices, retrying until it works."""
    delay = 5
    while True:
        try:
            await STATE.login()
            return
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 -- the panel stays up regardless
            _LOGGER.warning("login/discovery failed, retrying in %ss: %s", delay, exc)
            await asyncio.sleep(delay)
            delay = min(delay * 2, 120)


async def on_startup(app: web.Application) -> None:
    """Bring the HTTP server up first; find the vacuum afterwards.

    Discovery is not fast and not entirely under our control: the library gives
    the first connection attempt a fixed 15s before it stops waiting on it, and
    a vacuum that is offline or unreachable spends all of it. Blocking here made
    that the application's startup time, and made an absent vacuum look like an
    absent panel -- nothing answered port 8099 at all.

    So it runs alongside instead. Every handler already copes with there being
    no device yet (/api/state returns the shell of a payload, the sampler skips
    its tick), and the panel polls, so the page fills itself in the moment the
    device shows up. It also retries, which awaiting here never did: a login
    that failed at boot used to stay failed until someone restarted the
    container.
    """
    app["login"] = asyncio.create_task(_login_task(app))


async def on_cleanup(app: web.Application) -> None:
    task = app.get("login")
    if task and not task.done():
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    await STATE.close()


async def _sampler(app: web.Application) -> None:
    """The one thing in the panel that polls.

    Error durations and room progress must be right even when nobody has the
    page open -- an overnight jam should read "7h", not "since you opened the
    tab". That is what this has always been for, and it is now also the only
    poller: the browsers watch a WebSocket, and this is what decides how often
    the vacuum is actually asked anything.

    Four cadences, because the reasons differ:
      * nobody watching, idle          -- 60s, purely to keep the run log honest
      * nobody watching, a job running -- 30s, same, plus room progress
      * watching, idle                 -- 30s; the datapoint push carries the
        news, so this is only a safety net for the numbers it does not cover
      * watching, a job running        -- 4s for the progress numbers, and the
        map on its own slower timer because it is ~400 KB a time
    Anything a datapoint already reports (state, battery, error, fan, water,
    charge, drying) arrives by push in between, whatever this is doing.
    """
    await asyncio.sleep(2)
    last_map = 0.0
    last_pos = 0.0
    last_tl = 0.0
    last_wear = 0.0
    was_active = False
    tl_live = False        # the last timeline frame had events in it
    while True:
        interval = TRACK_INTERVAL_IDLE
        # Collected inside the lock, sent outside it: ws.send_str awaits the
        # socket draining, and a browser on a stalled connection would otherwise
        # hold STATE._lock -- the vacuum's single command queue -- while it did.
        ended = False
        live_obstacles = None
        timeline = None
        try:
            async with STATE._lock:
                if STATE.devices:
                    props = await STATE.props()
                    await props.status.refresh()
                    st = props.status
                    duid = STATE.selected_duid or ""
                    state = _int_or_none(st.state)
                    err = _int_or_none(st.error_code) or 0
                    if duid:
                        TRACKER.observe_error(duid, err)
                    moving = state not in (None, RoborockStateCode.idle.value, RoborockStateCode.charging.value)
                    watched = len(HUB) > 0
                    if moving or err:
                        interval = WATCH_INTERVAL_ACTIVE if watched else TRACK_INTERVAL_ACTIVE
                    elif watched:
                        interval = WATCH_INTERVAL_IDLE

                    # Seed the consumable counters, so the wear badge has real
                    # numbers to work from rather than an unread trait's zeros.
                    if time.time() - last_wear >= WEAR_REFRESH:
                        try:
                            await props.consumables.refresh()
                            last_wear = time.time()
                        except Exception:  # noqa: BLE001 -- a nicety, never fatal
                            _LOGGER.debug("consumable refresh failed", exc_info=True)

                    # Fold this poll into the run log. This is the only place the
                    # settings a run used ever get written down, and it runs off
                    # the MQTT poll, so it works with no browser attached.
                    in_cleaning = _int_or_none(st.in_cleaning) or 0
                    active = bool(in_cleaning) or state in CLEANING_STATES
                    if duid:
                        RUNS.observe(duid, {
                            "active": active,
                            "in_cleaning": in_cleaning,
                            "error": err,
                            "fan": _int_or_none(st.fan_power),
                            "water": _int_or_none(st.water_box_mode),
                            "mop": _int_or_none(st.mop_mode),
                            "repeat": _int_or_none(st.repeat),
                            "clean_time": _int_or_none(st.clean_time),
                            "clean_area": _int_or_none(st.clean_area),
                            "clean_percent": _int_or_none(st.clean_percent),
                        })

                    # Keep the map current. Coverage -- and therefore the whole
                    # resume offer -- is read off the path trace, so if nothing
                    # refreshed it here, resume would silently go missing until
                    # some other request happened to load a map. It is also what
                    # moves the robot on screen, which is why it goes faster while
                    # somebody is watching a job -- but never per-tick, because it
                    # is ~400 KB every time.
                    now = time.time()
                    gap = (WATCH_MAP_ACTIVE if watched else TRACK_INTERVAL_ACTIVE) \
                        if (active or err) else MAP_IDLE_GAP
                    try:
                        if now - last_map >= gap:
                            await props.map_content.refresh()
                            last_map = now
                            # Stamped, so /api/state can use this copy instead of
                            # pulling 400 KB of its own a second later.
                            _MAP_READ_AT[0] = now
                            _note_map_bytes(props)
                            # This parse read the position too, and from the same
                            # blob, so the separate read has nothing to add -- and
                            # its clock restarts from here rather than firing again
                            # immediately after.
                            _LIVE_POS["pct"] = None
                            last_pos = now
                            # And it read the obstacles. Harvesting them here costs
                            # nothing extra -- the map is already parsed -- and it
                            # is the only moment they exist: the run's own saved
                            # map keeps a subset, and the photos are recycled.
                            # One parse, two readers: the browsers and the archive.
                            live_obstacles = _obstacle_list(props.map_content.map_data)
                            if active and duid:
                                open_run = RUNS.open_run(duid)
                                if open_run and open_run.get("started_at"):
                                    new = ARCHIVE.note(duid, open_run["started_at"],
                                                       live_obstacles)
                                    if new:
                                        _LOGGER.info("obstacle archive: %d new detection(s) "
                                                     "during this run (%d seen)",
                                                     new, len(live_obstacles))
                        elif watched and active and now - last_pos >= _position_gap():
                            # Between picture refreshes: the badge only. Same blob
                            # off the wire, but parsed with nothing drawn, so it is
                            # the transfer and not the render -- and the map the
                            # browsers are showing is left alone.
                            await _read_position(props)
                            last_pos = now
                    except Exception:  # noqa: BLE001 -- map is best effort
                        _LOGGER.debug("sampler map refresh failed", exc_info=True)

                    # Docked and idle: whatever we launched is over, so stop
                    # suppressing the coverage check for the rooms it skipped.
                    if duid and state == RoborockStateCode.charging.value and not err and not in_cleaning:
                        TRACKER.clear_launch(duid)

                    # Kept reading for a little after the job ends, not only
                    # while it runs: the device holds the finished run's events
                    # briefly and the panel shows them, then clears itself when the
                    # device does -- which is only visible if we are still looking.
                    # Its own gap: the badge moves every tick, but a room's worth
                    # of events does not, and this is a paged read.
                    if watched and (active or tl_live) and now - last_tl >= WATCH_TL_GAP:
                        timeline = await _read_timeline(props)
                        global _LAST_TIMELINE
                        _LAST_TIMELINE = timeline
                        last_tl = now
                        tl_live = bool(timeline and timeline.get("available"))
                    ended = was_active and not active
                    if active and not was_active and _LAST_TRACE["pts"]:
                        # A new run has begun: whatever is pinned is now the run
                        # before this one, so stop drawing it. Cleared again when
                        # this run's own map is read back (see _pin_trace).
                        _LAST_TRACE["stale"] = True
                        _hcache_touch()
                        _kick_trace()
                    was_active = active
                    if ended:
                        _kick_trace()    # the route on screen switches to the pinned one
                        # Its own task: this waits out the settle window, and the
                        # sampler must not stop ticking for a minute to do it.
                        asyncio.create_task(_after_run())
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 -- sampler must never die
            _LOGGER.debug("sampler poll failed: %s", exc)
        # The lock is released. Tell the watchers what this tick found: the
        # progress numbers no datapoint carries, a map that actually moved, the
        # running job's events, and -- once -- that a job has ended, which makes
        # the history a run short and the resume offer moot.
        try:
            # The badge lives in the status frame, so a position that moved is a
            # status that changed -- Hub.send suppresses it when it is not.
            await push_status()
            # Unconditional: push_map recomputes the token and Hub.send drops a
            # frame that says nothing new, so this covers the map bytes changing,
            # a run starting or stopping (which switches the trace), and a trace
            # being pinned -- without any of them having to remember to say so.
            await push_map()
            if live_obstacles is not None:
                await push_obstacles(live_obstacles)
            if timeline is not None:
                await HUB.send("timeline", {"timeline": timeline})
            if ended:
                await HUB.send("runs", {"at": time.time()}, force=True)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            _LOGGER.debug("sampler push failed: %s", exc)
        await asyncio.sleep(interval)


# --- warming the history in the background --------------------------------
# The point of the caches is that the panel never waits for MQTT to draw a run.
# That only holds if something filled them before anyone asked, so this does --
# on the server, once, for every client, and out of everybody's way:
#   * nothing at all while the robot is working, or while any request is waiting
#     for the lock: those MQTT slots belong to the live poll, and a run that is
#     still going is not the run anyone is about to open;
#   * the lock is taken for ONE command at a time, with a gap after each, so a
#     status poll or a room tap always gets in between two of them;
#   * the newest run gets its déroulement and its detections, because that is the
#     one that gets opened. Older runs get their record only -- cheap, and enough
#     for the list to draw instantly.
WARM_DELAY = 20.0        # after start: the first page load comes first
WARM_INTERVAL = 600.0    # a lap; new runs arrive slower than this
WARM_GAP = 1.5           # between two commands, so live traffic is never queued behind us
WARM_RUNS = 20           # the window the history sheet shows


async def _warm_pass() -> None:
    if not STATE.devices:
        return
    async with STATE._lock:
        props = await STATE.props()
        st = props.status
        if _is_running(_int_or_none(st.state), st.in_cleaning):
            return
        await props.clean_summary.refresh()
        ids = [int(r) for r in list(props.clean_summary.records or [])[:WARM_RUNS]]
    if not ids:
        return

    for rid in ids:
        if rid in _CLEAN_RECORD_CACHE:
            continue
        async with STATE._lock:
            await _clean_record(await STATE.props(), rid)
        await asyncio.sleep(WARM_GAP)

    newest = ids[0]
    if newest not in _RUN_EVENTS_CACHE:
        async with STATE._lock:
            props = await STATE.props()
            await _run_events(props, newest, _CLEAN_RECORD_CACHE.get(newest))
        await asyncio.sleep(WARM_GAP)
    await _capture_run_map(newest)
    ARCHIVE.prune()
    _hcache_save()


async def _capture_run_map(begin: int) -> None:
    """Read a finished run's own saved map, once, for the two things it carries.

    Its PATH, which becomes the trace the map shows while nothing is running (the
    live map's own trace stops being about the run that just ended), and its
    DETECTIONS, merged into what was harvested live -- neither set is complete on
    its own, so the union is the honest answer to what it drove around.

    Wanted for every finished run, not only ones that avoided something: the path
    is the point. One ~400 KB read per run, and only once it has settled, because
    the vacuum writes that map as the run ends and a read that lands mid-write
    gets a short answer.
    """
    if not _record_settled(_CLEAN_RECORD_CACHE.get(begin)):
        return
    have_trace = int(_LAST_TRACE["begin"] or 0) == int(begin)
    if begin in _RECORD_MAP_CACHE and have_trace:
        return                       # both already taken off this run's map
    if not have_trace:
        # The detections may be cached from a previous process, and
        # _run_obstacle_list answers from that cache without parsing -- so the
        # trace, which only exists inside the parse, would never be taken. Drop
        # the entry so the map is actually read. It happens once per run: the
        # pinned trace is persisted, so the next start already has it.
        _RECORD_MAP_CACHE.pop(begin, None)
    async with STATE._lock:
        saved = await _run_obstacle_list(await STATE.props(), begin)
    if saved is None:
        return
    run = RUNS.for_run(STATE.selected_duid or "", begin)
    run_key = (run or {}).get("started_at") or begin
    # note() before set_begin(): set_begin only annotates a run it already has, and
    # a run with no live harvest behind it (the panel started mid-clean, or the job
    # was shorter than one sampler tick) would otherwise be left with begin=None
    # and only the start-time join to find it by.
    added = ARCHIVE.note(STATE.selected_duid or "", run_key, saved, source="record")
    ARCHIVE.set_begin(run_key, begin)
    if added:
        _LOGGER.info("obstacle archive: run %s map added %d detection(s) the live "
                     "harvest had not seen", begin, added)


async def _warm_map() -> None:
    """Do the map's derived work before the first browser asks for it.

    Everything /api/state computes off the map -- the room-only render, the
    per-pixel room index, the room centroids -- is cached per map, and whoever
    calls first pays for all of it. It is pure CPU on bytes we already have, so
    the fix is simply to be the one who calls first.
    """
    async with STATE._lock:
        props = await STATE.props()
        await props.rooms.refresh()
        await props.map_content.refresh()
        _MAP_READ_AT[0] = time.time()
        _note_map_bytes(props)      # the first version, before anybody has asked
        clean_render = (_clean_room_render(props.map_content.raw_api_response)
                        or props.map_content.image_content)
        # Fills _CLEAN_RENDER_CACHE, _ROOM_PIXELS_CACHE and _ROOM_INDEX_CACHE, in
        # that order, which is every derived thing /api/state needs.
        _room_index_payload(clean_render, [int(seg) for seg in props.rooms.room_map])


async def _after_run() -> None:
    """Take what a just-finished run leaves behind, without waiting for a lap.

    The warmer would get to it eventually, but "eventually" is up to ten minutes
    and the map is showing the wrong trace for all of it. So this waits out the
    settle window and goes straight for it.
    """
    await asyncio.sleep(_RECORD_SETTLE + 15)
    try:
        async with STATE._lock:
            props = await STATE.props()
            await props.clean_summary.refresh()
            ids = list(props.clean_summary.records or [])
        if not ids:
            return
        newest = int(ids[0])
        async with STATE._lock:
            await _clean_record(await STATE.props(), newest)
        await _capture_run_map(newest)
        await push_map()          # the trace changed, so the picture has
        _hcache_save()
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # noqa: BLE001 -- the warmer is still the backstop
        _LOGGER.info("post-run capture failed (%s); the warmer will retry",
                     exc.__class__.__name__)


async def _history_warmer(app: web.Application) -> None:
    # Login happens on its own task; there is nothing to warm until it lands.
    for _ in range(30):
        if STATE.devices:
            break
        await asyncio.sleep(2)
    try:
        await _warm_map()
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # noqa: BLE001
        _LOGGER.debug("map warm failed: %s", exc)
    await asyncio.sleep(WARM_DELAY)
    while True:
        try:
            await _warm_pass()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 -- a warm-up must never take the app down
            _LOGGER.debug("history warm pass failed: %s", exc)
        await asyncio.sleep(WARM_INTERVAL)


async def _start_warmer(app: web.Application) -> None:
    _hcache_load()
    app["warmer"] = asyncio.create_task(_history_warmer(app))


async def _stop_warmer(app: web.Application) -> None:
    task = app.get("warmer")
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    _hcache_save()


# --- keeping the pictures -------------------------------------------------
# A photo has to be fetched while the vacuum still has it, and each one is an
# MQTT blob, so they are pulled a few at a time on their own task rather than
# inside the sampler tick: a run that finds twelve obstacles must not turn one
# tick into twelve round trips with the lock held.
PHOTO_SWEEP = 6.0        # s between passes while there is anything outstanding
PHOTO_IDLE = 60.0        # s between passes when there is not
PHOTO_BATCH = 3          # per pass, so the live poll always gets a turn


async def _archive_photos() -> int:
    """Fetch and store any archived detection whose picture we do not have yet."""
    pending = ARCHIVE.pending_photos(PHOTO_BATCH)
    if not pending:
        return 0
    saved = 0
    for run_key, photo_id in pending:
        try:
            async with STATE._lock:
                raw, _used, expired = await _fetch_photo(await STATE.props(), photo_id)
        except Exception as exc:  # noqa: BLE001 -- one bad photo is not a failure
            _LOGGER.debug("photo %s fetch failed: %s", photo_id, exc)
            continue
        image = _photo_image(raw) if raw else None
        if image:
            ext = "png" if image.startswith(b"\x89PNG") else "jpg"
            frame, box = _photo_geometry(raw)
            if ARCHIVE.save_photo(float(run_key), photo_id, image, ext, frame, box):
                saved += 1
        elif expired:
            # The vacuum has recycled it. Nothing will bring it back, so stop
            # asking -- otherwise every sweep from now on spends a timeout on it.
            ARCHIVE.mark_photo_gone(run_key, photo_id)
            _LOGGER.info("photo %s is gone from the vacuum; recorded as such", photo_id)
        await asyncio.sleep(0.4)
    if saved:
        _LOGGER.info("obstacle archive: stored %d photo(s) (%s)", saved, ARCHIVE.stats())
    return saved


async def _photo_keeper(_app: web.Application) -> None:
    await asyncio.sleep(12)
    while True:
        busy = 0
        try:
            if STATE.devices:
                busy = await _archive_photos()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 -- must never take the app down
            _LOGGER.debug("photo keeper pass failed: %s", exc)
        # Outstanding work means come straight back; an empty queue means this can
        # go quiet, because new work only appears when a job runs.
        await asyncio.sleep(PHOTO_SWEEP if busy else PHOTO_IDLE)


async def _start_photo_keeper(app: web.Application) -> None:
    app["photos"] = asyncio.create_task(_photo_keeper(app))


async def _stop_photo_keeper(app: web.Application) -> None:
    task = app.get("photos")
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


async def _start_pusher(app: web.Application) -> None:
    app["pusher"] = asyncio.create_task(_pusher(app))


async def _stop_pusher(app: web.Application) -> None:
    task = app.get("pusher")
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


async def _start_sampler(app: web.Application) -> None:
    app["sampler"] = asyncio.create_task(_sampler(app))
    _care_cfg_load()
    app["mopcare"] = asyncio.create_task(_mop_care_watch(app))
    _queue_load()
    app["queue"] = asyncio.create_task(_queue_watch(app))


async def _stop_sampler(app: web.Application) -> None:
    task = app.get("sampler")
    if task:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


# --- AI obstacles ---------------------------------------------------------
# The map parser exposes four buckets (active/ignored x with/without photo).
# `photo_name` is the img_id that the vacuum's get_photo takes.
#
# Type codes ARE decoded now: jobs.OBSTACLE_INFO was read out of the device
# plugin's own tables (see tools/extract_obstacle_names.py), so each detection
# carries the app's name and explanation under "info". The raw code is still
# surfaced, and an unknown one says so rather than borrowing a neighbour's name.
_OBSTACLE_IGNORE_RADIUS = 48  # matches what the app sends for "avoidance not needed"

# python-roborock 4.20's map parser drops obstacle type/description/confidence
# and photo_name; 6.x extracts them from the same bytes. Rather than pin this
# app to 6.x, enrich from the 6.x sidecar when it is up, matching on (x, y),
# and degrade to geometry-only when it is not.
NEXT_URL = os.environ.get("RR_NEXT_URL", "").rstrip("/")


async def _enrich_obstacles(obstacles: list[dict]) -> list[dict]:
    if not obstacles or not NEXT_URL:
        return obstacles
    try:
        import aiohttp

        timeout = aiohttp.ClientTimeout(total=8)
        async with aiohttp.ClientSession(timeout=timeout) as sess:
            async with sess.get(f"{NEXT_URL}/api/obstacles") as resp:
                if resp.status != 200:
                    return obstacles
                detail = (await resp.json()).get("obstacles") or []
    except Exception:  # noqa: BLE001
        _LOGGER.debug("obstacle detail sidecar unavailable at %s", NEXT_URL, exc_info=True)
        return obstacles

    by_xy = {(_int_or_none(d.get("x")), _int_or_none(d.get("y"))): d for d in detail}
    for ob in obstacles:
        d = by_xy.get((ob["x"], ob["y"]))
        if not d:
            continue
        for key in ("type", "description", "confidence", "photo_id"):
            if d.get(key) is not None:
                ob[key] = d[key]
        # The type may have just changed, so the label has to be redone.
        ob["info"] = jobs_mod.describe_obstacle(ob.get("type"), ob.get("description"))
    return obstacles


def _obstacle_list(map_data: typing.Any) -> list[dict]:
    proj = _map_projector(map_data)
    out: list[dict] = []
    buckets = (
        ("obstacles", False, False),
        ("obstacles_with_photo", False, True),
        ("ignored_obstacles", True, False),
        ("ignored_obstacles_with_photo", True, True),
    )
    for attr, ignored, with_photo in buckets:
        for ob in (getattr(map_data, attr, None) or []):
            x, y = getattr(ob, "x", None), getattr(ob, "y", None)
            if x is None or y is None:
                continue
            # 6.x nests the AI classification under .details; 4.20 had it flat.
            det = getattr(ob, "details", None) or ob
            item = {
                "x": int(x), "y": int(y),
                "ignored": ignored,
                "has_photo": with_photo,
                "type": _int_or_none(getattr(det, "type", None)),
                "description": getattr(det, "description", None),
                "confidence": getattr(det, "confidence_level", None),
                "photo_id": getattr(det, "photo_name", None),
                "pct": None,
            }
            # Name and explanation for the bare type number, in the app's own
            # words (jobs.OBSTACLE_INFO, read out of the device plugin).
            item["info"] = jobs_mod.describe_obstacle(item["type"], item["description"])
            if proj:
                to_pct, _w, _h = proj
                try:
                    left, top = to_pct(float(x), float(y))
                    item["pct"] = [round(left, 3), round(top, 3)]
                except Exception:  # noqa: BLE001
                    pass
            out.append(item)
    return out


async def handle_obstacles(request: web.Request) -> web.Response:
    fresh = request.query.get("fresh") == "1"
    async with STATE._lock:
        props = await STATE.props()
        # A freshly started process has no map yet, so refresh when it is
        # missing -- not only when the caller asks for fresh data.
        if fresh or props.map_content.map_data is None:
            await props.map_content.refresh()
        obstacles = _obstacle_list(props.map_content.map_data)
    return web.json_response({
        "obstacles": obstacles,
        "active": sum(1 for o in obstacles if not o["ignored"]),
        "ignored": sum(1 for o in obstacles if o["ignored"]),
    })


# A past run's detections, from its own saved map. Fetching one is an MQTT round
# trip for ~400 KB and parsing it costs ~150ms, while the list distilled from it
# is a few hundred bytes -- which is why the window can be as wide as the history
# the panel shows. (At 12 the photo-count chips on a 20-run list evicted the
# entries the rows above were still using, and every scroll back up re-fetched a
# blob.)
_RECORD_MAP_CACHE: dict[int, list[dict]] = {}
_RECORD_MAP_KEEP = 60
# The vacuum writes a run's map when the run ends, and "ends" is not instantaneous.
# Records and event logs can be checked for finality directly (a non-zero `end`);
# a map cannot, so a run that finished in the last minute is read but NOT kept --
# otherwise one unlucky fetch during the write freezes a short list into the cache,
# and now that the cache is on disk it would stay wrong across restarts too.


async def _run_obstacle_list(props: typing.Any, begin: int, cache: bool = True) -> list[dict] | None:
    """A past run's detections, cached. None if the map could not be had.

    Caller holds STATE._lock. The ~400 KB blob is thrown away; only the distilled
    list is kept.
    """
    hit = _RECORD_MAP_CACHE.get(begin)
    if hit is not None:
        return hit
    try:
        raw = await props.map_content.rpc_channel.send_command(
            RoborockCommand.GET_CLEAN_RECORD_MAP, params={"start_time": begin})
        if not isinstance(raw, (bytes, bytearray)):
            _LOGGER.info("record map %s was not a map blob", begin)
            return None
        content = props.map_content.converter.parse_map_content(bytes(raw))
    except Exception as exc:  # noqa: BLE001
        _LOGGER.info("record map %s unavailable: %s", begin, exc.__class__.__name__)
        return None
    obstacles = _obstacle_list(content.map_data)
    # The same parse carries the run's path. Pinning it here means the map shows
    # the last clean's trace rather than whatever the live map has left over.
    if begin >= int(_LAST_TRACE["begin"] or 0):
        _pin_trace(begin, content.map_data, bytes(raw))
    rec = _CLEAN_RECORD_CACHE.get(begin) or {}
    # Logged with the blob size and the record's own count, because "the run map
    # has fewer detections than the live map did" is otherwise unfalsifiable: this
    # says whether the firmware pruned them, or whether we were handed a short blob
    # (which is a thing that happens -- see the photo-frame reassembly) and parsed
    # it happily.
    _LOGGER.info("record map %s: %d bytes -> %d detection(s) (%d with photo, %d ignored); "
                 "record says avoid_count=%s", begin, len(raw), len(obstacles),
                 sum(1 for o in obstacles if o.get("photo_id")),
                 sum(1 for o in obstacles if o["ignored"]), rec.get("avoid_count"))
    settled = _record_settled(rec)
    if cache and settled:
        _RECORD_MAP_CACHE[begin] = obstacles
        for stale in list(_RECORD_MAP_CACHE)[:-_RECORD_MAP_KEEP]:
            del _RECORD_MAP_CACHE[stale]
        _hcache_touch()
    elif cache:
        _LOGGER.info("record map %s not kept: the run has not settled yet", begin)
    return obstacles


# --------------------------------------------------------------------------- #
# Cleaning presets
# --------------------------------------------------------------------------- #
async def handle_presets(_request: web.Request) -> web.Response:
    """Saved jobs for the selected device, newest first."""
    return web.json_response({"presets": PRESETS.list(STATE.selected_duid)})


async def handle_preset_save(request: web.Request) -> web.Response:
    """Create, re-record or rename a preset.

    The panel sends the request body it would have posted to start the job, so a
    preset is a replay rather than a second way of expressing the same thing.
    """
    body = await request.json()
    try:
        record = PRESETS.save({**body, "duid": STATE.selected_duid})
    except ValueError as exc:
        raise web.HTTPBadRequest(reason=str(exc)) from None
    _LOGGER.info("preset saved: %s (%s -> %s)", record["name"], record["kind"], record["endpoint"])
    return web.json_response({"ok": True, "preset": record})


async def handle_preset_delete(request: web.Request) -> web.Response:
    body = await request.json()
    preset_id = str(body.get("id") or "")
    if not PRESETS.delete(preset_id):
        raise web.HTTPNotFound(reason="no such preset")
    return web.json_response({"ok": True})


async def handle_preset_arrange(request: web.Request) -> web.Response:
    """The whole preset layout in one call: order plus which row each sits on.

    Body: {items:[{id, group}]} in display order. `group` is any short string;
    presets sharing one are drawn side by side. Sent whole because a single drag
    changes both the preset that moved and the row it joined.
    """
    body = await request.json()
    items = body.get("items")
    if not isinstance(items, list):
        raise web.HTTPBadRequest(reason="items:[{id,group}] required")
    presets = PRESETS.arrange(items)
    return web.json_response({"ok": True, "presets": [
        p for p in presets if p.get("duid") in (None, STATE.selected_duid)]})


async def handle_preset_used(request: web.Request) -> web.Response:
    """Bump the usage counters after the panel has replayed a preset."""
    body = await request.json()
    PRESETS.note_used(str(body.get("id") or ""))
    return web.json_response({"ok": True})


async def handle_run_obstacles(request: web.Request) -> web.Response:
    """The obstacles a PAST run recorded, from that run's own saved map.

    The live map only carries what is on the floor now -- one ignored cable here
    -- while the run of 2026-08-08 20:11 avoided a shoe, a cable and two other
    things. `get_clean_record_map {"start_time": <record begin>}` answers "ok" on
    the ordinary channel and pushes the run's map blob down the MAP channel, so
    sending it through map_content's own channel returns the bytes directly.

    Parsed here rather than through `map_content.refresh()` on purpose: writing
    a historical map into the trait would swap it under the live map view.

    A note on the photos: `photo_name` survives in the map for a few days and
    then the vacuum recycles the image itself. `001Fqeps7PF7RHCO` was fetched
    successfully on 2026-08-09 and by 2026-08-14 the same request just never
    returns. So a run keeps its detections indefinitely but its pictures do not,
    and /api/obstacle/photo says which case it is rather than serving a broken
    image.
    """
    try:
        begin = int(request.query["begin"])
    except (KeyError, ValueError):
        raise web.HTTPBadRequest(reason="begin=<record timestamp> required") from None

    if request.query.get("fresh"):
        _RECORD_MAP_CACHE.pop(begin, None)   # re-read this one run's map from the vacuum
    obstacles = _RECORD_MAP_CACHE.get(begin)
    if obstacles is None:
        async with STATE._lock:
            obstacles = await _run_obstacle_list(await STATE.props(), begin)

    # The union of what the vacuum still has and what we kept. The archive is what
    # was on screen DURING the run, so it is usually the larger set -- a detection
    # the saved map dropped is exactly the case this exists for -- and for a run
    # whose map the vacuum has forgotten entirely it is the only set. Positions
    # are the join, the same identity the archive keys on.
    kept = ARCHIVE.for_begin(begin)
    # Folded with the archive's own identity rule, not an exact-position key: the
    # vacuum re-estimates a position as it drives past, so one chair leg arrives as
    # a spread of coordinates and keying on them exactly produced a pin per
    # sighting. jobs.ObstacleArchive.same_obstacle is the single definition.
    merged: list[dict] = []
    for ob in list(obstacles or []) + kept:
        cur = next((c for c in merged if jobs_mod.ObstacleArchive.same_obstacle(c, ob)), None)
        if cur is None:
            merged.append(dict(ob))
            continue
        for field, val in ob.items():
            if val is not None or field not in cur:
                cur[field] = val
    if not merged:
        raise web.HTTPBadGateway(reason="record map unavailable and nothing archived")
    obstacles = merged
    return web.json_response({
        "begin": begin,
        "obstacles": obstacles,
        "active": sum(1 for o in obstacles if not o.get("ignored")),
        "ignored": sum(1 for o in obstacles if o.get("ignored")),
        "with_photo": sum(1 for o in obstacles if o.get("photo_id")),
        # What survives here rather than on the vacuum, so the panel can say so.
        "archived": sum(1 for o in obstacles if o.get("photo")),
        "photos_expired": sum(1 for o in obstacles if o.get("photo_expired")),
    })


async def handle_obstacle_ignore(request: web.Request) -> web.Response:
    """Mark a spot as 'avoidance not needed' -- the app's evitement non necessaire.

    Verified against a capture of the phone doing it:
        set_ignore_identify_area {"data": [[1, 0], [2, x, y, 48]]}
    """
    body = await request.json()
    x, y = _int_or_none(body.get("x")), _int_or_none(body.get("y"))
    if x is None or y is None:
        raise web.HTTPBadRequest(reason="x and y required")
    async with STATE._lock:
        props = await STATE.props()
        await props.command.send("set_ignore_identify_area", {"data": [[1, 0], [2, x, y, _OBSTACLE_IGNORE_RADIUS]]})
        _drop_map_ttl()      # the obstacle buckets it reports have just changed
        # Re-read rather than assume: the ignore only counts once the vacuum
        # has it, and it moves the obstacle into the ignored bucket.
        await props.map_content.refresh()
        obstacles = _obstacle_list(props.map_content.map_data)
    return web.json_response({"ok": True, "obstacles": obstacles})


async def handle_obstacle_unignore(request: web.Request) -> web.Response:
    """Undo 'avoidance not needed'.

    set_ignore_identify_area is a full REPLACE, the same shape as save_map's
    barriers: `data` is a header row plus one row per ignored spot, and sending the
    header alone clears the list. So an undo is "send back everything that should
    stay" -- which for a single mis-tap is nothing at all.

    `keep` may name spots to preserve as [[x, y], ...]; without it every ignored
    area goes. The caller is expected to have read the current set from the map
    (they come back in the `ignored` bucket) and decided, because the device offers
    no getter -- SET_IGNORE_IDENTIFY_AREA is the only command of its family in
    python-roborock, and the map is the only place the current list is visible.
    """
    body = await request.json() if request.can_read_body else {}
    keep = []
    for spot in (body.get("keep") or []):
        x, y = _int_or_none(spot[0] if isinstance(spot, (list, tuple)) else spot.get("x")), \
               _int_or_none(spot[1] if isinstance(spot, (list, tuple)) else spot.get("y"))
        if x is not None and y is not None:
            keep.append([2, x, y, _OBSTACLE_IGNORE_RADIUS])
    data = [[1, 0]] + keep
    _journal("obstacle_unignore", {"keep": keep})
    async with STATE._lock:
        props = await STATE.props()
        result = await props.command.send("set_ignore_identify_area", {"data": data})
        _drop_map_ttl()
        await props.map_content.refresh()
        obstacles = _obstacle_list(props.map_content.map_data)
    still = [o for o in obstacles if o.get("ignored")]
    _LOGGER.info("unignore: kept %d, %d ignored area(s) remain -> %s",
                 len(keep), len(still), result)
    return web.json_response({"ok": True, "result": result, "kept": len(keep),
                              "still_ignored": len(still), "obstacles": obstacles})


# get_photo takes a `type`, and it decides what you get back:
#   0 -- the whole camera frame (800x600 on the Saros 10R) plus, in a separate
#        block, the rectangle the AI drew around what it recognised;
#   1 -- only that rectangle, as a square thumbnail (447x447, 360x360, ... the
#        size follows the box), and no box, because the crop IS the box.
# python-roborock asks for 1, which is why the panel used to show a photo cropped
# so tight it was hard to tell what the robot had seen. We ask for 0 and draw the
# box over the frame instead, which is also what the app does.
_PHOTO_TYPE_FULL = 0
_PHOTO_TYPE_CROP = 1
# A frame that is coming arrives in about a second (0.5s, 0.6s and 1.9s for the
# three 800x600 frames measured, the last of them fragmented across two MQTT
# messages); one that is not coming does not arrive at 45s either.
# So the wait is generous rather than patient, and the crop is tried next -- a
# 45s spinner for the answer "only the crop exists" is the wrong trade.
# (The per-call timeout is a local modification to the fork's V1RpcChannel; see
# python-roborock/roborock/devices/rpc/v1_channel.py.)
_PHOTO_TIMEOUT = 12.0
# Block layout of the reply, read off three live photos (see NOTES.md):
#   1 -> 4 bytes, frame size as two LE uint16 (800, 600)
#   2 -> 8 bytes, box as four LE uint16 (x0, y0, x1, y1) in frame pixels
#   3 -> the JPEG
#   4 -> 4 bytes, purpose unknown; ignored
_PHOTO_BLOCK_FRAME = 1
_PHOTO_BLOCK_BOX = 2
_PHOTO_BLOCK_IMAGE = 3
_IMAGE_MAGIC = (b"\xff\xd8\xff", b"\x89PNG\r\n\x1a\n")


def _photo_blocks(raw: bytes) -> dict[int, bytes]:
    """Split a get_photo reply into its typed blocks.

    Same framing python-roborock's parse_photo_data walks, but keeping every
    block rather than only the image: the box lives in one of the others.
    """
    out: dict[int, bytes] = {}
    off = 0
    while off + 8 <= len(raw):
        block = int.from_bytes(raw[off:off + 2], "little")
        header = int.from_bytes(raw[off + 2:off + 4], "little")
        payload = int.from_bytes(raw[off + 4:off + 8], "little")
        end = off + header + payload
        if header < 8 or end > len(raw):
            break                      # truncated; keep whatever parsed cleanly
        out.setdefault(block, raw[off + header:end])
        off = end
    return out


def _u16le(buf: bytes, count: int) -> list[int] | None:
    if len(buf) < count * 2:
        return None
    return [int.from_bytes(buf[i * 2:i * 2 + 2], "little") for i in range(count)]


async def _fetch_photo(props: typing.Any, photo_id: str,
                       order: typing.Sequence[int] = (_PHOTO_TYPE_FULL, _PHOTO_TYPE_CROP)
                       ) -> tuple[bytes | None, int | None, bool]:
    """(payload, type used, expired). Caller holds STATE._lock.

    Split out of handle_obstacle_photo so the archiver fetches pictures exactly
    the way the panel does. `expired` distinguishes "the vacuum has recycled this
    image" -- which is silence, not an error -- from a real failure, and that
    difference is what stops the archiver asking again forever.
    """
    trait = getattr(props, "obstacle_photos", None)
    if trait is None:
        return None, None, False
    expired = False
    for ptype in order:
        try:
            reply = await trait.rpc_channel.send_command(
                RoborockCommand.GET_PHOTO, params={"img_id": photo_id, "type": ptype},
                timeout=_PHOTO_TIMEOUT)
        except Exception as exc:  # noqa: BLE001
            if "timed out" in str(exc).lower():
                expired = True
                continue
            raise
        if isinstance(reply, (bytes, bytearray)):
            return bytes(reply), ptype, expired
    return None, None, expired


def _archive_seen_photo(photo_id: str, image: bytes,
                        frame: tuple | None = None, box: tuple | None = None) -> None:
    """Store a photo against whichever archived run knows this id.

    Any run holding the id, not only ones still waiting for a picture: an entry
    stored before the box was being kept needs re-writing WITH the geometry, and
    this fetch is exactly the one that has it.
    """
    key = ARCHIVE.run_of_photo(photo_id)
    if key is None:
        return
    ext = "png" if image.startswith(b"\x89PNG") else "jpg"
    if ARCHIVE.save_photo(float(key), photo_id, image, ext, frame, box):
        _LOGGER.info("obstacle archive: kept the photo you just opened (%s, box %s)",
                     photo_id, "yes" if box else "none")


def _photo_geometry(raw: bytes) -> tuple[tuple | None, tuple | None]:
    """(frame, box) out of a photo payload, in frame pixels."""
    blocks = _photo_blocks(raw)
    frame = _u16le(blocks.get(_PHOTO_BLOCK_FRAME) or b"", 2)
    box = _u16le(blocks.get(_PHOTO_BLOCK_BOX) or b"", 4)
    return (tuple(frame) if frame else None), (tuple(box) if box else None)


def _photo_image(raw: bytes) -> bytes | None:
    """The image out of a photo payload, or None if it carries none."""
    body = _photo_blocks(raw).get(_PHOTO_BLOCK_IMAGE)
    return body if body and body.startswith(_IMAGE_MAGIC) else None


async def handle_obstacle_photo(request: web.Request) -> web.Response:
    """Obstacle photo by img_id, fetched from the vacuum itself.

    The whole frame by default, with the AI's box in the response headers:

        X-Photo-Type:  0 (whole frame) or 1 (crop -- no box to report)
        X-Photo-Frame: 800x600
        X-Photo-Box:   422,0,795,446    (x0,y0,x1,y1, frame pixels)

    `?type=1` still asks for the crop. Nothing leaves this machine: 6.x's
    ObstaclePhotoTrait does the RSA exchange with the vacuum directly, and the
    trait only exists once discover_features() has run, which props() does on
    connect.
    """
    photo_id = request.query.get("id")
    if not photo_id:
        raise web.HTTPBadRequest(reason="id required")
    # Ours first. The vacuum recycles these after a few days, so for anything but
    # the last day or two the archived copy is the ONLY copy -- and when both
    # exist, serving the file costs no MQTT round trip.
    if request.query.get("fresh") != "1":
        kept = ARCHIVE.find_photo(photo_id)
        # Only when the geometry was recorded with it. A copy stored before the box
        # was being kept goes to the vacuum instead, which answers with both and
        # backfills the archive on the way through -- so it self-heals once, rather
        # than serving a box-less picture for ever.
        if kept is not None and not kept[1].get("known"):
            _LOGGER.info("archived photo %s predates the stored box; re-reading it", photo_id)
            kept = None
        if kept is not None:
            path, geom = kept
            # The frame size and the AI's box, exactly as the vacuum path reports
            # them. They live in the payload rather than in the image, so they have
            # to be carried separately or the panel has nothing to draw the box
            # from -- which is what happened when this shortcut was added.
            headers = {"Cache-Control": "max-age=86400", "X-Photo-Source": "archive"}
            if geom.get("frame"):
                headers["X-Photo-Frame"] = "%dx%d" % tuple(geom["frame"])
            if geom.get("box"):
                headers["X-Photo-Box"] = ",".join(str(v) for v in geom["box"])
            return web.FileResponse(path, headers=headers)
    forced = request.query.get("type")
    try:
        order = [int(forced)] if forced is not None else [_PHOTO_TYPE_FULL, _PHOTO_TYPE_CROP]
    except ValueError:
        raise web.HTTPBadRequest(reason="type must be a number") from None

    async with STATE._lock:
        props = await STATE.props()
        trait = getattr(props, "obstacle_photos", None)
        if trait is None:
            raise web.HTTPNotImplemented(
                reason="this device exposes no obstacle-photo trait (camera feature off?)")
        raw: bytes | None = None
        used = None
        expired = False
        for ptype in order:
            try:
                reply = await trait.rpc_channel.send_command(
                    RoborockCommand.GET_PHOTO, params={"img_id": photo_id, "type": ptype},
                    timeout=_PHOTO_TIMEOUT)
            except Exception as exc:  # noqa: BLE001
                # An id the device does not recognise comes back as a prompt
                # error; an id whose image it has recycled is simply never
                # answered. The timeout is therefore "expired", not "broken", and
                # the difference is what the UI needs to word it honestly.
                if "timed out" in str(exc).lower():
                    expired = True
                    continue          # a frame it will not send may still crop
                raise web.HTTPBadGateway(reason=f"photo fetch failed: {exc.__class__.__name__}")
            if not isinstance(reply, (bytes, bytearray)):
                raise web.HTTPBadGateway(reason="photo payload was not bytes")
            raw, used = bytes(reply), ptype
            break

    if raw is None:
        if expired:
            raise web.HTTPNotFound(
                reason="the vacuum no longer keeps this photo (they are recycled after a few days)")
        raise web.HTTPBadGateway(reason="photo fetch returned nothing")

    blocks = _photo_blocks(raw)
    body = blocks.get(_PHOTO_BLOCK_IMAGE)
    if not body or not body.startswith(_IMAGE_MAGIC):
        raise web.HTTPBadGateway(reason="photo payload contained no image")
    frame, box = _photo_geometry(raw)
    # Just fetched it off the vacuum, so keep it -- the picture AND the box, or the
    # archived copy would come back without a rectangle to draw.
    _archive_seen_photo(photo_id, body, frame, box)

    headers = {"Cache-Control": "max-age=3600", "X-Photo-Type": str(used),
               "X-Photo-Source": "vacuum"}
    if frame:
        headers["X-Photo-Frame"] = "%dx%d" % (frame[0], frame[1])
    if box:
        headers["X-Photo-Box"] = ",".join(str(v) for v in box)
    return web.Response(body=body, content_type="image/jpeg", headers=headers)


def make_app() -> web.Application:
    # Room for a sound upload (see handle_sound_upload); everything else is tiny.
    app = web.Application(client_max_size=SOUND_MAX_BYTES + 64 * 1024)
    app.on_startup.append(_start_pusher)
    app.on_cleanup.append(_stop_pusher)
    app.on_startup.append(_start_photo_keeper)
    app.on_cleanup.append(_stop_photo_keeper)
    app.on_startup.append(_start_sampler)
    app.on_cleanup.append(_stop_sampler)
    app.on_startup.append(_start_warmer)
    app.on_cleanup.append(_stop_warmer)
    app.router.add_get("/", handle_index)
    app.router.add_get("/api/state", handle_state)
    app.router.add_get("/api/status", handle_status)
    app.router.add_get("/api/map.png", handle_map_png)
    app.router.add_get("/api/trace", handle_trace)
    app.router.add_get("/api/run_trace", handle_run_trace)
    app.router.add_get("/api/diag", handle_diag)
    app.router.add_get("/api/status_raw", handle_status_raw)
    app.router.add_get("/api/probe", handle_probe)
    app.router.add_get("/api/launches", handle_launch_journal)
    app.router.add_post("/api/command_raw", handle_raw_command)
    app.router.add_get("/api/features", handle_features)
    app.router.add_get("/api/raw_map", handle_raw_map)
    app.router.add_get("/api/map_debug", handle_map_debug)
    app.router.add_get("/api/settings", handle_settings)
    app.router.add_get("/api/dock", handle_dock)
    app.router.add_get("/api/maintenance", handle_maintenance)
    app.router.add_get("/api/records", handle_records)
    app.router.add_get("/api/ws", handle_ws)
    app.router.add_get("/api/history", handle_history)
    app.router.add_get("/api/timeline", handle_timeline)
    app.router.add_get("/api/run_detail", handle_run_detail)
    app.router.add_get("/api/routines", handle_routines)
    app.router.add_get("/api/rooms_raw", handle_rooms_raw)
    # Icons lifted from the Roborock plugin bundle: the mode/route glyphs the
    # settings card uses, and the obstacle artwork (obstacle_new_p<N>.png, where
    # N is the AI type code -- see jobs.OBSTACLE_INFO).
    _mode_icons = HERE / "static" / "modeicons"
    if _mode_icons.is_dir():
        app.router.add_static("/modeicons/", path=str(_mode_icons), name="modeicons")
    _icons = HERE / "static" / "obsticons"
    if _icons.is_dir():
        app.router.add_static("/obsticons/", path=str(_icons), name="obsticons")
    # The app's own room-type artwork (roomtag_bubble_tag_<N>.png, where N is the
    # tag the vacuum stores against each segment -- see _room_tags).
    _roomtags = HERE / "static" / "roomtags"
    if _roomtags.is_dir():
        app.router.add_static("/roomtags/", path=str(_roomtags), name="roomtags")
    _static = HERE / "static"
    if _static.is_dir():
        app.router.add_static("/static/", path=str(_static), name="static")
        # Browsers ask for /favicon.ico on their own, whatever the page links to,
        # and so do bookmark bars and OS shortcut pickers. Serving it from the root
        # is the only way to answer those.
        _ico = _static / "icon" / "favicon.ico"
        if _ico.is_file():
            app.router.add_get("/favicon.ico", lambda _r: web.FileResponse(
                _ico, headers={"Cache-Control": "public, max-age=86400"}))
    app.router.add_get("/api/obstacles", handle_obstacles)
    app.router.add_get("/api/run_obstacles", handle_run_obstacles)
    app.router.add_get("/api/presets", handle_presets)
    app.router.add_post("/api/presets", handle_preset_save)
    app.router.add_post("/api/presets/delete", handle_preset_delete)
    app.router.add_post("/api/presets/arrange", handle_preset_arrange)
    app.router.add_post("/api/presets/used", handle_preset_used)
    app.router.add_get("/api/obstacle/photo", handle_obstacle_photo)
    app.router.add_post("/api/obstacle/ignore", handle_obstacle_ignore)
    app.router.add_post("/api/obstacle/unignore", handle_obstacle_unignore)
    app.router.add_get("/api/runs", handle_runs)
    app.router.add_get("/api/errors", handle_errors)
    app.router.add_post("/api/resume", handle_resume)
    app.router.add_post("/api/resume/dismiss", handle_resume_dismiss)
    app.router.add_get("/api/queue", handle_queue)
    app.router.add_post("/api/queue", handle_queue)
    app.router.add_get("/api/mop_care", handle_mop_care)
    app.router.add_post("/api/mop_care", handle_mop_care)
    app.router.add_post("/api/select", handle_select)
    app.router.add_post("/api/command", handle_command)
    app.router.add_post("/api/clean", handle_clean)
    app.router.add_post("/api/zone", handle_zone)
    app.router.add_post("/api/goto", handle_goto)
    app.router.add_post("/api/barriers", handle_barriers_save)
    app.router.add_post("/api/thresholds", handle_thresholds_save)
    app.router.add_post("/api/modes", handle_modes)
    app.router.add_post("/api/dock", handle_dock_set)
    app.router.add_post("/api/dock/action", handle_dock_action)
    app.router.add_get("/api/camera/status", handle_camera_status)
    app.router.add_post("/api/camera/start", handle_camera_start)
    app.router.add_post("/api/camera/offer", handle_camera_offer)
    app.router.add_get("/api/camera/ice", handle_camera_ice)
    app.router.add_post("/api/camera/ice", handle_camera_ice)
    app.router.add_post("/api/camera/stop", handle_camera_stop)
    app.router.add_post("/api/camera/voice", handle_camera_voice)
    app.router.add_get("/api/sounds", handle_sounds)
    app.router.add_get("/api/choreos", handle_choreos)
    app.router.add_post("/api/choreos", handle_choreos)
    app.router.add_post("/api/sounds", handle_sound_upload)
    app.router.add_post("/api/sounds/edit", handle_sound_edit)
    app.router.add_get("/api/sounds/file", handle_sound_file)
    app.router.add_get("/api/patrol/map", handle_patrol_map)
    app.router.add_get("/api/patrol/routes", handle_patrol_routes)
    app.router.add_post("/api/patrol/routes", handle_patrol_routes)
    app.router.add_get("/api/patrol/session", handle_patrol_session)
    app.router.add_post("/api/patrol/session", handle_patrol_session)
    app.router.add_post("/api/rc", handle_rc)
    app.router.add_post("/api/setting", handle_setting)
    app.router.add_post("/api/dnd", handle_dnd)
    app.router.add_post("/api/map/select", handle_map_select)
    app.router.add_post("/api/map/reset", handle_map_reset)
    app.router.add_post("/api/room/rename", handle_room_rename)
    app.router.add_post("/api/room/merge", handle_room_merge)
    app.router.add_post("/api/room/split", handle_room_split)
    app.router.add_post("/api/room/material", handle_room_material)
    app.router.add_get("/api/carpet", handle_carpet_get)
    app.router.add_post("/api/carpet_mode", handle_carpet_mode)
    app.router.add_post("/api/experimental", handle_experimental)
    app.router.add_post("/api/consumable/reset", handle_consumable_reset)
    app.router.add_post("/api/routine/run", handle_routine_run)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(_stop_patrol)
    app.on_shutdown.append(_on_shutdown)
    app.on_cleanup.append(on_cleanup)
    return app


async def _on_shutdown(_app: web.Application) -> None:
    """Runs before aiohttp waits for in-flight handlers, which is the point."""
    await HUB.close_all()


# How long to wait for anything still in flight once the sockets are closed. The
# default is 60s, which is six times Docker's grace period -- so any straggler at
# all meant a SIGKILL rather than a clean stop. Nothing here is worth more than a
# couple of seconds: requests are short, and everything durable is already written
# to disk as it happens.
SHUTDOWN_TIMEOUT = 2.0


if __name__ == "__main__":
    _LOGGER.info("Open http://%s:%d after login completes", HOST, PORT)
    web.run_app(make_app(), host=HOST, port=PORT, shutdown_timeout=SHUTDOWN_TIMEOUT)
