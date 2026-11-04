"""Error vocabulary and in-memory run tracking for the Roborock panel.

Deliberately stateless on disk. Two things the vacuum does not tell us:

  * How long it has been in a given error state. The robot reports a bare
    ``error_code`` with no timestamp, so "main brush jammed" looks identical
    whether it happened 4 seconds or 4 hours ago. We timestamp the first
    sighting of each episode. That only has to survive between polls, not
    across restarts -- so it lives in memory, and a restart simply restarts
    the clock.

  * What a given error actually means, and whether it needs a human. The
    library's enum is names only, and it silently mislabels codes it does not
    know (see describe_error).

Which rooms still need cleaning is NOT tracked here: it is reconstructed from
the map's own path trace in app._coverage, which works for runs that started
before the server did. The only run state kept below is the room list of a
segment clean we launched ourselves, used to avoid offering to "resume" rooms
you deliberately never asked for.
"""

from __future__ import annotations

import json
import logging
import pathlib
import threading
import typing
import time

_LOGGER = logging.getLogger("rr-demo.jobs")


# --------------------------------------------------------------------------- #
# Error vocabulary
# --------------------------------------------------------------------------- #
# Advice is keyed by the python-roborock enum *name*, not the number, so that a
# library update that renumbers or adds codes degrades to the generic text
# instead of showing confidently wrong instructions.
#
# severity:
#   blocking -- the robot has stopped and cannot continue without a human
#   attention -- it may keep going, but something needs looking at
#   info     -- transient / self-clearing
ERROR_INFO: dict[str, tuple[str, str, str]] = {
    # name: (severity, short title, what to do)
    "none": ("info", "No error", ""),
    "lidar_blocked": ("blocking", "LiDAR blocked", "Clear the spinning turret on top and wipe its window."),
    "bumper_stuck": ("blocking", "Bumper stuck", "Tap the front bumper a few times to free it, and clear any debris behind it."),
    "wheels_suspended": ("blocking", "Wheels suspended", "The robot thinks it is off the ground. Move it to a flat surface."),
    "cliff_sensor_error": ("blocking", "Cliff sensor dirty", "Wipe the drop sensors on the underside with a dry cloth."),
    "main_brush_jammed": (
        "blocking",
        "Main brush jammed",
        "Flip the robot, unclip the main brush cover and lift the brush out. Cut away hair or cord wrapped "
        "around it — check the bearing caps at both ends, which is where cord usually hides — then reseat it.",
    ),
    "side_brush_jammed": ("blocking", "Side brush jammed", "Remove the side brush and cut away anything wound around its post."),
    "wheels_jammed": ("blocking", "Wheels jammed", "Free both drive wheels and check for hair around the axles."),
    "robot_trapped": ("blocking", "Robot trapped", "It cannot find a way out. Move it to open floor and restart."),
    "no_dustbin": ("blocking", "Dustbin missing", "Refit the dustbin (and filter) until it clicks."),
    "strainer_error": ("attention", "Strainer problem", "Check and rinse the dock's strainer."),
    "low_battery": ("attention", "Battery low", "Let it charge; the run can be resumed afterwards."),
    "charging_error": ("attention", "Charging error", "Clean the dock contacts and the robot's charging pads."),
    "battery_error": ("blocking", "Battery error", "Power-cycle the robot; if it persists the battery needs service."),
    "wall_sensor_dirty": ("attention", "Wall sensor dirty", "Wipe the wall-follow sensor on the right-hand side."),
    "robot_tilted": ("blocking", "Robot tilted", "Move it onto level ground."),
    "fan_error": ("blocking", "Fan error", "Check the filter and airway for blockage, then restart."),
    "filter_blocked": ("attention", "Filter blocked", "Rinse and fully dry the filter (24h) before refitting."),
    "optical_flow_sensor_dirt": ("attention", "Optical flow sensor dirty", "Wipe the small lens on the underside."),
    "vertical_bumper_pressed": ("blocking", "Vertical bumper pressed", "Free the top bumper; check it is not wedged under furniture."),
    "dock_locator_error": ("attention", "Cannot find dock", "Clear the space around the dock and make sure it is powered."),
    "return_to_dock_fail": ("attention", "Return to dock failed", "Clear the path back to the dock."),
    "nogo_zone_detected": ("info", "Blocked by a no-go zone", "The robot started inside a no-go zone. Move it out."),
    "robot_on_carpet": ("info", "On carpet", "Move the robot off the carpet to start a mop job."),
    "vibrarise_jammed": ("blocking", "Mop lift jammed", "Check the mop-lift mechanism for obstruction."),
    "mopping_roller_1": ("blocking", "Mop roller jammed", "Remove the mop roller and clear hair or debris."),
    "mopping_roller_error_2": ("blocking", "Mop roller error", "Remove and refit the mop roller."),
    "cannot_cross_carpet": ("info", "Cannot cross carpet", "A carpet is blocking the route to the remaining area."),
    "invisible_wall_detected": ("info", "Blocked by a virtual wall", "A virtual wall is blocking the route."),
    "internal_error": ("attention", "Internal error", "Restart the robot. If it repeats, note the code and contact support."),
    # -- the rest of RoborockErrorCode, so no code the firmware can report falls
    # through to the generic text. Wording is derived from the library's own
    # enum name (the only authority we have for these) -- see _DERIVED below.
    "compass_error": ("attention", "Compass error", "The heading sensor is unhappy. Restart the robot on level ground away from magnets or speakers."),
    "side_brush_error": ("blocking", "Side brush error", "Remove the side brush, clear its post, and refit it until it clicks."),
    "dock": ("info", "Dock reports a problem", "The fault is on the dock rather than the robot — check the dock's own indicator and the dock error below."),
    "visual_sensor": ("attention", "Visual sensor dirty", "Wipe the forward-facing camera and its surround with a dry cloth."),
    "light_touch": ("info", "Light-touch sensor triggered", "The robot reports a light-touch event. Clear whatever is resting against it."),
    "collect_dust_error_3": ("attention", "Dust collection failed", "Check the dock's dust bag and duct, then run the empty cycle again."),
    "collect_dust_error_4": ("attention", "Dust collection failed", "Check the dock's dust bag and duct, then run the empty cycle again."),
    "clear_water_box_hoare": ("attention", "Clean-water tank problem", "Reseat the clean-water tank and check it is filled and its lid closed."),
    "dirty_water_box_hoare": ("attention", "Dirty-water tank problem", "Empty and reseat the dirty-water tank."),
    "sink_strainer_hoare": ("attention", "Wash-tray strainer problem", "Rinse the dock's wash tray and its strainer, then refit."),
    "clear_water_box_exception": ("attention", "Clean-water tank not detected", "Refit the clean-water tank until it seats."),
    "clear_brush_exception": ("attention", "Dock cleaning brush problem", "Clear the dock's cleaning brush and refit it."),
    "clear_brush_exception_2": ("attention", "Dock cleaning brush problem", "Remove the dock's cleaning brush, rinse it, and refit it."),
    "filter_screen_exception": ("attention", "Filter screen problem", "Rinse the filter screen and let it dry fully before refitting."),
    "mopping_roller_2": ("blocking", "Mop roller problem", "Remove the mop roller, clear hair and debris, and refit it."),
    "up_water_exception": ("attention", "Water intake failed", "The dock could not draw water. Check the clean-water tank and the intake."),
    "drain_water_exception": ("attention", "Draining failed", "The dock could not drain. Empty the dirty-water tank and clear the drain channel."),
    "temperature_protection": ("attention", "Temperature protection", "Something is too hot or too cold. Let the robot sit at room temperature before restarting."),
    "clean_carousel_exception": ("attention", "Wash carousel problem", "Clear the dock's wash carousel and check nothing blocks its rotation."),
    "clean_carousel_water_full": ("attention", "Wash carousel full of water", "Empty the dock's wash tray and check the drain."),
    "water_carriage_drop": ("blocking", "Mop carriage dropped", "Refit the mop carriage / mop plate until it latches."),
    "check_clean_carouse": ("attention", "Check the wash carousel", "Inspect the dock's wash carousel for debris."),
    "audio_error": ("info", "Audio error", "A sound file failed to play. Reinstall the voice pack from the official app if it persists."),
}

# Entries whose wording comes only from the library's enum name, with no observed
# behaviour or official text behind it. The UI says so, because "Wash carousel
# full of water" reads like a fact when it is really a translation of
# `clean_carousel_water_full`. The pair (5, 1) is the one case read straight off
# the phone's own error sheet -- see ERROR_PAIRS.
_DERIVED = frozenset({
    "compass_error", "side_brush_error", "dock", "visual_sensor", "light_touch",
    "collect_dust_error_3", "collect_dust_error_4", "clear_water_box_hoare",
    "dirty_water_box_hoare", "sink_strainer_hoare", "clear_water_box_exception",
    "clear_brush_exception", "clear_brush_exception_2", "filter_screen_exception",
    "mopping_roller_2", "up_water_exception", "drain_water_exception",
    "temperature_protection", "clean_carousel_exception", "clean_carousel_water_full",
    "water_carriage_drop", "check_clean_carouse", "audio_error",
})

# The dock reports separately from the robot: status carries `dock_error` next to
# `error_code`, and RoborockDockErrorCode is its vocabulary. Same rule -- every
# value the enum knows gets an entry so nothing falls through.
DOCK_ERROR_INFO: dict[str, tuple[str, str, str]] = {
    "ok": ("none", "Dock OK", ""),
    "no_dustbin_or_filter": ("blocking", "Dust bag or filter missing", "Refit the dock's dust bag and filter."),
    "auto_empty_dock_fan_error": ("blocking", "Dock fan error", "The dock's suction fan failed. Unplug it for a minute, then retry the empty cycle."),
    "duct_blockage": ("blocking", "Dock duct blocked", "Clear the dock's suction duct — this is usually a wad of hair at the bend."),
    "auto_empty_dock_voltage_error": ("blocking", "Dock power problem", "Check the dock's power supply and that it is plugged in firmly."),
    "water_empty": ("attention", "Clean-water tank empty", "Refill the dock's clean-water tank."),
    "waste_water_tank_full": ("attention", "Dirty-water tank full", "Empty the dock's dirty-water tank."),
    "maintenance_brush_jammed": ("blocking", "Dock brush jammed", "Free the dock's maintenance brush and clear hair from around it."),
    "dirty_tank_latch_open": ("attention", "Dirty-water tank not latched", "Push the dirty-water tank in until it latches."),
    "no_dustbin": ("blocking", "Dustbin missing", "Refit the dustbin."),
    "cleaning_tank_full_or_blocked": ("attention", "Wash tank full or blocked", "Empty the wash tray and clear its drain."),
}

_GENERIC = (
    "attention",
    "Unrecognised error",
    "This code is not in the local error table. Check the robot's own voice prompt or the official app for the "
    "exact wording, then clear whatever it reports.",
)


# Errors inside a clean record arrive as a (major, minor) pair -- `311` and
# `312` on an `i=31` timeline event -- and the official app displays the pair
# verbatim, e.g. "(5-1)Brosse principale bloquée." The major half is the
# RoborockErrorCode above; the minor half has no published meaning and is not in
# the plugin bundle, the APK or any captured cloud response (see NOTES.md), so
# it is only ever shown, never interpreted. Pairs confirmed against the phone's
# own error sheet get a sharper title here; everything else falls back to the
# major code's entry.
ERROR_PAIRS: dict[tuple[int, int], tuple[str, str]] = {
    # (major, minor): (title, advice) -- seen on the phone for the 08/01 run
    (5, 1): (
        "Main brush jammed",
        "Clean the main brush and its bearings, then restart. Flip the robot, unclip the brush cover, lift the "
        "brush out and cut away hair or cord — the bearing caps at both ends are where cord usually hides.",
    ),
}


def describe_error(code: int, minor: int | None = None) -> dict:
    """Human-facing description of an error code.

    `minor` is the second half of the pair a clean record carries. It never
    changes the severity -- only the wording, and only for pairs actually seen
    on the phone. `pair` is what the official app prints next to the message.

    Unknown codes are reported as unknown rather than silently mislabelled --
    that is the whole reason the vacuum's 'charger disconnected' state string
    was misleading in the first place.
    """
    if code == 0:
        return {"code": 0, "name": "none", "known": True, "severity": "none", "title": "No error", "advice": "",
                "minor": minor, "pair": None}
    try:
        from roborock.data.v1.v1_code_mappings import RoborockErrorCode

        member = RoborockErrorCode(code)
        # RoborockEnum._missing_ swallows unrecognised values and hands back
        # `none` (0) rather than raising, which would render a brand new error
        # code as "No error". Only trust the member if it round-trips.
        known = int(member.value) == code
        name = member.name if known else f"error_{code}"
    except Exception:  # noqa: BLE001 -- renumbered / absent code
        name = f"error_{code}"
        known = False
    severity, title, advice = ERROR_INFO.get(name, _GENERIC)
    if not known:
        title = f"Unrecognised error {code}"
    elif name not in ERROR_INFO:
        # Known to the library but with no advice text written yet: show the
        # real name, prettified, and say plainly that there is no guidance
        # rather than reusing the "unrecognised" wording.
        title = name.replace("_", " ").capitalize()
        advice = "No local guidance for this code yet — check the robot's voice prompt for the exact wording."
    pair_hit = ERROR_PAIRS.get((code, minor)) if minor is not None else None
    if pair_hit:
        title, advice = pair_hit
    return {
        "code": code,
        "name": name,
        "known": known,
        "severity": severity,
        "title": title,
        "advice": advice,
        "minor": minor,
        # What the official app shows in parentheses. Only formed when the
        # record actually carried a minor half.
        "pair": f"{code}-{minor}" if minor is not None else None,
        # True when the wording is a rendering of the enum name and nothing more,
        # so the UI can avoid presenting a guess as a diagnosis.
        "derived": name in _DERIVED and not pair_hit,
        # The pair we have actually seen the official app spell out.
        "pair_verified": pair_hit is not None,
    }


def describe_dock_error(code: int | None) -> dict:
    """Human-facing description of a dock error code.

    Same contract as describe_error: unknown codes say so instead of borrowing
    another entry's wording.
    """
    if not code:
        return {"code": 0, "name": "ok", "known": True, "severity": "none", "title": "Dock OK", "advice": ""}
    try:
        from roborock.data.v1.v1_code_mappings import RoborockDockErrorCode

        member = RoborockDockErrorCode(code)
        known = int(member.value) == code
        name = member.name if known else f"dock_error_{code}"
    except Exception:  # noqa: BLE001
        name, known = f"dock_error_{code}", False
    severity, title, advice = DOCK_ERROR_INFO.get(
        name, ("attention", f"Unrecognised dock error {code}", _GENERIC[2]))
    return {"code": code, "name": name, "known": known,
            "severity": severity, "title": title, "advice": advice}


# --------------------------------------------------------------------------- #
# Why a run ended
# --------------------------------------------------------------------------- #
# The record's `finish_reason` is the only field that says why a run stopped, and
# RoborockFinishReason names it in families -- manual_interrupt_5, finished_
# cleaning_4, cleanup_interrupted_2. The families are what carry the meaning, so
# the wording below is derived from the family rather than from a per-code table
# that would be mostly invented. Verified: 37 (manual_interrupt_5) is the phone's
# "Nettoyage interrompu par l'utilisateur", and 54/56/57 are "Nettoyage terminé".
_FINISH_FAMILIES: tuple[tuple[str, str, str], ...] = (
    # name prefix, outcome, title
    ("finished_cleaning", "completed", "Cleaning complete"),
    ("finished_clenaing", "completed", "Cleaning complete"),   # library's own typo
    ("patrol_success", "completed", "Patrol complete"),
    ("pet_patrol_success", "completed", "Pet patrol complete"),
    ("manual_interrupt", "stopped", "Cleaning stopped by you"),
    ("breakpoint", "interrupted", "Cleaning paused part-way"),
    ("cleanup_interrupted", "interrupted", "Cleaning interrupted"),
    ("locate_fail", "interrupted", "Robot lost its position"),
    ("area_unreachable", "interrupted", "Area could not be reached"),
    ("washing_error", "interrupted", "Mop wash failed"),
    ("back_to_wash_failure", "interrupted", "Could not return to wash the mop"),
    ("patrol_fail", "interrupted", "Patrol failed"),
    ("pet_patrol_fail", "interrupted", "Pet patrol failed"),
)


# --- obstacle types -------------------------------------------------------
# What the AI camera says it saw. The record map and the live map carry a bare
# `type` number and (sometimes) an English-ish `description`; the phone shows a
# translated name and a sentence about what the robot did. Both come from the
# device plugin bundle, read out of it directly rather than guessed --
# `tools/extract_obstacle_names.py` walks the Hermes bytecode for the
# plugin's own `obstacleNames` / `obstacleRemoveTips` tables (type -> i18n key)
# and the per-locale dictionaries those keys resolve against, and prints this
# block. See NOTES.md "Obstacle types" for the method.
#
# type -> (slug, French name, English name, French explanation)
# The wording is the app's own, including the places where it is odd: type 9 is
# the bathroom scale and the app labels it "Échelle" (a ladder) -- the sentence
# underneath says "pèse-personne", so both are kept as shipped rather than
# corrected here, where a difference from the phone would be a bug.
# Duplicates are the plugin's too: 3/4 are both the pedestal, 26/27 the
# crossbar furniture, 10/34 fabric, 51/66 fabric balls, 49/50 cat and dog under
# one "Animal", and 18/42 are the generic "Obstacle" with no explanation.
OBSTACLE_INFO: dict[int, tuple[str, str, str, str]] = {
    0  : ('xian', 'Câble', 'Wire',
         "Câble/fil évité pour empêcher tout risque d’enchevêtrement. Un risque"
         " d’enchevêtrement subsiste pour les longs câbles fins."),
    1  : ('baba', 'Excréments d’animaux domestiques', 'Pet Waste',
         "Excréments d’animaux évités pour empêcher tout risque de problème."),
    2  : ('shoe', 'Chaussures', 'Footwear',
         "Chaussures/pantoufles évitées pour empêcher tout risque de blocage."),
    3  : ('dizuo', 'Socle', 'Pedestal',
         "Pied évité pour empêcher tout risque de blocage."),
    4  : ('dizuo', 'Socle', 'Pedestal',
         "Pied évité pour empêcher tout risque de blocage."),
    5  : ('chapai', 'Prise multiple', 'Power Strip',
         "Multiprise évitée pour empêcher tout risque de blocage."),
    9  : ('tizhongcheng', 'Échelle', 'Scale',
         "Pèse-personne évité pour empêcher tout risque de blocage."),
    10 : ('zhiwu', 'Tissu', 'Fabric',
         "Tissu reconnu et évité pour empêcher tout risque d’enchevêtrement et de"
         " blocage."),
    18 : ('generic18', 'Obstacle', 'Obstacle',
         ""),
    25 : ('poqi', 'Pelle', 'Dustpan',
         "Ramasse-poussière ou objets similaires évités pour empêcher tout risque de"
         " blocage."),
    26 : ('henggan', 'Mobilier facilement coincé', 'Easily trapped furniture',
         "Mobilier muni d’une barre transversale, p. ex. chaise cantilever, évité pour"
         " empêcher tout risque de blocage."),
    27 : ('henggan', 'Mobilier facilement coincé', 'Easily trapped furniture',
         "Mobilier muni d’une barre transversale, p. ex. chaise cantilever, évité pour"
         " empêcher tout risque de blocage."),
    34 : ('zhiwu', 'Tissu', 'Fabric',
         "Tissu reconnu et évité pour empêcher tout risque d’enchevêtrement et de"
         " blocage."),
    42 : ('generic', 'Obstacle', 'Obstacle',
         ""),
    48 : ('loosewire', 'Câbles/fils', 'Cords/wires',
         "Câbles/fils évités pour empêcher tout risque d’enchevêtrement. Un risque"
         " d’enchevêtrement subsiste pour les longs câbles fins."),
    49 : ('cat', 'Animal', 'Pet',
         "Des animaux ont été reconnus pour empêcher toute collision mais aussi de les"
         " perturber."),
    50 : ('dog', 'Animal', 'Pet',
         "Des animaux ont été reconnus pour empêcher toute collision mais aussi de les"
         " perturber."),
    51 : ('curledfabric', 'Boules de tissu/papier', 'Fabric/paper balls',
         "Des boules de tissu/papier ont été reconnues pour empêcher les enchevêtrements"
         " et blocages du robot."),
    54 : ('floormirror', 'Miroir au sol', 'Stand Mirror',
         "Le robot reconnaît les miroirs au sol et tourne au préalable pour éviter les"
         " collisions."),
    65 : ('greenplant', 'Végétation', 'Greenery',
         "Le robot a détecté de la végétation et a tourné pour éviter de la heurter."),
    66 : ('curledfabric', 'Boules de tissu/papier', 'Fabric/paper balls',
         "Des boules de tissu/papier ont été reconnues pour empêcher les enchevêtrements"
         " et blocages du robot."),
    67 : ('thinhanger', 'Cintre', 'Hanger',
         "Le robot a détecté un cintre et a tourné pour éviter de s’y coincer."),
    69 : ('plasticbag', 'Sac plastique', 'Plastic bag',
         "Le robot a détecté un sac plastique et a tourné pour éviter de s’y coincer."),
    70 : ('paperbag', 'Cabas', 'Carryout bag',
         "Le robot a détecté un cabas et a tourné pour éviter de s’y coincer."),
    81 : ('card', 'Carte', 'Card',
         "La carte est évitée pour éviter qu'elle ne soit aspirée et ne provoque un"
         " blocage."),
}

# The plugin has artwork for these types only; everything else falls back to its
# p99 "unknown" glyph, so the panel does the same (static/obsticons/99.png).
OBSTACLE_ICON_TYPES: frozenset[int] = frozenset(
    {0, 1, 2, 3, 5, 9, 10, 18, 25, 26, 48, 49, 51, 54, 65, 67, 69, 70, 81, 99}
)


def describe_obstacle(code: int | None, description: str | None = None) -> dict:
    """What to show for one detection.

    `known` is False when the firmware reported a type this table has never
    seen: the caller then has only the raw number and whatever the device put in
    `description`, and should say so rather than inventing a name.
    """
    hit = OBSTACLE_INFO.get(code) if code is not None else None
    dev = (description or "").strip().replace("_", " ") or None
    if not hit:
        return {
            "type": code,
            "known": False,
            "name": dev.capitalize() if dev else ("Obstacle" if code is None else f"Obstacle (type {code})"),
            "name_en": dev,
            "slug": None,
            "desc": "",
            "device": dev,
        }
    slug, name_fr, name_en, desc = hit
    return {
        "type": code,
        "known": True,
        "name": name_fr,
        "name_en": name_en,
        "slug": slug,
        "desc": desc,
        # Kept alongside: the device sometimes disagrees with its own type
        # (a "shoes" description on a type the table calls fabric), and seeing
        # both is the only way to notice.
        "device": dev,
    }


def describe_finish(code: int | None, complete: int | None = None) -> dict:
    """Why a run ended, from its `finish_reason`.

    `complete` is the record's own flag and wins on the completed/not question --
    it is the field the phone uses to pick the green tick or the orange (!).
    The title comes from the enum family, and the raw code is always returned so
    the UI can show it next to the wording.
    """
    name = None
    if code is not None:
        try:
            from roborock.data.v1.v1_code_mappings import RoborockFinishReason

            member = RoborockFinishReason(code)
            if int(member.value) == code:
                name = member.name
        except Exception:  # noqa: BLE001 -- code absent from this library version
            name = None

    outcome, title = None, None
    for prefix, fam_outcome, fam_title in _FINISH_FAMILIES:
        if name and name.startswith(prefix):
            outcome, title = fam_outcome, fam_title
            break

    if outcome is None:
        # No family matched: fall back to the record's own flag and say plainly
        # that the reason code is not one we recognise.
        outcome = "completed" if complete else "interrupted"
        title = "Cleaning complete" if complete else "Cleaning incomplete"
    elif complete is not None:
        # The flag and the family disagreeing is worth not papering over: trust
        # the flag for the outcome, keep the family's wording.
        if complete and outcome != "completed":
            outcome = "completed"
        elif not complete and outcome == "completed":
            outcome = "interrupted"

    return {
        "code": code,
        "name": name or (f"finish_{code}" if code is not None else None),
        "known": name is not None,
        "outcome": outcome,
        "title": title,
        # A run the user stopped is not a failure, and the UI colours it differently.
        "by_user": bool(name and name.startswith("manual_interrupt")),
    }


def humanize_duration(seconds: float | None) -> str:
    if seconds is None:
        return ""
    s = int(max(0, seconds))
    if s < 60:
        return f"{s}s"
    m, s = divmod(s, 60)
    if m < 60:
        return f"{m}m {s:02d}s"
    h, m = divmod(m, 60)
    if h < 24:
        return f"{h}h {m:02d}m"
    d, h = divmod(h, 24)
    return f"{d}d {h}h"


# --------------------------------------------------------------------------- #
# In-memory run state
# --------------------------------------------------------------------------- #
class Tracker:
    """Per-device volatile state. No disk, no schema, no migrations.

    Everything here is derivable again from the next poll (error timing) or
    from the map (room coverage), so losing it on restart costs nothing but a
    reset stopwatch.
    """

    HISTORY = 20

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._open: dict[str, dict] = {}      # duid -> current error episode
        self._history: dict[str, list] = {}   # duid -> recent episodes, newest first
        self._launch: dict[str, dict] = {}    # duid -> the segment clean we last sent
        self._pending: dict[str, dict] = {}   # duid -> in-flight multi-step action

    # -- what we launched -------------------------------------------------- #
    def note_launch(self, duid: str, kind: str, segments: typing.Sequence[int] = (), params: dict | None = None) -> None:
        with self._lock:
            self._launch[duid] = {"kind": kind, "segments": [int(s) for s in segments], "params": params or {}, "at": time.time()}
        _LOGGER.info("launched %s%s", kind, f" segments={list(segments)}" if segments else "")

    def clear_launch(self, duid: str) -> None:
        with self._lock:
            self._launch.pop(duid, None)

    def last_launch(self, duid: str) -> dict | None:
        with self._lock:
            got = self._launch.get(duid)
            return dict(got) if got else None

    # -- error episodes ----------------------------------------------------- #
    def observe_error(self, duid: str, code: int) -> dict | None:
        """Feed one status sample in; returns the open episode, or None.

        An episode lasts while the code is unchanged. A different code closes
        the old one and starts a new clock, so the duration on screen always
        refers to the error on screen.
        """
        now = time.time()
        with self._lock:
            cur = self._open.get(duid)
            if cur is not None and (code == 0 or int(cur["code"]) != code):
                cur["cleared_at"] = now
                hist = self._history.setdefault(duid, [])
                hist.insert(0, cur)
                del hist[self.HISTORY:]
                self._open.pop(duid, None)
                _LOGGER.info("error %s cleared after %s", cur["name"], humanize_duration(now - cur["started_at"]))
                cur = None
            if code == 0:
                return None
            if cur is None:
                cur = {**describe_error(code), "started_at": now, "cleared_at": None}
                self._open[duid] = cur
                _LOGGER.warning("error %s (code %d) began", cur["name"], code)
            cur["last_seen_at"] = now
            return dict(cur)

    def recent_errors(self, duid: str) -> list[dict]:
        with self._lock:
            rows = ([self._open[duid]] if duid in self._open else []) + list(self._history.get(duid, ()))
            rows = [dict(r) for r in rows]
        now = time.time()
        for r in rows:
            end = r.get("cleared_at") or now
            r["seconds"] = end - r["started_at"]
            r["duration"] = humanize_duration(r["seconds"])
            r["ongoing"] = r.get("cleared_at") is None
        return rows

    # -- pending multi-step action ------------------------------------------ #
    # A resume that washes the pads first is two robot operations with minutes
    # of waiting in between. It runs as a background task so the UI keeps
    # polling; this is how that task reports where it has got to.
    def set_pending(self, duid: str, phase: str, detail: str = "") -> None:
        with self._lock:
            self._pending[duid] = {"phase": phase, "detail": detail, "since": time.time()}
        _LOGGER.info("pending: %s %s", phase, detail)

    def clear_pending(self, duid: str) -> None:
        with self._lock:
            self._pending.pop(duid, None)

    def pending(self, duid: str) -> dict | None:
        with self._lock:
            got = self._pending.get(duid)
            if not got:
                return None
            out = dict(got)
        out["seconds"] = time.time() - out["since"]
        out["duration"] = humanize_duration(out["seconds"])
        return out


# --------------------------------------------------------------------------- #
# Cleaning presets (small JSON file)
# --------------------------------------------------------------------------- #
# A preset is a named job you have already composed: which rooms (or zones) and
# the settings to clean them with. The device stores nothing of the sort -- per
# room settings travel WITH the job (see app_segment_clean_subdivision in
# NOTES.md) and are forgotten the moment it ends -- so keeping them is entirely
# on this side.
#
# What is stored is the request body the panel would have posted, plus the
# endpoint it goes to. Running a preset is then a replay of that request, with no
# second code path that could drift from the real one. `endpoint` is checked
# against a whitelist on the way in, so nothing in this file can aim a POST
# somewhere unintended later.
PRESET_ENDPOINTS = frozenset({"/api/clean", "/api/zone"})


class PresetStore:
    """Named cleaning jobs, kept in a plain JSON file.

    Same shape as RunLog below: a few dozen small records, rewritten whole,
    atomically, and a corrupt file starts empty rather than stopping the app.
    """

    KEEP = 60

    def __init__(self, path: str) -> None:
        self._path = pathlib.Path(path)
        self._lock = threading.Lock()
        self._items: list[dict] = []
        self._load()

    def _load(self) -> None:
        try:
            data = json.loads(self._path.read_text())
            self._items = list(data.get("presets") or [])
            _LOGGER.info("presets: %d loaded from %s", len(self._items), self._path)
        except FileNotFoundError:
            self._items = []
        except Exception as exc:  # noqa: BLE001
            _LOGGER.warning("presets unreadable (%s); starting empty", exc)
            self._items = []

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"presets": self._items[: self.KEEP]}, indent=1, ensure_ascii=False))
            tmp.replace(self._path)
        except Exception as exc:  # noqa: BLE001
            _LOGGER.warning("could not write presets: %s", exc)

    def list(self, duid: str | None = None) -> list[dict]:
        with self._lock:
            return [dict(p) for p in self._items if duid is None or p.get("duid") in (None, duid)]

    def get(self, preset_id: str) -> dict | None:
        with self._lock:
            for p in self._items:
                if p["id"] == preset_id:
                    return dict(p)
        return None

    def save(self, preset: dict) -> dict:
        """Insert or replace. Returns the stored record.

        An `id` that already exists is replaced in place, so renaming or
        re-recording a preset keeps its position in the list.
        """
        endpoint = str(preset.get("endpoint") or "")
        if endpoint not in PRESET_ENDPOINTS:
            raise ValueError(f"endpoint must be one of {sorted(PRESET_ENDPOINTS)}")
        if not isinstance(preset.get("body"), dict):
            raise ValueError("body must be an object")
        name = str(preset.get("name") or "").strip()
        if not name:
            raise ValueError("name is required")

        with self._lock:
            existing = next((p for p in self._items if p["id"] == preset.get("id")), None)
            record = {
                "id": existing["id"] if existing else f"p{int(time.time() * 1000):x}",
                "name": name[:60],
                "duid": preset.get("duid"),
                "kind": preset.get("kind"),
                "endpoint": endpoint,
                "body": preset["body"],
                "summary": preset.get("summary") or {},
                "created_at": existing.get("created_at") if existing else time.time(),
                "updated_at": time.time(),
                "used_at": existing.get("used_at") if existing else None,
                "used_count": existing.get("used_count", 0) if existing else 0,
                # Which row the panel draws this on: presets sharing a group id sit
                # side by side. Purely presentational, and carried through a
                # re-record so grouping a preset and then re-saving it does not
                # knock it back out of its row.
                "group": preset.get("group") if "group" in preset
                         else (existing.get("group") if existing else None),
            }
            if existing:
                self._items[self._items.index(existing)] = record
            else:
                self._items.insert(0, record)
                del self._items[self.KEEP:]
            self._save()
            return dict(record)

    def arrange(self, items: typing.Sequence[dict]) -> list[dict]:
        """Apply a whole layout at once: order plus group membership.

        `items` is [{id, group}] in display order. Sent as one call rather than
        one per preset because a drag changes both -- the preset that moved AND
        the row it joined -- and two requests could interleave into a layout
        neither end asked for.

        Ids not mentioned keep their group and are appended in their existing
        order, so a layout computed against a stale list cannot delete anything.
        A group left with a single member is dissolved: a row of one is just a
        row, and leaving the id behind would make the next drop join a group that
        looks empty.
        """
        with self._lock:
            by_id = {p["id"]: p for p in self._items}
            ordered: list[dict] = []
            for item in items:
                rec = by_id.pop(str(item.get("id") or ""), None)
                if rec is None:
                    continue
                group = item.get("group")
                rec["group"] = str(group)[:40] if group else None
                ordered.append(rec)
            ordered.extend(by_id[p["id"]] for p in self._items if p["id"] in by_id)
            counts: dict[str, int] = {}
            for rec in ordered:
                if rec.get("group"):
                    counts[rec["group"]] = counts.get(rec["group"], 0) + 1
            for rec in ordered:
                if rec.get("group") and counts[rec["group"]] < 2:
                    rec["group"] = None
            self._items = ordered
            self._save()
            return [dict(p) for p in self._items]

    def delete(self, preset_id: str) -> bool:
        with self._lock:
            before = len(self._items)
            self._items = [p for p in self._items if p["id"] != preset_id]
            if len(self._items) != before:
                # A group that just lost its second member is no longer a row.
                counts: dict[str, int] = {}
                for p in self._items:
                    if p.get("group"):
                        counts[p["group"]] = counts.get(p["group"], 0) + 1
                for p in self._items:
                    if p.get("group") and counts[p["group"]] < 2:
                        p["group"] = None
                self._save()
                return True
        return False

    def note_used(self, preset_id: str) -> None:
        with self._lock:
            for p in self._items:
                if p["id"] == preset_id:
                    p["used_at"] = time.time()
                    p["used_count"] = int(p.get("used_count") or 0) + 1
                    self._save()
                    return


# --------------------------------------------------------------------------- #
# Run log (small JSON file)
# --------------------------------------------------------------------------- #
# The device cannot tell us what settings a past run used -- its clean records
# carry no fan / water / mop fields. The only way to know is to watch while the
# run happens. The sampler polls over MQTT on a fixed cadence whether or not a
# browser is attached, and every sample taken during an active run is folded in
# here, so an interrupted run can later be resumed with the settings it was
# actually using.
#
# Deliberately a plain JSON file, not a database: a few dozen small records,
# rewritten whole, atomically.
class RunLog:
    KEEP = 60

    def __init__(self, path: str) -> None:
        self._path = pathlib.Path(path)
        self._lock = threading.Lock()
        self._runs: list[dict] = []
        self._load()

    # -- persistence -------------------------------------------------------- #
    def _load(self) -> None:
        try:
            data = json.loads(self._path.read_text())
            self._runs = list(data.get("runs") or [])
            _LOGGER.info("run log: %d run(s) from %s", len(self._runs), self._path)
        except FileNotFoundError:
            self._runs = []
        except Exception as exc:  # noqa: BLE001 -- a corrupt log must not stop the app
            _LOGGER.warning("run log unreadable (%s); starting empty", exc)
            self._runs = []

    def _save(self) -> None:
        """Write via temp file + rename so a crash cannot leave a half file."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"runs": self._runs[-self.KEEP:]}, indent=1, ensure_ascii=False))
            tmp.replace(self._path)
        except Exception as exc:  # noqa: BLE001
            _LOGGER.warning("could not write run log: %s", exc)

    # -- queries ------------------------------------------------------------ #
    def open_run(self, duid: str) -> dict | None:
        with self._lock:
            for r in reversed(self._runs):
                if r["duid"] == duid and r.get("ended_at") is None:
                    return dict(r)
        return None

    def last_run(self, duid: str) -> dict | None:
        """Most recent run for this device, open or finished."""
        with self._lock:
            for r in reversed(self._runs):
                if r["duid"] == duid:
                    return dict(r)
        return None

    def recent(self, duid: str, limit: int = 20) -> list[dict]:
        with self._lock:
            return [dict(r) for r in reversed(self._runs) if r["duid"] == duid][:limit]

    # -- ingest ------------------------------------------------------------- #
    def note_request(self, duid: str, kind: str, rooms: typing.Sequence[int], params: dict,
                     per_room: typing.Sequence[dict] = (),
                     zones: typing.Sequence[dict] = ()) -> None:
        """Record what WE asked for, before the robot has started moving.

        This is the only record of a job's intent that survives. The vacuum
        keeps none: get_customize_clean_mode comes back empty even after a
        configured run, and the clean record has no mode fields at all. Its
        event log has settings, but only for rooms it actually entered -- so
        for the rooms an interrupted run never reached, this is it.
        """
        with self._lock:
            self._runs.append({
                "duid": duid, "kind": kind, "requested_rooms": [int(r) for r in rooms],
                "requested": {k: params.get(k) for k in ("fan", "water", "mop", "repeat")},
                # One entry per room when the job carried per-room settings.
                # Empty for a job that cleans everything the same way.
                "requested_per_room": [
                    {"id": int(p["id"]), **{k: p.get(k) for k in ("fan", "water", "mop", "repeat")}}
                    for p in (per_room or []) if p.get("id") is not None
                ],
                # A zone job's names, in launch order. The robot keeps NONE of
                # this: every zone of a zone clean reports as the one pseudo
                # segment 41, so the event log can only ever say "Zone". What we
                # called them at launch is the only record there will be, which
                # is why it is written here rather than inferred later.
                # name, rect AND the settings. Keeping only the first two threw
                # away the very thing the timeline's launch note needs: the caller
                # resolves each zone's effective suction, water, route and passes
                # before handing them over, and rebuilding the dict here dropped
                # them on the floor -- so a job launched with Max/Standard x2 on one
                # zone was recorded as having no per-zone settings at all, and the
                # note then guessed the job-wide ones and disagreed with the robot.
                "requested_zones": [
                    {"name": str(z.get("name") or "").strip(),
                     "rect": list(z.get("rect") or []),
                     **{k: z.get(k) for k in ("fan", "water", "mop", "repeat", "seg")
                        if z.get(k) is not None}}
                    for z in (zones or [])
                ],
                "started_at": time.time(), "ended_at": None, "source": "requested",
                "modes": {}, "errors": [], "samples": 0,
            })
            self._save()
        _LOGGER.info("run log: opened %s run%s", kind,
                     f" rooms={list(rooms)}" if rooms else
                     (f" zones={[z.get('name') or '?' for z in zones]}" if zones else ""))

    # The device's clean record and this log share no id: the record's `begin`
    # is the robot's, and an entry here is written before the robot has said
    # anything at all. When they ran is the only join available -- and a sound
    # one, since two runs cannot start within the same couple of minutes.
    MATCH_SLACK_SEC = 300

    def for_run(self, duid: str, begin: float, slack: float | None = None) -> dict | None:
        """The logged request behind the clean record that began at `begin`."""
        if not begin:
            return None
        slack = self.MATCH_SLACK_SEC if slack is None else slack
        with self._lock:
            hits = [(abs(float(r.get("started_at") or 0) - float(begin)), r)
                    for r in self._runs
                    if r["duid"] == duid and abs(float(r.get("started_at") or 0) - float(begin)) <= slack]
        if not hits:
            return None
        return dict(min(hits, key=lambda kv: kv[0])[1])

    def dismiss_resume(self, duid: str, begin: float) -> bool:
        """Mark the run that began at `begin` as one nobody wants finishing.

        Written to the log so it survives a reload and reaches every client; the
        resume plan reads it back and offers nothing. Matched the same way
        `for_run` matches, because the record's `begin` and our `started_at` are
        two different clocks.
        """
        with self._lock:
            hits = [r for r in self._runs
                    if r["duid"] == duid
                    and abs(float(r.get("started_at") or 0) - begin) <= self.MATCH_SLACK_SEC]
            if not hits:
                # A run this panel never launched has no entry to flag, so one is
                # made for it -- otherwise a job started from the phone could never
                # be dismissed.
                self._runs.append({"duid": duid, "kind": "unknown", "requested_rooms": [],
                                   "requested": {}, "requested_per_room": [],
                                   "requested_zones": [], "started_at": begin,
                                   "ended_at": begin, "source": "dismissed",
                                   "modes": {}, "errors": [], "samples": 0,
                                   "resume_dismissed": True})
                self._save()
                return False
            for r in hits:
                r["resume_dismissed"] = True
            self._save()
            return True

    def resume_dismissed(self, duid: str, begin: float) -> bool:
        with self._lock:
            for r in self._runs:
                if r["duid"] != duid:
                    continue
                if abs(float(r.get("started_at") or 0) - begin) <= self.MATCH_SLACK_SEC:
                    if r.get("resume_dismissed"):
                        return True
        return False

    def observe(self, duid: str, sample: dict) -> dict | None:
        """Fold one poll into the run log; returns the open run, if any.

        `sample` needs: active (bool), fan, water, mop, repeat, error, plus the
        optional progress fields clean_time / clean_area / clean_percent.
        """
        now = time.time()
        with self._lock:
            run = None
            for r in reversed(self._runs):
                if r["duid"] == duid and r.get("ended_at") is None:
                    run = r
                    break

            if sample.get("active"):
                if run is None:
                    run = {
                        "duid": duid, "kind": "unknown", "requested_rooms": [], "requested": {},
                        "started_at": now, "ended_at": None, "source": "observed",
                        "modes": {}, "errors": [], "samples": 0,
                    }
                    self._runs.append(run)
                    _LOGGER.info("run log: run started (observed over MQTT)")
                # Settings as they are DURING the run -- the whole point of the
                # log. Last sample wins: if the user changes suction mid-run,
                # the later value is what the rest of the run used.
                for key in ("fan", "water", "mop", "repeat"):
                    if sample.get(key) is not None:
                        run["modes"][key] = sample[key]
                run["samples"] = int(run.get("samples", 0)) + 1
                for key in ("clean_time", "clean_area", "clean_percent"):
                    if sample.get(key) is not None:
                        run[key] = sample[key]
                err = int(sample.get("error") or 0)
                if err and err not in run["errors"]:
                    run["errors"].append(err)
                run["last_seen_at"] = now
                self._save()
                return dict(run)

            if run is not None:
                run["ended_at"] = now
                run["ended_complete"] = not sample.get("error") and not sample.get("in_cleaning")
                _LOGGER.info("run log: run closed after %s (%d samples)",
                             humanize_duration(now - run["started_at"]), run.get("samples", 0))
                self._save()
            return None

    def note_outcome(self, duid: str, missed: list[int], coverage: dict) -> None:
        """Attach the map-derived coverage verdict to the most recent run."""
        with self._lock:
            for r in reversed(self._runs):
                if r["duid"] == duid:
                    r["missed_rooms"] = [int(m) for m in missed]
                    r["coverage"] = coverage
                    self._save()
                    return


# --------------------------------------------------------------------------- #
# Obstacle archive
# --------------------------------------------------------------------------- #
# The vacuum's memory of what it drove around is lossy, in two separate ways.
#
# While a job is running the LIVE map shows a detection, photo and all. When the
# job finishes, the run's own saved map -- which is what the history reads back --
# keeps a subset: detections that were on screen mid-clean are simply not in it
# afterwards. And the photos have their own clock: `photo_name` survives in the
# map for a few days and then the vacuum recycles the image itself, so a picture
# not fetched is a picture lost.
#
# So neither is left to the vacuum. The sampler harvests the live map on every
# refresh while a job runs, the photos are pulled while the device still has them,
# and the run's own map is merged in once it has settled. Same obstacle seen ten
# times is one entry: identity is the position it was reported at, which is what
# the device itself keys them by.
#
# What is kept, and for how long, differs on purpose:
#   * the metadata -- position, type, label, confidence, when it was first and
#     last seen -- is a few hundred bytes a run and is kept for every run, always.
#   * the photos are the bulk (tens of KB each), so only the newest PHOTO_RUNS
#     runs keep theirs. Past that the file is deleted and the entry says so. An
#     obstacle from last month is a position and a label, and that is enough.
# How far apart two sightings of the same type can be and still be one object, in
# the map's own millimetres. A robot is ~350mm across, so anything inside this is
# the same thing as far as a pin on a floorplan is concerned. Two chair legs closer
# than this merge, which is the right trade: they are one piece of furniture, and
# the alternative was seven pins for one chair.
OBSTACLE_SAME_MM = 300


def _f(v: typing.Any) -> float | None:
    try:
        return None if v is None else float(v)
    except (TypeError, ValueError):
        return None


class ObstacleArchive:
    """Obstacles seen during runs, with photos for the most recent few.

    Metadata lives in one JSON file; photos are files in a sibling directory,
    named by the run and the device's own photo id. Not base64 inside the JSON:
    the metadata is read and rewritten on every harvest, and carrying megabytes
    of image through that on each pass would be slow and pointless when the
    filesystem already stores blobs perfectly well.
    """

    PHOTO_RUNS = 10          # how many runs back keep their pictures
    KEEP_RUNS = 400          # metadata is small, but not unbounded

    def __init__(self, path: str) -> None:
        self._path = pathlib.Path(path)
        self._photos = self._path.with_name(self._path.stem + "_photos")
        self._lock = threading.Lock()
        self._runs: dict[str, dict] = {}
        self._load()
        self.compact()

    # -- persistence -------------------------------------------------------- #
    def _load(self) -> None:
        try:
            data = json.loads(self._path.read_text())
            self._runs = dict(data.get("runs") or {})
            photos = sum(1 for r in self._runs.values()
                         for o in r.get("obstacles", []) if o.get("photo"))
            _LOGGER.info("obstacle archive: %d run(s), %d photo(s) from %s",
                         len(self._runs), photos, self._path)
        except FileNotFoundError:
            self._runs = {}
        except Exception as exc:  # noqa: BLE001 -- a corrupt archive must not stop the app
            _LOGGER.warning("obstacle archive unreadable (%s); starting empty", exc)
            self._runs = {}

    def _save(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(".tmp")
            tmp.write_text(json.dumps({"runs": self._runs}, ensure_ascii=False))
            tmp.replace(self._path)
        except Exception as exc:  # noqa: BLE001
            _LOGGER.warning("could not write obstacle archive: %s", exc)

    def compact(self) -> int:
        """Re-fold every run's list under the current identity rule.

        Runs on load, so entries written when identity was exact position -- one
        chair leg as seven pins -- collapse without anyone having to delete the
        file. A no-op once there is nothing left to merge.
        """
        merged_total = 0
        with self._lock:
            for run in self._runs.values():
                keep: list[dict] = []
                for ob in run.get("obstacles", []):
                    hit = self._find(keep, ob)
                    if hit is None:
                        keep.append(ob)
                        continue
                    merged_total += 1
                    # Prefer whichever sighting knows more: a photo, then the
                    # higher confidence.
                    if ob.get("photo") and not hit.get("photo"):
                        hit.update({k: ob.get(k) for k in
                                    ("photo", "photo_geom", "photo_frame", "photo_box")})
                    hc, oc = _f(hit.get("confidence")), _f(ob.get("confidence"))
                    if hc is None or (oc is not None and oc > hc):
                        for k in ("x", "y", "pct", "confidence"):
                            if ob.get(k) is not None:
                                hit[k] = ob[k]
                    hit["sightings"] = (int(hit.get("sightings") or 1)
                                        + int(ob.get("sightings") or 1))
                if len(keep) != len(run.get("obstacles", [])):
                    run["obstacles"] = keep
            if merged_total:
                self._save()
        if merged_total:
            _LOGGER.info("obstacle archive: folded %d duplicate sighting(s) into their "
                         "own obstacle", merged_total)
        return merged_total

    # -- identity ----------------------------------------------------------- #
    @staticmethod
    def same_obstacle(a: dict, b: dict) -> bool:
        """Are these two sightings the same thing on the floor?

        Position alone is not the identity, which is what an earlier version of
        this assumed. The vacuum RE-ESTIMATES an obstacle's position as it drives
        past: one chair leg came back seven times during a five-minute run at
        x=25153, 25160, 25247, 25293, 25294, 25295, 25308 -- same type, same photo
        id, same thing -- and an exact-position key made seven pins out of it.

        So: same type, and then either the device's own photo id agrees (the
        strongest signal there is, when both have one) or the two are close enough
        on the floor to be one object. Type has to match regardless -- a cable
        lying against a chair leg is two things worth telling apart.
        """
        if a.get("type") != b.get("type"):
            return False
        pa, pb = a.get("photo_id"), b.get("photo_id")
        if pa and pb and pa == pb:
            return True
        try:
            return (abs(int(a.get("x") or 0) - int(b.get("x") or 0)) <= OBSTACLE_SAME_MM
                    and abs(int(a.get("y") or 0) - int(b.get("y") or 0)) <= OBSTACLE_SAME_MM)
        except (TypeError, ValueError):
            return False

    @classmethod
    def _find(cls, index: list[dict], ob: dict) -> dict | None:
        for cur in index:
            if cls.same_obstacle(cur, ob):
                return cur
        return None

    def _newest_keys(self) -> list[str]:
        """Run keys, newest first. The key is the run's start time."""
        return sorted(self._runs, key=lambda k: float(self._runs[k].get("at") or k or 0),
                      reverse=True)

    # -- writing ------------------------------------------------------------ #
    def note(self, duid: str, run_key: float, obstacles: typing.Sequence[dict],
             source: str = "live") -> int:
        """Merge one sighting of a run's obstacles in. Returns how many are new.

        Fields already held are not overwritten with nothing: a later harvest that
        has lost the photo id must not erase the one an earlier harvest caught,
        which is the whole reason this exists.
        """
        key = str(int(run_key))
        added = 0
        with self._lock:
            run = self._runs.setdefault(key, {"at": float(run_key), "duid": duid,
                                              "begin": None, "obstacles": []})
            index = list(run["obstacles"])
            now = time.time()
            for ob in obstacles:
                if ob.get("x") is None or ob.get("y") is None:
                    continue
                cur = self._find(index, ob)
                if cur is None:
                    cur = {"first_seen": now, "sources": [], "photo": None,
                           "x": ob.get("x"), "y": ob.get("y")}
                    run["obstacles"].append(cur)
                    index.append(cur)
                    added += 1
                cur["sightings"] = int(cur.get("sightings") or 0) + 1
                for field in ("type", "description", "info", "has_photo", "photo_id"):
                    val = ob.get(field)
                    if val is not None:
                        cur[field] = val
                # The position comes from the BEST sighting, not the latest. The
                # vacuum's estimate wanders as it drives past, so taking the newest
                # made the pin crawl across the floor over a run; taking the one it
                # was most sure about keeps it still and keeps it honest.
                conf, best = _f(ob.get("confidence")), _f(cur.get("confidence"))
                if best is None or (conf is not None and conf > best):
                    for field in ("x", "y", "pct", "confidence"):
                        if ob.get(field) is not None:
                            cur[field] = ob[field]
                # `ignored` is the one field a later sighting may legitimately
                # flip either way: it is a decision, not an observation.
                if "ignored" in ob:
                    cur["ignored"] = bool(ob["ignored"])
                cur["last_seen"] = now
                if source not in cur["sources"]:
                    cur["sources"].append(source)
            for stale in self._newest_keys()[self.KEEP_RUNS:]:
                self._drop_photos(stale)
                self._runs.pop(stale, None)
            self._save()
        return added

    def set_begin(self, run_key: float, begin: int) -> None:
        """Tie a harvested run to the clean record it turned out to be."""
        key = str(int(run_key))
        with self._lock:
            run = self._runs.get(key)
            if run is not None and run.get("begin") != int(begin):
                run["begin"] = int(begin)
                self._save()

    def save_photo(self, run_key: float, photo_id: str, data: bytes, ext: str = "jpg",
                   frame: tuple[int, int] | None = None,
                   box: tuple[int, int, int, int] | None = None) -> bool:
        """Store one picture, with the geometry that goes with it.

        `frame` and `box` are the camera frame size and the rectangle the AI drew
        on it, both in frame pixels. They live in the payload the vacuum sends, not
        in the image, so storing the image alone loses the box -- and the box is
        half of what the picture is for.
        """
        key = str(int(run_key))
        safe = "".join(c for c in str(photo_id) if c.isalnum() or c in "-_")
        if not safe or not data:
            return False
        try:
            folder = self._photos / key
            folder.mkdir(parents=True, exist_ok=True)
            name = f"{safe}.{ext}"
            (folder / name).write_bytes(data)
        except Exception as exc:  # noqa: BLE001
            _LOGGER.warning("could not store obstacle photo %s: %s", photo_id, exc)
            return False
        with self._lock:
            run = self._runs.get(key)
            if run is None:
                return False
            for ob in run["obstacles"]:
                if str(ob.get("photo_id") or "") == str(photo_id):
                    ob["photo"] = name
                    # Set unconditionally, so its presence means "the geometry was
                    # looked for". A crop legitimately has no box, and without this
                    # flag that is indistinguishable from a copy stored before the
                    # geometry was being kept at all -- which must be re-fetched.
                    ob["photo_geom"] = True
                    ob["photo_frame"] = [int(frame[0]), int(frame[1])] if frame else None
                    ob["photo_box"] = [int(v) for v in box] if box else None
            self._save()
        return True

    def run_of_photo(self, photo_id: str) -> str | None:
        """Which archived run holds this photo id, stored or not."""
        with self._lock:
            for key in self._newest_keys():
                for ob in self._runs[key]["obstacles"]:
                    if str(ob.get("photo_id") or "") == str(photo_id):
                        return key
        return None

    # -- reading ------------------------------------------------------------ #
    def photo_file(self, run_key: str | float, photo_id: str) -> tuple[pathlib.Path, dict] | None:
        """(path, {frame, box}) for an archived picture, or None."""
        key = str(int(float(run_key)))
        with self._lock:
            run = self._runs.get(key)
            if run is None:
                return None
            for ob in run["obstacles"]:
                if str(ob.get("photo_id") or "") == str(photo_id) and ob.get("photo"):
                    path = self._photos / key / str(ob["photo"])
                    if not path.is_file():
                        return None
                    return path, {"frame": ob.get("photo_frame"), "box": ob.get("photo_box"),
                                  "known": bool(ob.get("photo_geom"))}
        return None

    def find_photo(self, photo_id: str) -> tuple[pathlib.Path, dict] | None:
        """Any archived copy of this photo, whichever run it came from."""
        for key in self._newest_keys():
            hit = self.photo_file(key, photo_id)
            if hit is not None:
                return hit
        return None

    def for_begin(self, begin: int, slack: float = 300.0) -> list[dict]:
        """A run's archived obstacles, found by its clean record's `begin`.

        Matched on `begin` where we have been told it, and otherwise on start
        time within `slack` -- the same join RunLog.for_run makes, and for the
        same reason: a harvest starts when the robot starts moving, which is a
        minute or two either side of the record's own timestamp.
        """
        with self._lock:
            for run in self._runs.values():
                if run.get("begin") == int(begin):
                    return [dict(o) for o in run["obstacles"]]
            # Only runs that do NOT already know their own begin. One that does
            # and disagrees belongs to a different clean record, and the slack is
            # wide enough to reach it: the pair on 2026-09-15 started 283s apart,
            # inside the 300s window, so a near-miss on the exact match would have
            # handed one run the other's detections.
            hits = [(abs(float(run.get("at") or 0) - float(begin)), run)
                    for run in self._runs.values()
                    if run.get("begin") in (None, int(begin))
                    and abs(float(run.get("at") or 0) - float(begin)) <= slack]
            if hits:
                return [dict(o) for o in min(hits, key=lambda kv: kv[0])[1]["obstacles"]]
        return []

    def pending_photos(self, limit: int = 8) -> list[tuple[str, str]]:
        """(run_key, photo_id) for pictures we know of but have not stored.

        Newest runs first: a photo the vacuum is about to recycle is the urgent
        one, and it is always the most recent.
        """
        out: list[tuple[str, str]] = []
        with self._lock:
            for key in self._newest_keys()[: self.PHOTO_RUNS]:
                for ob in self._runs[key]["obstacles"]:
                    pid = str(ob.get("photo_id") or "")
                    if pid and not ob.get("photo") and not ob.get("photo_gone"):
                        out.append((key, pid))
                        if len(out) >= limit:
                            return out
        return out

    def mark_photo_gone(self, run_key: str | float, photo_id: str) -> None:
        """The vacuum will not give this one up; stop asking for it."""
        key = str(int(float(run_key)))
        with self._lock:
            run = self._runs.get(key)
            if run is None:
                return
            for ob in run["obstacles"]:
                if str(ob.get("photo_id") or "") == str(photo_id):
                    ob["photo_gone"] = True
            self._save()

    # -- retention ---------------------------------------------------------- #
    def _drop_photos(self, key: str) -> int:
        """Delete one run's picture files. Caller holds the lock."""
        run = self._runs.get(key)
        gone = 0
        folder = self._photos / key
        for ob in (run or {}).get("obstacles", []):
            if ob.get("photo"):
                try:
                    (folder / str(ob["photo"])).unlink(missing_ok=True)
                except Exception:  # noqa: BLE001
                    pass
                ob["photo"] = None
                ob["photo_expired"] = True     # we had it; it is aged out, not lost
                gone += 1
        try:
            if folder.is_dir() and not any(folder.iterdir()):
                folder.rmdir()
        except Exception:  # noqa: BLE001
            pass
        return gone

    def prune(self) -> int:
        """Keep pictures for the newest PHOTO_RUNS runs only."""
        with self._lock:
            stale = self._newest_keys()[self.PHOTO_RUNS:]
            gone = sum(self._drop_photos(k) for k in stale)
            # Orphan folders, from a run whose metadata has since aged out.
            keep = set(self._runs)
            try:
                for folder in (self._photos.iterdir() if self._photos.is_dir() else ()):
                    if folder.is_dir() and folder.name not in keep:
                        for f in folder.iterdir():
                            f.unlink(missing_ok=True)
                        folder.rmdir()
            except Exception:  # noqa: BLE001
                pass
            if gone:
                self._save()
        if gone:
            _LOGGER.info("obstacle archive: aged out %d photo(s) beyond the newest %d runs",
                         gone, self.PHOTO_RUNS)
        return gone

    def stats(self) -> dict:
        with self._lock:
            obs = sum(len(r.get("obstacles", ())) for r in self._runs.values())
            photos = sum(1 for r in self._runs.values()
                         for o in r.get("obstacles", ()) if o.get("photo"))
            return {"runs": len(self._runs), "obstacles": obs, "photos": photos}
