"""Saved camera routes and a bounded, cancellable navigation session."""
from __future__ import annotations

import asyncio
import copy
import json
import math
from pathlib import Path
import secrets
import time


def station_approach(charger, projector, convert):
    """A point 80 cm in front of the dock, in Tap & Go's native coordinates."""
    angle = getattr(charger, "a", None)
    if charger is None or angle is None or not projector or not convert:
        return None
    if not all(math.isfinite(float(n)) for n in (charger.x, charger.y, angle)):
        return None
    # Project two points so map rotation, scale and the optional Y flip agree
    # with every other navigation command. This uses the dock's map heading,
    # not live robot heading estimation or calibration.
    a = math.radians(float(angle))
    to_pct, w, h = projector
    x0, y0 = to_pct(charger.x, charger.y)
    x1, y1 = to_pct(charger.x + math.cos(a)*1000, charger.y + math.sin(a)*1000)
    dock = convert(x0*w/100, y0*h/100)
    front = convert(x1*w/100, y1*h/100)
    dx, dy = front[0]-dock[0], front[1]-dock[1]
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance) or distance <= 0:
        return None
    target = [round(dock[0]+800*dx/distance), round(dock[1]+800*dy/distance)]
    return target if all(0 <= n <= 51200 for n in target) else None


def validate_steps(value):
    if not isinstance(value, list) or not 1 <= len(value) <= 40:
        raise ValueError("Ajoutez entre 1 et 40 étapes.")
    out = []
    for index, step in enumerate(value):
        if not isinstance(step, dict):
            raise ValueError("Étape invalide.")
        kind = step.get("kind")
        keys = {"goto": ("x", "y"), "turn": ("degrees",), "wait": ("seconds",), "home": ()}.get(kind)
        if keys is None:
            raise ValueError("Étape inconnue.")
        try:
            nums = [float(step[k]) for k in keys]
        except (KeyError, ValueError, TypeError):
            raise ValueError("Valeur d’étape invalide.") from None
        if not all(math.isfinite(n) for n in nums):
            raise ValueError("Valeur d’étape invalide.")
        if kind == "goto" and not all(0 <= n <= 51200 for n in nums):
            raise ValueError("Point hors de la carte.")
        if kind == "turn" and not 15 <= abs(nums[0]) <= 360:
            raise ValueError("Rotation : de 15 à 360 degrés.")
        if kind == "wait" and not 1 <= nums[0] <= 60:
            raise ValueError("Pause : de 1 à 60 secondes.")
        if kind == "home" and index != len(value)-1:
            raise ValueError("Le retour à la station doit être la dernière étape.")
        clean = {"kind": kind, **dict(zip(keys, nums))}
        if kind == "turn":
            speed = step.get("speed", 1)
            if isinstance(speed, bool) or not isinstance(speed, (int, float)) or speed not in (0.5, 1, 2):
                raise ValueError("Vitesse de rotation : lente, normale ou rapide.")
            clean["speed"] = speed
        out.append(clean)
    return out


class RouteStore:
    def __init__(self, path):
        self.path = Path(path)

    def list(self, duid):
        try:
            items = json.loads(self.path.read_text())
        except FileNotFoundError:
            items = []
        if not isinstance(items, list):
            raise ValueError("Fichier des parcours invalide.")
        return [p for p in items if p.get("duid") == duid]

    def update(self, duid, body, map_flag):
        items = json.loads(self.path.read_text()) if self.path.exists() else []
        rid = body.get("id")
        hit = next((p for p in items if p.get("id") == rid and p.get("duid") == duid), None)
        if rid and hit is None:
            raise ValueError("Parcours introuvable.")
        if body.get("delete"):
            if hit is None:
                raise ValueError("Parcours introuvable.")
            items.remove(hit)
        else:
            name = str(body.get("name") or "").strip()[:60]
            if not name:
                raise ValueError("Donnez un nom au parcours.")
            steps = validate_steps(body.get("steps"))
            if hit is None:
                if len(self.list(duid)) >= 60:
                    raise ValueError("Maximum de 60 parcours.")
                hit = {"id": secrets.token_hex(6), "duid": duid}
                items.append(hit)
            hit.update(name=name, map_flag=map_flag, steps=steps, updated_at=time.time())
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(items, ensure_ascii=False, indent=2))
        tmp.replace(self.path)
        return hit


class Patrol:
    """One session. Only its browser renews the lease; losing it stops motion."""
    LEASE = 15.0
    GOTO_TIMEOUT = 180.0
    HOME_TIMEOUT = 300.0
    UNLOCK_TIMEOUT = 8.0
    CLOSE_TIMEOUT = 15.0
    POLL = 0.8
    PAIR_DELAY = 2.0
    OMEGA = 0.6
    IDLE = {2, 3, 8, 100}
    GOTO = 16
    MANUAL = {4, 7}

    def __init__(self, send, snapshot):
        self.send = send
        self.snapshot = snapshot
        self.task = None
        self.starting = False
        self.token = None
        self.deadline = 0
        self.duid = None
        self.map_flag = None
        self.motion = None
        self.state = {"phase": "idle", "step": 0, "total": 0, "message": ""}
        self.lock = asyncio.Lock()
        self.browser_closed = asyncio.Event()

    @property
    def active(self):
        return self.starting or (self.task is not None and not self.task.done())

    def status(self):
        return {**self.state, "active": self.active}

    async def start(self, duid, map_flag, steps, name, end_session=True):
        steps = validate_steps(steps)
        async with self.lock:
            if self.active:
                raise ValueError("Un déplacement ou parcours est déjà en cours.")
            self.starting = True
            try:
                if not duid or map_flag is None:
                    raise ValueError("Carte indisponible.")
                self.duid, self.map_flag = duid, map_flag
                st = await self.snapshot(duid, False)
                self._check(st)
                if st["state"] not in self.IDLE or st.get("in_cleaning"):
                    raise ValueError("Arrêtez le nettoyage ou le pilotage avant de lancer un parcours.")
                self.end_session = end_session
                self.browser_closed.clear()
                self.token = secrets.token_hex(16)
                self.deadline = time.monotonic() + self.LEASE
                self.state = {"phase": "running", "step": 0, "total": len(steps), "name": name,
                              "message": "Démarrage…", "end_session": end_session, "camera_revision": 0}
                self.task = asyncio.create_task(self._run(copy.deepcopy(steps)))
                return self.token
            finally:
                self.starting = False

    def touch(self, token):
        if not token or token != self.token:
            raise ValueError("Session de déplacement expirée.")
        if self.active:
            self.deadline = time.monotonic() + self.LEASE
        return self.status()

    def camera_closed(self, token):
        self.touch(token)
        if not self.state.get("camera_close_requested"):
            raise ValueError("Fermeture de la vidéo non demandée.")
        self.browser_closed.set()
        return self.status()

    async def stop(self, token=None):
        async with self.lock:
            if token is not None and token != self.token:
                raise ValueError("Session de déplacement expirée.")
            if self.active:
                self.state.update(phase="stopping", message="Arrêt…")
                self.task.cancel()
                try:
                    await self.task
                except asyncio.CancelledError:
                    pass
            return self.status()

    def _check(self, st):
        if st.get("error"):
            raise ValueError("Le robot signale une erreur. Parcours arrêté.")
        if st.get("map_flag") != self.map_flag:
            raise ValueError("La carte a changé. Parcours arrêté.")
        if st.get("in_cleaning"):
            raise ValueError("Un nettoyage est en cours. Parcours arrêté.")

    async def _watch(self):
        while True:
            if time.monotonic() >= self.deadline:
                self.state.update(phase="error", message="Connexion à la page perdue. Session arrêtée.")
                self.task.cancel()
                return
            await asyncio.sleep(0.5)

    async def _command(self, method, params=None):
        return await asyncio.wait_for(self.send(self.duid, method, params), timeout=12)

    async def _navigation_command(self, method, params=None):
        # Remote viewing can retain the action lock after app_rc_end, even
        # though get_status already says idle. Release only the camera session,
        # never the user's child-lock setting, and retry the rejected navigation.
        deadline = time.monotonic() + self.UNLOCK_TIMEOUT
        released_camera = False
        while True:
            try:
                result = await self._command(method, params)
                if released_camera and method != "app_charge":
                    self.state["camera_revision"] = self.state.get("camera_revision", 0) + 1
                return result
            except Exception as exc:
                if not any(s in str(exc).lower() for s in ("action locked", "invalid status", "-10003", "-10007")):
                    raise
                if time.monotonic() >= deadline:
                    raise ValueError("Le robot garde le déplacement verrouillé. Réessayez après la fermeture de la vidéo.") from exc
                st = await self.snapshot(self.duid, False)
                self._check(st)
                if st.get("home_sec_status") and not released_camera:
                    await self._command("stop_camera_preview", {"client_id": "676f32727463"})
                    released_camera = True
                await asyncio.sleep(self.POLL)

    async def _goto(self, step):
        self.motion = "goto"
        await self._navigation_command("app_goto_target", [round(step["x"]), round(step["y"])])
        deadline = time.monotonic() + self.GOTO_TIMEOUT
        entered = False
        started = time.monotonic()
        while time.monotonic() < deadline:
            await asyncio.sleep(self.POLL)
            st = await self.snapshot(self.duid, False)
            self._check(st)
            if st["state"] == self.GOTO:
                entered = True
                continue
            if st["state"] in self.IDLE:
                # A fresh map checks arrival, including targets already under the robot.
                # Idle on its own can also mean a blocked or cancelled journey.
                if not entered and time.monotonic() - started < 3:
                    continue
                st = await self.snapshot(self.duid, True)
                self._check(st)
                pos = st.get("position")
                if st["state"] in self.IDLE and pos and math.hypot(pos[0]-step["x"], pos[1]-step["y"]) <= 350:
                    self.motion = None
                    return
                if entered or time.monotonic() - started > 12:
                    raise ValueError("Le robot n’a pas atteint le point demandé.")
            elif st["state"] != self.GOTO:
                raise ValueError("Déplacement interrompu. Parcours arrêté.")
        raise ValueError("Point non atteint après 3 minutes. Parcours arrêté.")

    async def _close_remote(self):
        # Let the owning browser stop its microphone, audio and WebRTC before
        # shutting down the robot's session. It must not reconnect during docking.
        self.state.update(camera_close_requested=True, message="Fermeture de la vidéo et du pilotage…")
        try:
            await asyncio.wait_for(self.browser_closed.wait(), self.CLOSE_TIMEOUT)
        except asyncio.TimeoutError:
            raise ValueError("Fermeture de la vidéo non confirmée. Actualisez la page puis réessayez.") from None
        self.motion = "turn"
        await self._command("app_rc_end")
        await self._command("stop_camera_preview", {"client_id": "676f32727463"})
        deadline = time.monotonic() + self.UNLOCK_TIMEOUT
        while time.monotonic() < deadline:
            st = await self.snapshot(self.duid, False)
            self._check(st)
            if st["state"] in self.IDLE and not st.get("home_sec_status"):
                self.motion = None
                self.state["camera_closed"] = True
                return
            await asyncio.sleep(self.POLL)
        raise ValueError("Le robot n’a pas confirmé la fermeture du pilotage et de la vidéo.")

    async def _home(self):
        self.end_session = True
        self.state["end_session"] = True
        st = await self.snapshot(self.duid, True)
        self._check(st)
        docked = st["state"] in (8, 100)
        if not docked:
            target = st.get("dock_approach")
            if not target:
                raise ValueError("Point d’approche de la station indisponible sur la carte.")
            self.state["message"] = "Approche de la station…"
            pos = st.get("position")
            if not pos or math.hypot(pos[0]-target[0], pos[1]-target[1]) > 350:
                await self._goto({"x": target[0], "y": target[1]})
        await self._close_remote()
        if docked:
            return
        self.state["message"] = "Vidéo fermée · Retour à la station…"
        self.motion = "home"
        await self._navigation_command("app_charge")
        deadline = time.monotonic() + self.HOME_TIMEOUT
        started = time.monotonic()
        while time.monotonic() < deadline:
            await asyncio.sleep(self.POLL)
            st = await self.snapshot(self.duid, False)
            self._check(st)
            if st["state"] in (8, 100, 22, 23, 25):
                self.motion = None
                return
            if st["state"] in (6, 15, 26, 6310):
                continue
            if st["state"] in self.IDLE and time.monotonic()-started < 10:
                continue
            raise ValueError(f"Retour à la station interrompu (état {st['state']}).")
        raise ValueError("Station non atteinte après 5 minutes. Session arrêtée.")

    async def _turn(self, step):
        self.motion = "turn"
        await self._command("app_rc_start")
        await asyncio.sleep(self.PAIR_DELAY)
        st = await self.snapshot(self.duid, False)
        self._check(st)
        if st["state"] not in self.MANUAL:
            raise ValueError("Le robot n’a pas activé le pilotage.")
        # Timed rotation, deliberately without heading tracking or calibration.
        speed = self.OMEGA * step.get("speed", 1)
        remaining = math.radians(abs(step["degrees"])) / speed
        omega = speed if step["degrees"] < 0 else -speed
        seq = 0
        while remaining > 0.001:
            duration = min(0.5, remaining)
            seq += 1
            await self._command("app_rc_move", [{"velocity": 0, "omega": omega,
                                                 "duration": max(100, round(duration*1000)), "seqnum": seq}])
            await asyncio.sleep(duration)
            remaining -= duration
            if seq % 3 == 0:
                st = await self.snapshot(self.duid, False)
                self._check(st)
                if st["state"] not in self.MANUAL:
                    raise ValueError("Rotation interrompue. Parcours arrêté.")
        await self._command("app_rc_end")
        deadline = time.monotonic() + self.UNLOCK_TIMEOUT
        while time.monotonic() < deadline:
            await asyncio.sleep(self.POLL)
            st = await self.snapshot(self.duid, False)
            self._check(st)
            if st["state"] in self.IDLE:
                self.motion = None
                return
            if st["state"] not in self.MANUAL:
                raise ValueError("Fin de rotation interrompue.")
        raise ValueError("Le robot n’a pas quitté le pilotage après la rotation.")

    async def _cleanup(self):
        failures = []
        commands = ["app_stop"] if self.motion in ("goto", "home") else ["app_rc_end"] if self.motion == "turn" else []
        if not self.state.get("camera_closed") and (self.end_session or self.state["phase"] != "complete"):
            commands.append("stop_camera_preview")
        for method in commands:
            try:
                await self._command(method, {"client_id": "676f32727463"} if method == "stop_camera_preview" else None)
            except Exception:
                failures.append(method)
        self.motion = None
        if failures:
            original = self.state.get("message", "") if self.state["phase"] == "error" else ""
            self.state.update(phase="error", cleanup_errors=failures,
                              message=(original + " " if original else "") + "Fin de session non confirmée par le robot. Vérifiez sa connexion.")

    async def _run(self, steps):
        watcher = asyncio.create_task(self._watch())
        try:
            for i, step in enumerate(steps):
                self._check(await self.snapshot(self.duid, False))
                kind = step["kind"]
                label = {"goto": "Déplacement", "turn": "Observation en rotation", "wait": "Pause d’observation", "home": "Retour à la station"}[kind]
                self.state.update(step=i+1, message=f"{i+1}/{len(steps)} · {label}")
                if kind == "goto":
                    await self._goto(step)
                elif kind == "turn":
                    await self._turn(step)
                elif kind == "home":
                    await self._home()
                else:
                    deadline = time.monotonic() + step["seconds"]
                    while time.monotonic() < deadline:
                        await asyncio.sleep(min(1, deadline-time.monotonic()))
                        st = await self.snapshot(self.duid, False)
                        self._check(st)
                        if st["state"] not in self.IDLE:
                            raise ValueError("Observation interrompue. Parcours arrêté.")
            self.state.update(phase="complete", message="Parcours terminé. Session fermée." if self.end_session else "Destination atteinte.")
        except asyncio.CancelledError:
            if self.state["phase"] != "error":
                self.state.update(phase="stopped", message="Session arrêtée.")
        except Exception as exc:
            self.state.update(phase="error", message=str(exc) or "Connexion au robot perdue.")
        finally:
            watcher.cancel()
            await self._cleanup()
