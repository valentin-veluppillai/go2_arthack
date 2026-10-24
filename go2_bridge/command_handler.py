"""ZMQ REQ/REP command processing loop."""

from __future__ import annotations

import json
import logging
import threading

import zmq

from . import config
from .protocol import ACTION_REGISTRY, make_response, parse_request

log = logging.getLogger(__name__)


class CommandHandler:
    """Listens on a ZMQ REP socket and dispatches commands."""

    def __init__(self, robot, movement_loop, ctx: zmq.Context) -> None:
        self._robot = robot
        self._movement = movement_loop
        self._ctx = ctx
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="cmd-handler")
        self._thread.start()
        log.info("Command handler listening on port %d", config.ZMQ_CMD_PORT)

    def shutdown(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        rep = self._ctx.socket(zmq.REP)
        rep.bind(f"tcp://*:{config.ZMQ_CMD_PORT}")
        poller = zmq.Poller()
        poller.register(rep, zmq.POLLIN)

        while self._running:
            events = dict(poller.poll(timeout=500))
            if rep not in events:
                continue
            raw = rep.recv()
            try:
                cmd, params = parse_request(raw)
                response = self._dispatch(cmd, params)
            except Exception as exc:
                log.exception("Error handling command")
                response = make_response(False, str(exc))
            rep.send(response)

        rep.close()

    def _dispatch(self, cmd: str, params: dict) -> bytes:
        if cmd == "action":
            return self._handle_action(params)
        if cmd == "move":
            return self._handle_move(params)
        if cmd == "stop":
            return self._handle_stop()
        if cmd == "obstacle_avoidance":
            return self._handle_obstacle_avoidance(params)
        if cmd == "speed_level":
            return self._handle_speed_level(params)
        if cmd == "light":
            return self._handle_light(params)
        if cmd == "list_actions":
            return self._handle_list_actions()
        if cmd == "status":
            return self._handle_status()
        if cmd == "slam":
            return self._handle_slam(params)
        return make_response(False, f"unknown command: {cmd}")

    # ── handlers ──────────────────────────────────────────────────

    def _handle_action(self, params: dict) -> bytes:
        name = params.get("name", "")
        entry = ACTION_REGISTRY.get(name)
        if entry is None:
            return make_response(False, f"unknown action: {name}")
        method_name, args, kwargs = entry
        try:
            code = self._robot.execute_action(method_name, args, kwargs)
            return make_response(True, f"{name} executed", {"code": code})
        except Exception as exc:
            return make_response(False, f"{name} failed: {exc}")

    def _handle_move(self, params: dict) -> bytes:
        vx = float(params.get("vx", 0.0))
        vy = float(params.get("vy", 0.0))
        vyaw = float(params.get("vyaw", 0.0))
        self._movement.set_velocity(vx, vy, vyaw)
        return make_response(True, "velocity updated")

    def _handle_stop(self) -> bytes:
        self._movement.stop()
        self._robot.move(0.0, 0.0, 0.0)
        return make_response(True, "stopped")

    def _handle_obstacle_avoidance(self, params: dict) -> bytes:
        enabled = bool(params.get("enabled", True))
        try:
            self._robot.set_obstacle_avoidance(enabled)
            return make_response(True, f"obstacle avoidance {'enabled' if enabled else 'disabled'}")
        except Exception as exc:
            return make_response(False, str(exc))

    def _handle_speed_level(self, params: dict) -> bytes:
        level = int(params.get("level", 1))
        self._robot.speed_level = level
        return make_response(True, f"speed level set to {self._robot.speed_level}")

    def _handle_light(self, params: dict) -> bytes:
        on = bool(params.get("on", True))
        try:
            code = self._robot.set_light(on)
            state = "on" if self._robot.light_on else "off"
            return make_response(code == 0, f"light {state}", {"code": code})
        except Exception as exc:
            return make_response(False, str(exc))

    def _handle_list_actions(self) -> bytes:
        return make_response(True, "actions", {"actions": list(ACTION_REGISTRY.keys())})

    def _handle_slam(self, params: dict) -> bytes:
        command = str(params.get("command", ""))
        if command.count("/") < 1:
            return make_response(False, "expected a uslam command like 'mapping/start'")
        self._robot.uslam_command(command)
        return make_response(True, f"sent uslam command {command} (ack on the SLAM log stream)")

    def _handle_status(self) -> bytes:
        return make_response(True, "ok", {
            "obstacle_avoidance": self._robot.obstacle_avoidance_enabled,
            "speed_level": self._robot.speed_level,
            "light_on": self._robot.light_on,
        })
