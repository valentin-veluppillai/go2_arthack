#!/usr/bin/env python3
"""Minimal CLI client for the Go2 ZMQ bridge.

Usage:
    python go2_client/cli_client.py status
    python go2_client/cli_client.py list_actions
    python go2_client/cli_client.py action stand_up
    python go2_client/cli_client.py move 0.3 0.0 0.0
    python go2_client/cli_client.py stop
    python go2_client/cli_client.py camera_frame          # saves one JPEG
    python go2_client/cli_client.py obstacle_avoidance 1
    python go2_client/cli_client.py speed_level 2
    python go2_client/cli_client.py slam mapping/start    # any uslam command
"""

from __future__ import annotations

import json
import os
import sys

import zmq

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
CMD_PORT = int(os.getenv("GO2_ZMQ_CMD_PORT", "5555"))
PUB_PORT = int(os.getenv("GO2_ZMQ_PUB_PORT", "5556"))


def send_command(cmd: str, params: dict | None = None) -> dict:
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 5000)
    sock.connect(f"tcp://{BRIDGE_HOST}:{CMD_PORT}")

    msg: dict = {"cmd": cmd}
    if params:
        msg["params"] = params
    sock.send_json(msg)

    resp = sock.recv_json()
    sock.close()
    ctx.term()
    return resp


def save_camera_frame(path: str = "frame.jpg") -> None:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVTIMEO, 5000)
    sub.connect(f"tcp://{BRIDGE_HOST}:{PUB_PORT}")
    sub.subscribe(b"camera")

    print("Waiting for camera frame…")
    topic, data = sub.recv_multipart()
    with open(path, "wb") as f:
        f.write(data)
    print(f"Saved {len(data)} bytes to {path}")

    sub.close()
    ctx.term()


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    subcmd = sys.argv[1]

    if subcmd == "camera_frame":
        save_camera_frame()
        return

    if subcmd == "status":
        resp = send_command("status")
    elif subcmd == "list_actions":
        resp = send_command("list_actions")
    elif subcmd == "stop":
        resp = send_command("stop")
    elif subcmd == "action":
        if len(sys.argv) < 3:
            print("Usage: cli_client.py action <name>")
            sys.exit(1)
        resp = send_command("action", {"name": sys.argv[2]})
    elif subcmd == "move":
        if len(sys.argv) < 5:
            print("Usage: cli_client.py move <vx> <vy> <vyaw>")
            sys.exit(1)
        resp = send_command("move", {
            "vx": float(sys.argv[2]),
            "vy": float(sys.argv[3]),
            "vyaw": float(sys.argv[4]),
        })
    elif subcmd == "obstacle_avoidance":
        enabled = sys.argv[2] if len(sys.argv) > 2 else "1"
        resp = send_command("obstacle_avoidance", {"enabled": enabled == "1"})
    elif subcmd == "speed_level":
        level = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        resp = send_command("speed_level", {"level": level})
    elif subcmd == "slam":
        if len(sys.argv) < 3:
            print("Usage: cli_client.py slam <module/action>   e.g. mapping/start, mapping/stop")
            sys.exit(1)
        resp = send_command("slam", {"command": sys.argv[2]})
    else:
        print(f"Unknown subcommand: {subcmd}")
        sys.exit(1)

    print(json.dumps(resp, indent=2))


if __name__ == "__main__":
    main()
