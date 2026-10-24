"""Action registry and JSON message helpers."""

from __future__ import annotations

import json
from typing import Any

# Maps action name -> (method_name, args, kwargs)
ACTION_REGISTRY: dict[str, tuple[str, tuple, dict]] = {
    "stand_up":       ("StandUp", (), {}),
    "stand_down":     ("StandDown", (), {}),
    "balance_stand":  ("BalanceStand", (), {}),
    "recovery_stand": ("RecoveryStand", (), {}),
    "sit":            ("Sit", (), {}),
    "hello":          ("Hello", (), {}),
    "stretch":        ("Stretch", (), {}),
    "dance1":         ("Dance1", (), {}),
    "dance2":         ("Dance2", (), {}),
    "heart":          ("Heart", (), {}),
    "front_flip":     ("FrontFlip", (), {}),
    "front_jump":     ("FrontJump", (), {}),
    "back_flip":      ("BackFlip", (), {}),
    "left_flip":      ("LeftFlip", (), {}),
    "hand_stand":     ("HandStand", (True,), {}),
    "damp":           ("Damp", (), {}),
    "stop_move":      ("StopMove", (), {}),
}


def cloud_meta(msg: Any) -> dict[str, Any]:
    """PointCloud2 header as a JSON-able dict; sent next to the raw data buffer."""
    return {
        "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": msg.header.frame_id,
        "width": msg.width,
        "height": msg.height,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "is_bigendian": msg.is_bigendian,
        "is_dense": msg.is_dense,
        "fields": [
            {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
            for f in msg.fields
        ],
    }


def make_request(cmd: str, params: dict[str, Any] | None = None) -> bytes:
    """Encode a command request as JSON bytes."""
    msg: dict[str, Any] = {"cmd": cmd}
    if params is not None:
        msg["params"] = params
    return json.dumps(msg).encode()


def make_response(ok: bool, msg: str = "", data: Any = None) -> bytes:
    """Encode a command response as JSON bytes."""
    return json.dumps({"ok": ok, "msg": msg, "data": data}).encode()


def parse_request(raw: bytes) -> tuple[str, dict[str, Any]]:
    """Decode a command request. Returns (cmd, params)."""
    obj = json.loads(raw)
    return obj.get("cmd", ""), obj.get("params", {})
