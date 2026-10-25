"""Person follower: python -m go2_follow

Reads go2_yolo's detections and steers the robot so the largest person
stays in frame:

  * box centre near the left / right edge  → turn left / right
  * box area below AREA_FAR of the frame   → walk forward (person is far)
  * box area above AREA_NEAR of the frame  → walk backward (person is close)

Deadman switch: the robot only moves while SPACE is held in the follower
window. Releasing it (or the window losing focus) sends an immediate stop;
the bridge's 250 ms movement timeout is the second safety net.
"""

from __future__ import annotations

import io
import json
import logging
import os
import time

import pygame
import zmq

BRIDGE_HOST = os.getenv("GO2_BRIDGE_HOST", "localhost")
CMD_PORT = int(os.getenv("GO2_ZMQ_CMD_PORT", "5555"))
YOLO_HOST = os.getenv("GO2_YOLO_HOST", BRIDGE_HOST)
YOLO_PUB_PORT = int(os.getenv("GO2_YOLO_PUB_PORT", "5557"))

FOLLOW_EDGE = float(os.getenv("GO2_FOLLOW_EDGE", "0.4"))  # turn zone, fraction of width per side
FOLLOW_AREA_FAR = float(os.getenv("GO2_FOLLOW_AREA_FAR", "0.15"))  # walk forward below this
FOLLOW_AREA_NEAR = float(os.getenv("GO2_FOLLOW_AREA_NEAR", "0.17"))  # walk backward above this
FOLLOW_VX = float(os.getenv("GO2_FOLLOW_VX", "0.3"))  # m/s
FOLLOW_VYAW = float(os.getenv("GO2_FOLLOW_VYAW", "0.7"))  # rad/s
FOLLOW_CONF = float(os.getenv("GO2_FOLLOW_CONF", "0.5"))
FOLLOW_STALE_S = 0.5  # stop if no detections for this long
FOLLOW_HZ = 20

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("go2_follow")


# ── control logic ─────────────────────────────────────────────────


def pick_person(msg: dict) -> dict | None:
    """Largest confident person box, or None."""
    people = [d for d in msg["detections"] if d["cls"] == "person" and d["conf"] >= FOLLOW_CONF]
    if not people:
        return None
    return max(people, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))


def decide(msg: dict | None) -> tuple[float, float, str]:
    """Return (vx, vyaw, reason) for one detections message."""
    if msg is None:
        return 0.0, 0.0, "no detections"
    person = pick_person(msg)
    if person is None:
        return 0.0, 0.0, "no person"

    x1, y1, x2, y2 = person["box"]
    w, h = msg["width"], msg["height"]
    cx = (x1 + x2) / 2 / w
    area = (x2 - x1) * (y2 - y1) / (w * h)

    vx, vyaw = 0.0, 0.0
    reasons = []
    if cx < FOLLOW_EDGE:
        vyaw = FOLLOW_VYAW  # positive yaw = counter-clockwise = left
        reasons.append("turn left")
    elif cx > 1 - FOLLOW_EDGE:
        vyaw = -FOLLOW_VYAW
        reasons.append("turn right")
    if area < FOLLOW_AREA_FAR:
        vx = FOLLOW_VX
        reasons.append("forward")
    elif area > FOLLOW_AREA_NEAR:
        vx = -FOLLOW_VX
        reasons.append("backward")

    reason = " + ".join(reasons) or "hold"
    return vx, vyaw, f"{reason} (cx {cx:.2f}, area {area:.3f})"


# ── bridge client ─────────────────────────────────────────────────


class Bridge:
    """REQ client that recreates its socket after a timeout (a REQ socket is stuck otherwise)."""

    def __init__(self, ctx: zmq.Context) -> None:
        self._ctx = ctx
        self._sock: zmq.Socket | None = None

    def send(self, cmd: str, params: dict | None = None) -> bool:
        if self._sock is None:
            self._sock = self._ctx.socket(zmq.REQ)
            self._sock.setsockopt(zmq.RCVTIMEO, 200)
            self._sock.setsockopt(zmq.LINGER, 0)
            self._sock.connect(f"tcp://{BRIDGE_HOST}:{CMD_PORT}")
        msg: dict = {"cmd": cmd}
        if params:
            msg["params"] = params
        try:
            self._sock.send_json(msg)
            return bool(self._sock.recv_json().get("ok"))
        except zmq.Again:
            log.warning("Bridge did not answer '%s'", cmd)
            self._sock.close()
            self._sock = None
            return False

    def move(self, vx: float, vyaw: float) -> bool:
        return self.send("move", {"vx": vx, "vy": 0.0, "vyaw": vyaw})

    def stop(self) -> bool:
        return self.send("stop")

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()


# ── UI ────────────────────────────────────────────────────────────


def draw(screen: pygame.Surface, font: pygame.font.Font, frame: pygame.Surface | None,
         held: bool, vx: float, vyaw: float, reason: str) -> None:
    sw, sh = screen.get_size()
    bar = 60
    screen.fill((20, 20, 20))
    if frame is not None:
        screen.blit(pygame.transform.smoothscale(frame, (sw, sh - bar)), (0, 0))
    # turn zones
    for x in (int(sw * FOLLOW_EDGE), int(sw * (1 - FOLLOW_EDGE))):
        pygame.draw.line(screen, (255, 255, 0), (x, 0), (x, sh - bar), 1)

    color = (30, 140, 60) if held else (170, 40, 40)
    pygame.draw.rect(screen, color, (0, sh - bar, sw, bar))
    state = "ACTIVE" if held else "HOLD SPACE TO MOVE"
    line1 = f"{state}   vx {vx:+.2f} m/s   vyaw {vyaw:+.2f} rad/s"
    screen.blit(font.render(line1, True, (255, 255, 255)), (12, sh - bar + 6))
    screen.blit(font.render(reason, True, (230, 230, 230)), (12, sh - bar + 32))
    pygame.display.flip()


def main() -> None:
    ctx = zmq.Context()
    bridge = Bridge(ctx)

    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{YOLO_HOST}:{YOLO_PUB_PORT}")
    sub.subscribe(b"detections")
    sub.subscribe(b"camera_yolo")
    log.info("Following detections from tcp://%s:%d, commands to tcp://%s:%d",
             YOLO_HOST, YOLO_PUB_PORT, BRIDGE_HOST, CMD_PORT)

    pygame.init()
    screen = pygame.display.set_mode((960, 600), pygame.RESIZABLE)
    pygame.display.set_caption("go2 follow — hold SPACE to move")
    font = pygame.font.SysFont(None, 26)
    clock = pygame.time.Clock()

    latest: dict | None = None
    t_latest = 0.0
    frame: pygame.Surface | None = None
    was_held = False
    running = True

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q)
                ):
                    running = False

            # drain everything that arrived, keep the newest of each topic
            while True:
                try:
                    topic, payload = sub.recv_multipart(zmq.NOBLOCK)
                except zmq.Again:
                    break
                if topic == b"detections":
                    latest, t_latest = json.loads(payload), time.monotonic()
                else:
                    frame = pygame.image.load(io.BytesIO(payload), "frame.jpg")

            fresh = latest if time.monotonic() - t_latest < FOLLOW_STALE_S else None
            vx, vyaw, reason = decide(fresh)

            # deadman: SPACE held and our window focused (SDL clears key state on focus loss)
            held = running and pygame.key.get_focused() and pygame.key.get_pressed()[pygame.K_SPACE]
            if held:
                bridge.move(vx, vyaw)
            elif was_held:
                bridge.stop()
                log.info("Deadman released — stop")
            if held and not was_held:
                log.info("Deadman held — following")
            was_held = held

            draw(screen, font, frame, held, vx if held else 0.0, vyaw if held else 0.0, reason)
            clock.tick(FOLLOW_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
        bridge.close()
        sub.close()
        ctx.term()
        pygame.quit()


if __name__ == "__main__":
    main()
