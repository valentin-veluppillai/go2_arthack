"""SDK wrapper — initialise SportClient, ObstaclesAvoidClient, VideoClient."""

from __future__ import annotations

import logging
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import (
    ObstaclesAvoidClient,
)
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.vui.vui_client import VuiClient
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from . import config

log = logging.getLogger(__name__)


class Robot:
    """High-level handle to the Go2 robot SDK clients."""

    def __init__(self) -> None:
        self.sport: SportClient | None = None
        self.obstacles: ObstaclesAvoidClient | None = None
        self.video: VideoClient | None = None
        self.vui: VuiClient | None = None
        self._uslam_cmd: ChannelPublisher | None = None
        self.obstacle_avoidance_enabled: bool = config.OBSTACLE_AVOIDANCE
        self._speed_level: int = 1  # 1 = normal
        self._light_on: bool = False

    # ── lifecycle ──────────────────────────────────────────────────

    def init(self) -> None:
        """Initialise DDS channel and all SDK clients."""
        log.info("Initialising DDS on interface %s", config.NETWORK_INTERFACE)
        ChannelFactoryInitialize(0, config.NETWORK_INTERFACE)

        # SportClient
        self.sport = SportClient()
        self.sport.SetTimeout(3.0)
        self.sport.Init()
        log.info("SportClient ready")

        # ObstaclesAvoidClient
        self.obstacles = ObstaclesAvoidClient()
        self.obstacles.SetTimeout(3.0)
        self.obstacles.Init()
        log.info("ObstaclesAvoidClient ready")

        if self.obstacle_avoidance_enabled:
            self._enable_obstacle_avoidance()

        # VideoClient
        self.video = VideoClient()
        self.video.SetTimeout(3.0)
        self.video.Init()
        log.info("VideoClient ready")

        # VuiClient (light/volume)
        self.vui = VuiClient()
        self.vui.SetTimeout(3.0)
        self.vui.Init()
        log.info("VuiClient ready")

        # uSLAM command channel (plain "module/action" strings)
        self._uslam_cmd = ChannelPublisher("rt/uslam/client_command", String_)
        self._uslam_cmd.Init()

    def shutdown(self) -> None:
        """Clean up: zero velocity, disable remote API control."""
        log.info("Shutting down robot clients")
        try:
            if self.sport:
                self.sport.Move(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            if self.obstacles:
                self.obstacles.UseRemoteCommandFromApi(False)
        except Exception:
            pass

    # ── obstacle avoidance ────────────────────────────────────────

    def _enable_obstacle_avoidance(self) -> None:
        log.info("Enabling obstacle avoidance")
        # Poll until switch is confirmed on, re-sending SwitchSet each iteration
        # (matches SDK example pattern)
        for _ in range(50):
            code, on = self.obstacles.SwitchGet()
            if on:
                break
            self.obstacles.SwitchSet(True)
            time.sleep(0.1)
        else:
            log.warning("Obstacle avoidance switch did not confirm on")

        log.info("Obstacle avoidance switch confirmed on")
        self.obstacles.UseRemoteCommandFromApi(True)
        # Wait for API control to take effect before first move
        time.sleep(0.5)
        self.obstacle_avoidance_enabled = True
        log.info("Obstacle avoidance enabled")

    def _disable_obstacle_avoidance(self) -> None:
        log.info("Disabling obstacle avoidance")
        self.obstacles.UseRemoteCommandFromApi(False)
        self.obstacles.SwitchSet(False)
        self.obstacle_avoidance_enabled = False
        log.info("Obstacle avoidance disabled")

    def set_obstacle_avoidance(self, enabled: bool) -> None:
        if enabled and not self.obstacle_avoidance_enabled:
            self._enable_obstacle_avoidance()
        elif not enabled and self.obstacle_avoidance_enabled:
            self._disable_obstacle_avoidance()

    # ── movement ──────────────────────────────────────────────────

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        """Send a single move command via the appropriate client."""
        if self.obstacle_avoidance_enabled:
            code = self.obstacles.Move(vx, vy, vyaw)
            if vx != 0 or vy != 0 or vyaw != 0:
                log.debug("obstacles.Move(%.2f, %.2f, %.2f) -> %s", vx, vy, vyaw, code)
        else:
            code = self.sport.Move(vx, vy, vyaw)
            if vx != 0 or vy != 0 or vyaw != 0:
                log.debug("sport.Move(%.2f, %.2f, %.2f) -> %s", vx, vy, vyaw, code)

    # ── actions ───────────────────────────────────────────────────

    def execute_action(self, method_name: str, args: tuple, kwargs: dict) -> int:
        """Call a SportClient method by name. Returns SDK code."""
        fn = getattr(self.sport, method_name)
        return fn(*args, **kwargs)

    # ── camera ────────────────────────────────────────────────────

    def get_image(self) -> tuple[int, bytes]:
        """Return (code, jpeg_bytes)."""
        return self.video.GetImageSample()

    # ── SLAM ──────────────────────────────────────────────────────

    def uslam_command(self, command: str) -> None:
        """Send a uslam_server command such as "mapping/start". Acks arrive on the server log."""
        msg = std_msgs_msg_dds__String_()
        msg.data = command
        self._uslam_cmd.Write(msg)

    # ── speed level ───────────────────────────────────────────────

    @property
    def speed_level(self) -> int:
        return self._speed_level

    @speed_level.setter
    def speed_level(self, level: int) -> None:
        self._speed_level = max(1, min(level, 3))
        if self.sport:
            self.sport.SpeedLevel(self._speed_level)

    # ── light ─────────────────────────────────────────────────────

    @property
    def light_on(self) -> bool:
        return self._light_on

    def set_light(self, on: bool) -> int:
        """Turn the head light on (max brightness) or off. Returns SDK code."""
        code = self.vui.SetBrightness(10 if on else 0)
        if code == 0:
            self._light_on = on
        return code
