"""Configuration from environment variables."""

import os


NETWORK_INTERFACE: str = os.getenv("GO2_NETWORK_INTERFACE", "eno1")

ZMQ_CMD_PORT: int = int(os.getenv("GO2_ZMQ_CMD_PORT", "5555"))
ZMQ_PUB_PORT: int = int(os.getenv("GO2_ZMQ_PUB_PORT", "5556"))

MOVE_HZ: int = int(os.getenv("GO2_MOVE_HZ", "20"))
MOVE_TIMEOUT_MS: int = int(os.getenv("GO2_MOVE_TIMEOUT_MS", "250"))

CAMERA_FPS: int = int(os.getenv("GO2_CAMERA_FPS", "10"))

LIDAR_ENABLED: bool = os.getenv("GO2_LIDAR", "1") == "1"
LIDAR_TOPIC: str = os.getenv("GO2_LIDAR_TOPIC", "rt/utlidar/cloud")
LIDAR_SWITCH_ON: bool = os.getenv("GO2_LIDAR_SWITCH_ON", "1") == "1"
ZMQ_LIDAR_PORT: int = int(os.getenv("GO2_ZMQ_LIDAR_PORT", "5558"))

SLAM_ENABLED: bool = os.getenv("GO2_SLAM", "1") == "1"
ZMQ_SLAM_PORT: int = int(os.getenv("GO2_ZMQ_SLAM_PORT", "5559"))

OBSTACLE_AVOIDANCE: bool = os.getenv("GO2_OBSTACLE_AVOIDANCE", "1") == "1"

BRIDGE_HOST: str = os.getenv("GO2_BRIDGE_HOST", "localhost")
WEBAPP_HOST: str = os.getenv("GO2_WEBAPP_HOST", "0.0.0.0")
WEBAPP_PORT: int = int(os.getenv("GO2_WEBAPP_PORT", "8080"))
