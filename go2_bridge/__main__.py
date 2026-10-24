"""Entry point: python -m go2_bridge"""

import logging
import signal
import sys

import zmq

from . import config
from .camera_publisher import CameraPublisher
from .command_handler import CommandHandler
from .lidar_publisher import LidarPublisher
from .movement_loop import MovementLoop
from .slam_publisher import SlamPublisher
from .robot import Robot

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
log = logging.getLogger("go2_bridge")


def main() -> None:
    robot = Robot()
    robot.init()

    ctx = zmq.Context()
    movement = MovementLoop(robot)
    camera = CameraPublisher(robot, ctx)
    handler = CommandHandler(robot, movement, ctx)
    lidar = LidarPublisher(ctx) if config.LIDAR_ENABLED else None
    slam = SlamPublisher(ctx) if config.SLAM_ENABLED else None

    movement.start()
    camera.start()
    handler.start()
    if lidar:
        lidar.start()
    if slam:
        slam.start()

    log.info(
        "Go2 Bridge running — CMD port %d, PUB port %d, LiDAR port %s, SLAM port %s",
        config.ZMQ_CMD_PORT,
        config.ZMQ_PUB_PORT,
        config.ZMQ_LIDAR_PORT if lidar else "off",
        config.ZMQ_SLAM_PORT if slam else "off",
    )

    shutdown_event = False

    def on_signal(signum, frame):
        nonlocal shutdown_event
        if shutdown_event:
            return
        shutdown_event = True
        log.info("Shutting down…")
        handler.shutdown()
        if lidar:
            lidar.shutdown()
        if slam:
            slam.shutdown()
        camera.shutdown()
        movement.shutdown()
        robot.shutdown()
        ctx.term()
        log.info("Goodbye")
        sys.exit(0)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Block main thread
    signal.pause()


if __name__ == "__main__":
    main()
