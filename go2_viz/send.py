"""Send visual parameters to go2_viz: python -m go2_viz.send key=value ...

Example:
    python -m go2_viz.send particles=8000 jitter=0.4 noise=0.3 markerChance=0.2 paintColor=#ff8c1a
    python -m go2_viz.send paintColor=height

The same message format is what a robot-data process publishes: connect a ZMQ
PUB socket to tcp://<viz host>:5561 and send [b"viz", json].
"""

from __future__ import annotations

import json
import os
import sys
import time

import zmq

VIZ_HOST = os.getenv("GO2_VIZ_PARAM_HOST", "localhost")
VIZ_PARAM_PORT = int(os.getenv("GO2_VIZ_PARAM_PORT", "5561"))


def parse(value: str):
    try:
        return float(value)
    except ValueError:
        return value


def main() -> None:
    if len(sys.argv) < 2 or any("=" not in arg for arg in sys.argv[1:]):
        print(__doc__)
        sys.exit(1)
    update = {k: parse(v) for k, v in (arg.split("=", 1) for arg in sys.argv[1:])}

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://{VIZ_HOST}:{VIZ_PARAM_PORT}")
    time.sleep(0.3)  # PUB/SUB drops messages sent before the connection is up
    pub.send_multipart([b"viz", json.dumps(update).encode()])
    print(f"sent {update} to tcp://{VIZ_HOST}:{VIZ_PARAM_PORT}")
    pub.close(linger=1000)
    ctx.term()


if __name__ == "__main__":
    main()
