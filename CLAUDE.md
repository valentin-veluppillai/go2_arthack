# Go2 Robot Control Project

## Project Overview

This project provides a reusable interface layer for the Unitree Go2 quadruped robot. A ZeroMQ bridge process (`go2_bridge`) sits between the Unitree SDK and any number of clients, exposing a clean JSON protocol for actions, movement, and camera streaming. Three client applications are included: a CLI tool, a web app, and an MCP server.

## Directory Layout

```
go2/
├── go2_bridge/          # ZMQ bridge process (python -m go2_bridge)
│   ├── config.py        # Env-var configuration
│   ├── protocol.py      # Action registry + JSON message helpers
│   ├── robot.py         # SDK wrapper (SportClient, ObstaclesAvoidClient, VideoClient)
│   ├── movement_loop.py # 20 Hz heartbeat thread with 250ms timeout safety
│   ├── camera_publisher.py  # Camera JPEG → ZMQ PUB thread
│   ├── lidar_publisher.py   # LiDAR PointCloud2 (DDS) → ZMQ PUB thread (port 5558)
│   ├── slam_publisher.py    # uslam scans/pose/log (DDS) → ZMQ PUB thread (port 5559)
│   ├── command_handler.py   # ZMQ REQ/REP command dispatcher
│   └── __main__.py      # Entry point, wires components, graceful shutdown
├── go2_client/
│   └── cli_client.py    # CLI demo (send commands, save camera frames)
├── go2_webapp/          # FastAPI web app (python -m go2_webapp)
│   ├── __main__.py      # HTTP/WS server, proxies to bridge via ZMQ
│   └── static/          # Single-page UI (HTML + vanilla JS + CSS)
├── go2_mcp/             # MCP server (python -m go2_mcp)
│   ├── server.py        # FastMCP tools → ZMQ bridge client
│   └── __main__.py      # Entry point (stdio transport)
├── go2_lidar/           # LiDAR subscriber, logs point counts (python -m go2_lidar)
├── go2_map/             # Live 3D map from uslam in a Rerun viewer (python -m go2_map)
├── unitree_sdk2_python/  # Unitree SDK2 Python (clone, not committed)
├── go2_py311/           # Python 3.11 venv (not committed)
├── requirements_bridge.txt
├── requirements_map.txt
└── CLAUDE.md
```

## Running

```bash
# Activate venv
source go2_py311/bin/activate

# Start the bridge (must be on same network as robot)
GO2_NETWORK_INTERFACE=eno1 python -m go2_bridge

# In another terminal — CLI
python go2_client/cli_client.py status
python go2_client/cli_client.py action stand_up
python go2_client/cli_client.py move 0.3 0.0 0.0

# In another terminal — Web app
python -m go2_webapp
# Open http://localhost:8080

# MCP server (for Claude Code / Claude Desktop)
python -m go2_mcp

# LiDAR subscriber (logs point counts)
python -m go2_lidar

# Live 3D map from the on-board SLAM (deps: pip install -r requirements_map.txt)
# sends mapping/start; 'cli_client.py slam mapping/stop' saves and overwrites the robot's single map slot
python -m go2_map
```

**Claude Desktop configuration** (`claude_desktop_config.json`):
```json
{
  "mcpServers": {
    "go2": {
      "command": "/path/to/go2/go2_py311/bin/python",
      "args": ["/path/to/go2/go2_mcp/server.py"]
    }
  }
}
```

## Configuration

All config is via environment variables (defaults in `go2_bridge/config.py`):

| Variable | Default | Description |
|----------|---------|-------------|
| `GO2_NETWORK_INTERFACE` | `eno1` | DDS network interface |
| `GO2_ZMQ_CMD_PORT` | `5555` | REQ/REP command port |
| `GO2_ZMQ_PUB_PORT` | `5556` | PUB/SUB stream port |
| `GO2_MOVE_HZ` | `20` | Movement loop frequency |
| `GO2_MOVE_TIMEOUT_MS` | `250` | Movement timeout (auto-stop) |
| `GO2_CAMERA_FPS` | `10` | Camera publish rate |
| `GO2_OBSTACLE_AVOIDANCE` | `1` | Enable obstacle avoidance at startup |
| `GO2_BRIDGE_HOST` | `localhost` | Bridge host (for clients) |
| `GO2_WEBAPP_HOST` | `0.0.0.0` | Web app bind address |
| `GO2_WEBAPP_PORT` | `8080` | Web app HTTP port |
| `GO2_LIDAR` | `1` | Enable the LiDAR publisher in the bridge |
| `GO2_LIDAR_TOPIC` | `rt/utlidar/cloud` | DDS PointCloud2 topic to forward |
| `GO2_LIDAR_SWITCH_ON` | `1` | Send `ON` to `rt/utlidar/switch` at startup |
| `GO2_ZMQ_LIDAR_PORT` | `5558` | LiDAR PUB/SUB port |
| `GO2_SLAM` | `1` | Forward the on-board SLAM (uslam) streams in the bridge |
| `GO2_ZMQ_SLAM_PORT` | `5559` | SLAM PUB/SUB port |
| `GO2_MAP_VOXEL` | `0.05` | Map resolution in metres (one point per voxel) |
| `GO2_MAP_CEILING` | `1.5` | Hide points this far above the robot's start height (0 = show all) |
| `GO2_MAP_START` | `1` | `go2_map` sends `mapping/start` at startup |
| `GO2_MAP_WEB` | `0` | Open the viewer in the browser instead of the native Rerun window |
| `GO2_MAP_SAVE` | — | Record to this `.rrd` file instead of showing a viewer |
| `GO2_MAP_FILTER` | `1` | Remove moving objects (people walking past) from the map |
| `GO2_MAP_FILTER_CELL` | `0.1` | Cell size in metres of the occupancy grid used by the filter |
| `GO2_MAP_PLY` | `go2_map_<timestamp>.ply` | Map file written on exit (`0` = off) |

## ZMQ Protocol

Commands go over REQ/REP (port 5555). Streams go over PUB/SUB (port 5556).

**Command examples:**
```json
{"cmd": "action", "params": {"name": "stand_up"}}
{"cmd": "move", "params": {"vx": 0.3, "vy": 0.0, "vyaw": 0.0}}
{"cmd": "stop"}
{"cmd": "status"}
{"cmd": "list_actions"}
```

**PUB topics:** `camera` (JPEG bytes), `state` (JSON).

## Key SDK References

- `unitree_sdk2_python/unitree_sdk2py/go2/sport/sport_client.py` — Action methods (StandUp, Move, etc.)
- `unitree_sdk2_python/unitree_sdk2py/go2/obstacles_avoid/obstacles_avoid_client.py` — Obstacle avoidance Move, SwitchSet, UseRemoteCommandFromApi
- `unitree_sdk2_python/unitree_sdk2py/go2/video/video_client.py` — GetImageSample (returns code + list of ints, must convert to bytes)
- `unitree_sdk2_python/example/obstacles_avoid/obstacles_avoid_move.py` — Reference init pattern

## Important SDK Notes

- `SportClient.Move()` and `ObstaclesAvoidClient.Move()` use `_CallNoReply` (fire-and-forget, safe at 20 Hz)
- All other sport actions use `_Call()` (blocking RPC)
- `VideoClient.GetImageSample()` returns `(code, list_of_ints)` — convert with `bytes(data)`
- Obstacle avoidance init must poll `SwitchGet()` + re-send `SwitchSet(True)` until confirmed (SDK example pattern)
- `UseRemoteCommandFromApi(True)` needs ~500ms before first move command takes effect
- Always call `UseRemoteCommandFromApi(False)` on shutdown

## Architecture Conventions

- Bridge components are started as daemon threads (movement_loop, camera_publisher, command_handler, lidar_publisher, slam_publisher)
- Movement safety: 20 Hz heartbeat relays last velocity; 250ms timeout zeros velocity automatically
- Graceful shutdown via SIGINT/SIGTERM: zeros velocity, disables remote API control, terminates ZMQ context
- Web app uses async ZMQ (`zmq.asyncio`) for non-blocking bridge communication
- Camera frames are relayed: SDK → ZMQ PUB → WebSocket → browser `<img>` tag via blob URLs
- MCP server is a thin ZMQ bridge client (stdio transport); all clients (CLI, web, MCP) can run simultaneously
- MCP `get_camera_frame` subscribes to PUB topic and returns a single frame as base64 JPEG
