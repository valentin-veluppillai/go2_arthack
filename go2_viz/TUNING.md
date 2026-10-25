# Dialing in go2_viz

Where to turn each knob, from quickest to deepest. Line numbers are as of this
commit; search for the name if they have moved.

**Workflow.** Tune in the panel, then **Presets → save to server** (with
`GO2_VIZ_PRESET=look.json`) or **save to file**. Every browser that opens the
viz then starts with that look. A preset saved in a browser wins over the
server's in that browser; **reset to defaults** clears it. `h` hides all UI,
`f` toggles fullscreen, **Render → particles only** hides every helper.

## 1. Panel settings (saved in presets)

Defaults live in `params` / `inputs` in [static/viz.js](static/viz.js#L61).

| Panel | Setting | Variable | Default | Line |
|---|---|---|---|---|
| Human | distance, gesture velocity, posture confidence, detected scale, bearing | `inputs.*` (simulated human; overridden live by `go2_viz.feed`) | 5, 0, 0, 0, 0 | [viz.js:62](static/viz.js#L62) |
| Robot & canvas | head yaw / pitch, cone angle, head tracks human, entropy | `inputs.*` | 0, 0, 60°, off, 0 | [viz.js:68](static/viz.js#L68) |
| Render | base / focus particles, robot, view cone, human column, HUD | `params.show*` | all on | [viz.js:118](static/viz.js#L118) |
| Tuning → Swarm | base particles | `params.particles` | 4000 | [viz.js:97](static/viz.js#L97) |
| | focus density (per cone point, ×1–15 with velocity) | `params.focusDensity` | 0.12 | [viz.js:98](static/viz.js#L98) |
| | attraction (spring to the surface) | `params.attraction` | 0.8 | [viz.js:99](static/viz.js#L99) |
| | damping | `params.damping` | 0.95 | [viz.js:100](static/viz.js#L100) |
| | flow drift / scale / speed | `params.noise`, `noiseScale`, `flowSpeed` | 0.12, 0.5, 0.08 | [viz.js:101](static/viz.js#L101) |
| | wander (hops between points) | `params.wander` | 0.004 | [viz.js:104](static/viz.js#L104) |
| | floor share | `params.floorShare` | 0.15 | [viz.js:108](static/viz.js#L108) |
| Tuning → Look | size, opacity | `params.size`, `params.opacity` | 0.05, 0.55 | [viz.js:109](static/viz.js#L109) |
| | colour mode (outside the cone) | `params.colorMode` | gray | [viz.js:112](static/viz.js#L112) |
| | ceiling | `params.ceiling` | 1.8 m | [viz.js:111](static/viz.js#L111) |
| | cone paint (robot channel), paint colour | `params.paintMode`, `paintColor` | time of flight | [viz.js:113](static/viz.js#L113) |
| | human channel on/off | `params.humanChannel` | on | [viz.js:115](static/viz.js#L115) |
| | view range, desaturate | `params.fovRange`, `desaturate` | 4 m, 3 s | [viz.js:116](static/viz.js#L116) |
| Tuning → Markers | changes / s, outline chance, max inflation | `params.markerChange`, `outlineChance`, `markerInflate` | 0.2, 0.3, 3× | [viz.js:105](static/viz.js#L105) |
| Tuning → Camera | auto-rotate, rotate speed (+ the view itself) | `params.autoRotate`, `rotateSpeed` | on, 0.15 | [viz.js:124](static/viz.js#L124) |

## 2. Input → visual mapping (code)

The response curves of the spec, all in `deriveFromInputs` ([viz.js:83](static/viz.js#L83)):

| Mapping | Formula | Line |
|---|---|---|
| distance → jitter | 1/d², 1 at ≤ 1 m, 0 at ≥ 3 m | [viz.js:85](static/viz.js#L85) |
| distance → tearing | 0 at 1 m, 1 at 0.3 m | [viz.js:86](static/viz.js#L86) |
| velocity → focus density | 1 + 14·velocity | [viz.js:87](static/viz.js#L87) |
| entropy → dispersion, confidence → saturation, scale → marker share | 1:1 | [viz.js:88](static/viz.js#L88) |
| engagement (human-channel weight) | 1 − (1 − jitter)(1 − velocity) | [viz.js:92](static/viz.js#L92) |
| dual-stream screen blend | 1 − (1 − (1−w)·robot)(1 − w·human) | [viz.js:824](static/viz.js#L824) |
| confidence desaturation and fade from gray | | [viz.js:826](static/viz.js#L826) |
| base gray level (outside the cone) | 0.42 + 0.08·height + speed | [viz.js:816](static/viz.js#L816) |
| marker size distribution | 1 + (max − 1)·r² | [viz.js:808](static/viz.js#L808) |

## 3. Constants in the renderer

[static/viz.js](static/viz.js#L35), top of the file:

| Constant | Default | What it does | Line |
|---|---|---|---|
| `BASE_MAX`, `FOCUS_MAX` | 30 000 each | Upper limits of the two particle pools | [35](static/viz.js#L35) |
| `HOP_CELL`, `MAX_HOP` | 0.4 m, 0.5 m | Neighbourhood and longest hop between map points | [38](static/viz.js#L38) |
| `RESEED` | 0.1 | Share of base particles moved to a random spot on each new map | [39](static/viz.js#L39) |
| `FADE_IN`, `FADE_OUT` | 2 s, 0.6 s | Appear / disappear speed | [41](static/viz.js#L41) |
| `CONE_REFRESH` | 0.1 s | How often the cone's point set is recomputed | [43](static/viz.js#L43) |
| `JITTER_RANGE`, `JITTER_RATE` | 6 cm, 0.35 | Jitter amplitude and twitch rate at jitter = 1 | [44](static/viz.js#L44) |
| `TEAR_RANGE`, `TEAR_CHANCE` | 0.15–0.5 m, 0.03 | Tearing below 1 m | [46](static/viz.js#L46) |
| `MAX_DISPERSION` | 0.5 m | Aerosol distance at entropy = 1 | [48](static/viz.js#L48) |
| `SATURATE` | 0.4 s | Time to full colour inside the cone | [49](static/viz.js#L49) |
| `HEAD_OFFSET`, `CAMERA_OFFSET` | | Virtual head and front camera on the body | [50](static/viz.js#L50) |
| `VIGNETTE_EDGE` | 0.7–1.3 box half-sizes | Soft edge of the human-channel vignette | [52](static/viz.js#L52) |
| `VIGNETTE_DEPTH` | ±0.4–0.9 m | Depth window around the human | [53](static/viz.js#L53) |
| `BOX_STALE`, `HUMAN_WIDTH` | 2 s, 0.5 m | Fallback to the simulated human's box | [54](static/viz.js#L54) |
| `BACKGROUND` | `#1a1b1e` | Clear colour (also `--bg` in [index.html](static/index.html#L8)) | [128](static/viz.js#L128) |
| `REMOTE_EASE` | 0.5 s | How fast live inputs are eased in | [133](static/viz.js#L133) |
| `SCALE_MARKERS` | diamond, crosshair | Marker shapes used for the scale input | [205](static/viz.js#L205) |
| `minSize` | 2 px | Smallest drawn point | [281](static/viz.js#L281) |
| helper opacities | 0.06 cone, 0.12 human | View cone and human column | [358](static/viz.js#L358), [380](static/viz.js#L380) |
| `PALETTE`, `VIRIDIS` | | Height and time-of-flight colour ramps | [483](static/viz.js#L483) |

## 4. Live data (environment variables)

How raw robot data becomes inputs. Restart the process after changing them.

**`go2_viz.feed`** ([feed.py](feed.py#L52)):

| Variable | Default | Effect | Line |
|---|---|---|---|
| `GO2_CAMERA_HFOV`, `GO2_CAMERA_PITCH` | 120°, 0° | Bearing, box angles, camera distance estimate: **calibrate first** | [53](feed.py#L53) |
| `GO2_FEED_VEL_FULL` | 0.4 | Keypoint speed (box heights/s) for velocity 1: gesture sensitivity | [60](feed.py#L60) |
| `GO2_FEED_SCALE_FULL` | 0.25 | Box share of the frame for scale 1 | [59](feed.py#L59) |
| `GO2_FEED_ENTROPY_S` | 3 s | Entropy window: longer = calmer | [62](feed.py#L62) |
| `GO2_FEED_LOST_S` | 1 s | Person gone → neutral | [63](feed.py#L63) |
| `GO2_FEED_PERSON_H`, `GO2_FEED_TORSO` | 1.7 m, 0.5 m | Camera distance fallback | [55](feed.py#L55) |
| `GO2_FEED_LIDAR`, `…_WINDOW_S`, `…_MIN_POINTS` | on, 0.5 s, 3 | LiDAR distance | [64](feed.py#L64) |
| `FEED_VEL_NOISE`, `SMOOTH` (code) | 0.05, 0.3 | Velocity dead band and smoothing | [61](feed.py#L61), [74](feed.py#L74) |
| `LIDAR_CLUSTER_DEPTH`, `LIDAR_MIN_HEIGHT`, `LIDAR_BAND` (code) | 0.4 m, 0.5 m | What counts as the person in the scans | [67](feed.py#L67) |

**`go2_map` camera colours** ([../go2_map/\_\_main\_\_.py](../go2_map/__main__.py#L70)):

| Variable | Default | Effect | Line |
|---|---|---|---|
| `GO2_MAP_COLOR_BLUR` | 0.03 | Blur of camera frames: higher hides misalignment, lower is crisper | [72](../go2_map/__main__.py#L72) |
| `GO2_MAP_COLOR_HZ`, `GO2_MAP_COLOR_RANGE` | 3, 5 m | Colouring rate and reach | [71](../go2_map/__main__.py#L71) |
| `GO2_CAMERA_HFOV`, `GO2_CAMERA_PITCH` | 120°, 0° | Projection, same values as the feed | [74](../go2_map/__main__.py#L74) |
| `COLOR_MIN_RATE`, `COLOR_FULL_SEEN` (code) | 0.1, 3 | How fast colours settle / follow lighting, frames for full confidence | [81](../go2_map/__main__.py#L81) |
| `COLOR_MAX_TURN`, `COLOR_OCCLUSION` (code) | 0.6 rad/s, 0.25 m | Frame skipping while turning, occlusion tolerance | [78](../go2_map/__main__.py#L78) |
| `GO2_MAP_VOXEL`, `GO2_MAP_FILTER_CELL` | 5 cm, 10 cm | Map resolution and moving-object filter | |

**`go2_viz` server** ([\_\_main\_\_.py](__main__.py#L69)):

| Variable | Default | Effect | Line |
|---|---|---|---|
| `GO2_VIZ_MAX_POINTS` | 80 000 | Map points sent to the browser: more = denser, slower | [72](__main__.py#L72) |
| `GO2_VIZ_PRESET` | — | Preset file loaded at startup and written by "save to server" | [76](__main__.py#L76) |
| `GO2_VIZ_TEXTURE`, `GO2_VIZ_BLUR` | —, 0.03 | Offline human channel | [74](__main__.py#L74) |
| `NORMAL_NEIGHBOURS` (code) | 12 | Smoothness of the surface normals (jitter / dispersion direction) | [80](__main__.py#L80) |
