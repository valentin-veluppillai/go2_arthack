# Dialing in go2_viz

Where to turn each knob, from quickest to deepest. Line numbers are as of this
commit; search for the name if they have moved.

**Workflow.** Tune in the panel, then **Presets → save to server** (with
`GO2_VIZ_PRESET=look.json`) or **save to file**. Every browser that opens the
viz then starts with that look. A preset saved in a browser wins over the
server's in that browser; **reset to defaults** clears it. `h` hides all UI,
`f` toggles fullscreen, **Render → particles only** hides every helper.

## 1. Panel settings (saved in presets)

Defaults live in `params` / `inputs` in [static/viz.js](static/viz.js#L62).

| Panel | Setting | Variable | Default | Line |
|---|---|---|---|---|
| Human | distance, gesture velocity, posture confidence, detected scale, bearing | `inputs.*` (simulated human; overridden live by `go2_viz.feed`) | 5, 0, 0, 0, 0 | [viz.js:63](static/viz.js#L63) |
| Robot & canvas | head yaw / pitch, cone angle, head tracks human, entropy | `inputs.*` | 0, 0, 60°, off, 0 | [viz.js:69](static/viz.js#L69) |
| Render | base / focus particles, robot, view cone, human column, HUD | `params.show*` | all on | [viz.js:124](static/viz.js#L124) |
| Tuning → Swarm | base particles | `params.particles` | 4000 | [viz.js:98](static/viz.js#L98) |
| | focus density (per cone point, ×1–15 with velocity) | `params.focusDensity` | 0.12 | [viz.js:99](static/viz.js#L99) |
| | attraction (spring to the surface) | `params.attraction` | 0.8 | [viz.js:100](static/viz.js#L100) |
| | damping | `params.damping` | 0.95 | [viz.js:101](static/viz.js#L101) |
| | flow drift / scale / speed | `params.noise`, `noiseScale`, `flowSpeed` | 0.12, 0.5, 0.08 | [viz.js:102](static/viz.js#L102) |
| | wander (hops between points) | `params.wander` | 0.004 | [viz.js:105](static/viz.js#L105) |
| | floor share | `params.floorShare` | 0.15 | [viz.js:109](static/viz.js#L109) |
| Tuning → Look | size, opacity | `params.size`, `params.opacity` | 0.05, 0.55 | [viz.js:110](static/viz.js#L110) |
| | colour mode (outside the cone) | `params.colorMode` | gray | [viz.js:113](static/viz.js#L113) |
| | ceiling | `params.ceiling` | 1.8 m | [viz.js:112](static/viz.js#L112) |
| | cone paint (robot channel), paint colour | `params.paintMode`, `paintColor` | time of flight | [viz.js:114](static/viz.js#L114) |
| | human channel on/off | `params.humanChannel` | on | [viz.js:116](static/viz.js#L116) |
| | view range, desaturate (cone colour decay) | `params.fovRange`, `desaturate` | 4 m, 8 s | [viz.js:117](static/viz.js#L117) |
| | cone fade (focus particles left behind by the cone) | `params.coneFade` | 3 s | [viz.js:119](static/viz.js#L119) |
| | saturation floor (in-cone saturation at posture confidence 0) | `params.saturationFloor` | 0.4 | [viz.js:120](static/viz.js#L120) |
| | saturation boost (robot and human channel) | `params.saturationBoost` | 1.5 | [viz.js:121](static/viz.js#L121) |
| | base gray (brightness outside the cone) | `params.baseGray` | 0.48 | [viz.js:122](static/viz.js#L122) |
| | background | `params.background` | `#0a0a0b` | [viz.js:123](static/viz.js#L123) |
| Tuning → Markers | changes / s, outline chance, max inflation | `params.markerChange`, `outlineChance`, `markerInflate` | 0.2, 0.3, 3× | [viz.js:106](static/viz.js#L106) |
| Tuning → Camera | auto-rotate, rotate speed (+ the view itself) | `params.autoRotate`, `rotateSpeed` | on, 0.15 | [viz.js:130](static/viz.js#L130) |

## 2. Input → visual mapping (code)

The response curves of the spec, all in `deriveFromInputs` ([viz.js:84](static/viz.js#L84)):

| Mapping | Formula | Line |
|---|---|---|
| distance → jitter | 1/d², 1 at ≤ 1 m, 0 at ≥ 3 m | [viz.js:86](static/viz.js#L86) |
| distance → tearing | 0 at 1 m, 1 at 0.3 m | [viz.js:87](static/viz.js#L87) |
| velocity → focus density | 1 + 14·velocity | [viz.js:88](static/viz.js#L88) |
| entropy → dispersion, scale → marker share (in the cone only) | 1:1 | [viz.js:89](static/viz.js#L89) |
| confidence → saturation | floor + (1 − floor)·confidence | [viz.js:90](static/viz.js#L90) |
| engagement (human-channel weight) | 1 − (1 − jitter)(1 − velocity) | [viz.js:93](static/viz.js#L93) |
| dual-stream screen blend | 1 − (1 − (1−w)·robot)(1 − w·human) | [viz.js:831](static/viz.js#L831) |
| saturation (× boost, clamped) and fade from gray | | [viz.js:834](static/viz.js#L834) |
| base gray level (outside the cone) | baseGray + 0.08·height + speed | [viz.js:823](static/viz.js#L823) |
| marker size distribution | 1 + (max − 1)·r² | [viz.js:815](static/viz.js#L815) |

## 3. Constants in the renderer

[static/viz.js](static/viz.js#L35), top of the file:

| Constant | Default | What it does | Line |
|---|---|---|---|
| `BASE_MAX`, `FOCUS_MAX` | 30 000 each | Upper limits of the two particle pools | [35](static/viz.js#L35) |
| `HOP_CELL`, `MAX_HOP` | 0.4 m, 0.5 m | Neighbourhood and longest hop between map points | [38](static/viz.js#L38) |
| `RESEED` | 0.1 | Share of base particles moved to a random spot on each new map | [39](static/viz.js#L39) |
| `FADE_IN`, `FADE_OUT` | 2 s, 0.6 s | Appear / disappear speed (focus particles left by the cone use *cone fade*) | [41](static/viz.js#L41) |
| `MARKER_PRESENCE` | 0.5 | Cone colour a particle needs to be a marker, so markers stay in the cone and fade with it | [43](static/viz.js#L43) |
| `CONE_REFRESH` | 0.1 s | How often the cone's point set is recomputed | [44](static/viz.js#L44) |
| `JITTER_RANGE`, `JITTER_RATE` | 6 cm, 0.35 | Jitter amplitude and twitch rate at jitter = 1 | [45](static/viz.js#L45) |
| `TEAR_RANGE`, `TEAR_CHANCE` | 0.15–0.5 m, 0.03 | Tearing below 1 m | [47](static/viz.js#L47) |
| `MAX_DISPERSION` | 0.5 m | Aerosol distance at entropy = 1 | [49](static/viz.js#L49) |
| `SATURATE` | 0.4 s | Time to full colour inside the cone | [50](static/viz.js#L50) |
| `HEAD_OFFSET`, `CAMERA_OFFSET` | | Virtual head and front camera on the body | [51](static/viz.js#L51) |
| `VIGNETTE_EDGE` | 0.7–1.3 box half-sizes | Soft edge of the human-channel vignette | [53](static/viz.js#L53) |
| `VIGNETTE_DEPTH` | ±0.4–0.9 m | Depth window around the human | [54](static/viz.js#L54) |
| `BOX_STALE`, `HUMAN_WIDTH` | 2 s, 0.5 m | Fallback to the simulated human's box | [55](static/viz.js#L55) |
| `--bg` | `#0a0a0b` | Page colour before the first frame; the panel's *background* overrides it | [index.html:8](static/index.html#L8) |
| `REMOTE_EASE` | 0.5 s | How fast live inputs are eased in | [138](static/viz.js#L138) |
| `SCALE_MARKERS` | diamond, crosshair | Marker shapes used for the scale input | [210](static/viz.js#L210) |
| `minSize` | 2 px | Smallest drawn point | [286](static/viz.js#L286) |
| helper opacities | 0.06 cone, 0.12 human | View cone and human column | [363](static/viz.js#L363), [385](static/viz.js#L385) |
| `PALETTE`, `VIRIDIS` | | Height and time-of-flight colour ramps | [488](static/viz.js#L488) |

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
