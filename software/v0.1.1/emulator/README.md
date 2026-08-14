# MATLAB UAV Acoustic + Vision Emulator (v0.2)

A modular MATLAB toolbox that simulates a network of ground sensor
nodes observing a UAV in flight. Each node carries **a microphone at
ground level and an upward-looking fisheye camera**, and both streams
are generated from the same trajectory on the same time base.

It is a **simulator**, not a detection algorithm. The goal is
physically plausible synthetic data for downstream work: TDOA
localization, vision-based tracking, audio/vision fusion, and
resilience studies on self-arranging sensor networks where nodes can
drop out.

Realism target is unchanged from v0.1: about 80 percent. Signals change
with range, channels differ from each other, motion affects timing and
apparent position, and noise can obscure the target, without
over-engineering either the acoustics or the optics.

## What changed from v0.1

**The acoustic path is untouched.** `simulate_uav_acoustics.m` and
everything under it are byte-identical to v0.1. The same model, the
same parameters, the same outputs. Audio still runs at the full
sampling rate you set.

**A vision path was added alongside it.** Every node now also renders
low-rate camera frames, snapped onto the audio sample grid so the two
streams need no resampling to fuse.

**Node dropout became coherent.** In v0.1 dropout applied to
microphones only. In v0.2 a node is a package: `'deadNodes'` takes out
its microphone and its camera together, which is the failure mode the
network is supposed to survive. `'deadMics'` and `'deadCameras'` are
still available for one-sided failures.

**Two airframes are modeled.** The SWIFT-UAV fixed wing is the default;
the F550-class hexacopter is available with `'platform', 'f550'`.

## Files

### New in v0.2

| File | Purpose |
|---|---|
| `simulate_uav_scene.m` | Main entry point. Runs both channels, handles node dropout coherently, packages one scene struct. |
| `simulate_uav_cameras.m` | Renders synchronized fisheye frames at every node, with per-frame ground truth. |
| `uav_platform.m` | Airframe geometry, appearance, nav lights, and suggested acoustics for the SWIFT fixed wing and the F550 hexacopter. |
| `make_fisheye_camera.m` | Camera intrinsics and precomputed per-pixel angle maps. |
| `fisheye_project.m` | World points to pixel coordinates through the lens. |
| `render_sky_background.m` | Procedural sky: day, cloudy, dusk, night. Drifting cloud, sun and glare, stars. |
| `render_uav_projection.m` | Rasterizes the airframe polygons into the frame and reports the ground-truth box. |
| `add_image_noise.m` | Shot noise, read noise, analog gain, 8-bit quantization. |
| `estimate_uav_attitude.m` | Derives yaw, pitch, and bank from the trajectory so the silhouette is oriented sensibly. |
| `generate_node_array.m` | Node layouts: ring, grid, line, or random drop inside a disc. |
| `plot_scene_results.m` | Scene overview plus a synchronized multi-camera montage with boxes. |
| `save_scene.m` | Saves a scene, optionally splitting the frame array into a companion file. |
| `export_yolo_dataset.m` | Writes PNGs and exact YOLO-format labels for detector training. |
| `pixel_range_budget.m` | Turns camera resolution and FOV into a detection range budget. |
| `demo_uav_scene.m` | Five worked examples. |

### Carried over unchanged from v0.1

`simulate_uav_acoustics.m`, `generate_uav_trajectory.m`,
`generate_source_signal.m`, `propagate_to_microphones.m`,
`add_noise.m`, `save_simulation.m`, `plot_simulation_results.m`,
`demo_uav_acoustics.m`

The v0.1 demo still runs on its own if you only want audio.

## Quick start

```matlab
addpath('/path/to/uav-emulator-v0.2');

fs = 4000;
t  = (0:1/fs:24)';

nodeXY = generate_node_array(6, 'ring', 'radius', 50);   % 6 packages

uavXYZ = generate_uav_trajectory(t, 'straight', ...
    'startXY', [-300 -40], 'endXY', [300 40], 'altitude', 80);

scene = simulate_uav_scene(nodeXY, uavXYZ, t, ...
    'platform', 'swift', ...   % or 'f550'
    'sky',      'cloudy', ...  % day | cloudy | dusk | night
    'fps',      1, ...
    'imageSize',[240 240], ...
    'snrDb',    10);

plot_scene_results(scene);
save_scene('run_001.mat', scene, 'splitVideo', true);
```

## The scene struct

```
scene.audio.Y            numSamples x numNodes microphone signals
scene.audio.t, .fs       audio time base
scene.audio.meta         v0.1 acoustic metadata
scene.audio.extras       cleanY, distances, delays, doppler estimate

scene.video.frames       H x W x C x numFrames x numNodes, uint8
scene.video.meta         cameras, frame times, sky, sensor model, truth

scene.truth.uavXYZ       trajectory
scene.truth.attitude     yaw, pitch, roll actually rendered
scene.truth.rangeToNodes slant range per node, audio rate
scene.nodes              node coordinates and which are dead
```

`scene.video.frames(:,:,:,k,n)` is frame `k` from node `n`.

### Ground truth for the vision channel

`scene.video.meta.truth` holds, for every frame and node:

| Field | Meaning |
|---|---|
| `u`, `v` | pixel coordinates of the airframe origin |
| `range` | slant range from node to UAV, meters |
| `bbox` | `[uMin vMin width height]` in pixels |
| `pixelArea` | composited target coverage in pixels |
| `angularSize` | apparent angular size of the airframe, degrees |
| `inFOV` | target inside the field of view |

Boxes come from the renderer, so they are exact rather than
hand-labeled. `export_yolo_dataset.m` writes them straight out in YOLO
format with negatives included.

## Time alignment

Audio runs at `fs`. Frames are placed **on the audio sample grid**, so
frame `k` at every node corresponds exactly to audio sample
`scene.video.meta.frameIdx(k)` at time
`scene.video.meta.frameTimes(k)`. All nodes are triggered on the same
instants. No interpolation is needed to line the streams up.

```matlab
k   = 10;
idx = scene.video.meta.frameIdx(k);        % audio sample index
win = idx + (-fs/2 : fs/2);                % one second of audio
clip = scene.audio.Y(win, :);              % around frame k
```

## Camera model

The camera looks straight up from `camHeight` (0.15 m by default). For
a point at zenith angle `theta` and azimuth `phi`:

```
r_pix = f * theta                (equidistant, the default)
u     = cx + r_pix*cos(phi)
v     = cy - r_pix*sin(phi)
```

`equisolid` (`r = 2f sin(theta/2)`) and `orthographic`
(`r = f sin(theta)`) are also available. The focal length is set so the
FOV edge lands on the image circle, so the whole hemisphere fits inside
a circle inscribed in the frame and the corners stay black, as on a
real circular fisheye.

Apparent size is not scripted: the airframe polygons are projected
vertex by vertex, so range, aspect, and lens curvature all fall out of
the geometry. Polygon edges are subdivided before projection, so a long
straight member such as a wing spar bends the way the lens actually
bends it rather than being drawn as a chord.

## Angular resolution, and why it dominates

A 185 degree fisheye spreads the whole sky across the sensor. At
240x240 that is 0.77 degrees, or 13.5 mrad, per pixel. In daylight the
limit on detection range is therefore angular resolution, not signal
level. Full span of the airframe, equidistant lens, 185 degree FOV:

| Sensor | SWIFT, 1 px | SWIFT, 5 px | SWIFT, 10 px | F550, 1 px | F550, 5 px |
|---|---|---|---|---|---|
| 240 x 240 | 193 m | 39 m | 19 m | 60 m | 12 m |
| 480 x 480 | 386 m | 77 m | 39 m | 120 m | 24 m |
| 960 x 960 | 773 m | 155 m | 77 m | 240 m | 48 m |

A detector needs well more than one pixel of extent: roughly 3 to 5 px
for a matched-filter or track-before-detect approach on a point-like
target, and more like 10 to 20 px before a learned detector such as
YOLO is reliable.

The practical consequence: a fixed wing crossing at 80 m altitude over
a 50 m ring peaks at about **1.1 px** on a 240x240 node. That is a
point target. Either spend more pixels, narrow the FOV, or treat the
visual channel as a bearing-only point detector and let the microphones
carry range. Run `pixel_range_budget` before committing to hardware.

## Sky models

| Type | Character | Notes |
|---|---|---|
| `day` | Clear, bright, sun disc and halo | Highest contrast on a dark airframe. Static, so it is cached and renders fast. |
| `cloudy` | Broken cloud, drifting | Puts high-contrast edges of roughly target size all over the frame. The realistic false-alarm case. |
| `dusk` | Low sun, warm gradient | Moderate gain, nav lights on by default. |
| `night` | Dark sky, stars | The silhouette disappears entirely. Only nav and strobe lights remain. |

Cloud is a three-octave value-noise deck at a settable height, sampled
along each pixel ray, so it shows correct perspective and piles up
toward the horizon. Beyond about 60 degrees off zenith it fades into
uniform haze, which is both physically reasonable and avoids aliasing
at the rim.

Nav lights are on by default for dusk and night: wingtip lights plus a
1 Hz belly strobe on the fixed wing, motor-pod LEDs on the hexacopter.
Because the strobe is sampled at 1 fps, it is sometimes caught mid-flash
and sometimes missed, which is the honest behavior.

## Airframes

| | SWIFT-UAV | F550-class hex |
|---|---|---|
| Name | `'swift'` | `'f550'` |
| Span or tip-to-tip | 2.6 m | 0.81 m |
| Key geometry | 0.33 m chord, twin boom, 21 in tractor prop | 550 mm wheelbase, six 10 in props |
| Suggested source | rotor-like, 130 Hz | rotor-like, 170 Hz |
| YOLO class id | 0 | 1 |

Dimensions are first-order representations, enough to set apparent
angular size and rough silhouette shape. They are not CAD outer mold
lines. The tractor propeller sits in a vertical plane, so from directly
below it correctly projects nearly edge-on, while the hexacopter's
rotor discs are horizontal and dominate its silhouette.

The suggested acoustics are **defaults only**. Passing `'sourceType'`
or `'sourceFreq'` explicitly overrides them, and the acoustic model
itself is unchanged either way.

## Node dropout and network resilience

```matlab
scene = simulate_uav_scene(nodeXY, uavXYZ, t, ...
    'deadNodes',   4, ...        % node 4 entirely offline
    'deadMics',    2, ...        % node 2 keeps its camera only
    'deadCameras', [5 6], ...    % nodes 5 and 6 keep audio only
    'dropoutRate', 0.1);         % plus random failures on top
```

Dead microphones flatline, matching the v0.1 convention so dropout is
unambiguous downstream. Dead cameras return all-zero frames. Explicit
indices are deterministic, which is what ablation studies want:
re-run the same scene with each node removed in turn and see when the
network stops tracking.

## Sensor model

Radiance is converted to counts through a first-order photon transfer
model rather than a fixed SNR, so noise follows the scene:

```
electrons  = radiance * fullWell
electrons += sqrt(electrons) * N(0,1)      shot noise
electrons += readNoise * N(0,1)            read noise
DN         = gain * electrons * 255/fullWell + blackLevel
```

A bright day frame is nearly noise free. A night frame pushed up with
analog gain (12x by default) is visibly grainy. That is exactly the
regime where vision stops carrying the detection and the microphones
have to, which is the point of building the two channels together.

Motion blur is available: set `'subFrames'` above 1 and the renders are
averaged across `'exposureTime'`.

## Performance

At 240x240 with a static sky, roughly 30 ms per frame per node on a
laptop. Six nodes and 25 frames is about 8 seconds. Cost scales with
pixel count, with `'subFrames'`, and with drifting cloud (which forces
a background re-render each frame). The sky is cached automatically
whenever nothing in it moves.

Frame memory is `H*W*C*numFrames*numNodes` bytes: 240x240 mono, 25
frames, 6 nodes is about 8.6 MB. Use `save_scene(..., 'splitVideo',
true)` to keep the main file small when only audio and truth are
needed.

## Validation

`simulate_uav_scene` and `simulate_uav_cameras` check node and UAV
coordinates for finiteness, trajectory and time lengths for agreement,
time for strict monotonicity, frame rate and channel count for
validity, and dropout specifications for range, all before any
rendering starts. The v0.1 acoustic validation runs unchanged.

## Notes and known simplifications

- No toolboxes are required. `plot_simulation_results.m` still skips
  the spectrogram panel gracefully without Signal Processing Toolbox.
- The lens model is ideal: no calibrated radial distortion, no
  decentering, no chromatic effects. Add a calibration layer on top if
  you need to match a specific Tamron fisheye.
- Faces that straddle the FOV rim are dropped rather than clipped. With
  the default 185 degree FOV and a camera below the aircraft this
  effectively never fires.
- No occlusion, no ground clutter, no horizon or structure in frame.
  Every camera sees clean sky.
- Atmospheric extinction and haze contrast reduction with range are not
  modeled. At the ranges of interest here this is a small effect
  compared with the angular resolution limit.
- Attitude is kinematic, not a 6-DOF flight model: the airframe flies
  along its velocity vector and banks into turns.
- As in v0.1, the toolbox generates channel data only. It implements no
  detection, localization, classification, or fusion.
