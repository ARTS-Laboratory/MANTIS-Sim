# MATLAB UAV Acoustic Emulator

A script-friendly MATLAB toolbox for generating synthetic microphone time series from a UAV moving through three-dimensional space above a two-dimensional microphone plane.

The emulator is intended for downstream work such as time-difference-of-arrival studies, distributed sensing, feature development, classification experiments, missing-node analysis, and algorithm prototyping. It deliberately uses a moderate-complexity physical model rather than a full aeroacoustic solver.

## Main capabilities

- Any number of microphones supplied as an `N x 2` XY coordinate array.
- Arbitrary UAV trajectories supplied as a `T x 3` XYZ array and trajectory time vector.
- Built-in straight, arc, loiter, and piecewise waypoint trajectory generators.
- Tone, multitone, rotor-harmonic, broadband, and user-supplied source waveforms.
- Time-varying propagation delay and distance-dependent attenuation.
- Optional retarded-time Doppler refinement and Doppler diagnostics.
- Optional single ground-reflection arrival.
- White, pink-like, brown-like, wind-like, or mixed ambient noise.
- Per-channel target SNR with an optional absolute ambient-noise floor.
- Complete-sensor, burst, or independent-sample dropout.
- Repeatable simulation using a random seed without changing the caller's random stream.
- MAT-file saving, geometry/waveform plotting, and a toolbox self-test.
- Base MATLAB implementation; no Simulink or Signal Processing Toolbox is required.

The code is designed for MATLAB R2018b or newer.

## Install or add to the path

Unzip the folder, make it the current MATLAB folder, and run:

```matlab
install_uav_acoustic_emulator
```

To attempt to save the path permanently:

```matlab
install_uav_acoustic_emulator(true)
```

You can also use `addpath` directly.

## Quick start with defaults

```matlab
[Y, t, meta] = simulate_uav_acoustics();
plot_simulation_results(Y, t, meta);
```

This creates five microphones, a straight-line flight, a rotor-like source, inverse-distance attenuation, and white Gaussian noise.

## Typical custom simulation

```matlab
micXY = [
    -40 -30
     40 -30
      0   0
    -40  30
     40  30
];

pathParameters = struct();
pathParameters.startXYZ = [-120 -60 70];
pathParameters.endXYZ   = [ 120  60 90];
[uavXYZ, tTrajectory] = generate_uav_trajectory( ...
    'straight', 8, 20, pathParameters);

settings = uav_acoustic_defaults();
settings.fs = 12000;
settings.c = 343;
settings.alpha = 1;
settings.snrDb = 8;
settings.randomSeed = 2026;
settings.enableDoppler = true;
settings.enableReflection = true;

settings.source.type = 'rotor';
settings.source.fundamentalHz = 115;
settings.source.numHarmonics = 6;
settings.source.broadbandFraction = 0.10;

settings.noise.type = 'mixed';
settings.noise.commonFraction = 0.10;
settings.noise.windFraction = 0.15;

[Y, t, meta, cleanY, distances, delays, doppler] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings);

plot_simulation_results(Y, t, meta, ...
    'MicrophoneIndices', 1:5, ...
    'ShowSpectrogram', true, ...
    'SpectrogramMic', 3);

save_simulation('uav_dataset.mat', Y, t, meta, ...
    cleanY, distances, delays, doppler);
```

The output convention is:

```matlab
Y(:, i)       % noisy signal at microphone i
cleanY(:, i)  % propagated signal before ambient noise and dropout
```

## Main API

```matlab
[Y, tOut, meta, cleanY, distances, delays, doppler] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings)
```

### Inputs

- `micXY`: `N x 2` microphone coordinates in metres. All microphones are at `z = 0`.
- `uavXYZ`: `T x 3` UAV coordinates in metres.
- `tTrajectory`: `T x 1` strictly increasing trajectory time vector in seconds.
- `settings`: scalar structure. Start with `uav_acoustic_defaults()` and override only the fields you need.

The trajectory does not need to be sampled at the acoustic sample rate. It is interpolated internally onto a uniform output time vector at `settings.fs`.

### Outputs

- `Y`: `numSamples x numMics` signal matrix after noise and dropout.
- `tOut`: acoustic output time vector.
- `meta`: simulation settings, geometry, path, source description, noise statistics, and diagnostic summaries.
- `cleanY`: propagated channels before noise and dropout.
- `distances`: receiver-time source-to-microphone ranges in metres.
- `delays`: `distances / c` in seconds.
- `doppler`: structure with:
  - `radialVelocityMps`
  - `ratio`
  - `shiftHz`
  - `referenceFrequencyHz`
  - model description and enable flag

For broadband or user waveforms without a defined reference frequency, `doppler.shiftHz` is `NaN`; the dimensionless ratio remains available.

## Core settings

```matlab
settings.fs = 16000;               % Acoustic sample rate, Hz
settings.c = 343;                  % Speed of sound, m/s
settings.alpha = 1;                % Spreading-loss exponent
settings.snrDb = 10;               % Target SNR per microphone
settings.enableDoppler = false;
settings.enableReflection = false;
settings.randomSeed = 42;
settings.referenceDistanceM = 1;
settings.minDistanceM = 1;         % Prevents singular gain
```

The attenuation gain is

```text
(referenceDistanceM / max(range, minDistanceM))^alpha
```

Use `alpha = 1` for inverse-distance amplitude loss or `alpha = 2` for a stronger phenomenological loss law.

## Source models

### Single tone

```matlab
settings.source.type = 'tone';
settings.source.frequencyHz = 180;
settings.source.phaseRad = 0;
settings.source.amplitudeRms = 1;
```

### Tone plus broadband noise

```matlab
settings.source.type = 'tone-noise';
settings.source.frequencyHz = 180;
settings.source.broadbandFraction = 0.12;
settings.source.noiseColor = 'pink';
```

### Multitone

```matlab
settings.source.type = 'multitone';
settings.source.frequenciesHz = [100 200 300 500];
settings.source.relativeAmplitudes = [1 0.7 0.4 0.2];
settings.source.phasesRad = [0 0.2 0.4 0.6];
```

### Rotor-like harmonic source

```matlab
settings.source.type = 'rotor';
settings.source.fundamentalHz = 120;
settings.source.numHarmonics = 6;
settings.source.harmonicRolloff = 1.1;
settings.source.modulationHz = 3.5;
settings.source.modulationDepth = 0.10;
settings.source.broadbandFraction = 0.08;
settings.source.noiseColor = 'pink';
```

`broadbandFraction` is treated as an approximate power fraction in the normalized source mixture.

### Broadband signal or burst

```matlab
settings.source.type = 'broadband';
settings.source.noiseColor = 'pink';
settings.source.burstStartSec = 1.0;
settings.source.burstDurationSec = 2.5;
```

Leave both burst fields empty for continuous broadband noise.

### User-supplied waveform

```matlab
settings.source.type = 'user';
settings.source.userSignal = myWaveform;
settings.source.userFs = myWaveformSampleRate;
```

Alternatively, provide `settings.source.userTime`. User-waveform time begins at zero after interpolation. The waveform itself is omitted from stored metadata to avoid duplicating a large array; its length is recorded.

## Trajectory generation

```matlab
[uavXYZ, tTrajectory, trajectoryMeta] = ...
    generate_uav_trajectory(pathType, durationSec, trajectoryRateHz, params);
```

### Straight line

```matlab
params.startXYZ = [-100 -50 70];
params.endXYZ = [100 50 90];
[uavXYZ, tTrajectory] = generate_uav_trajectory('straight', 8, 20, params);
```

### Arc

```matlab
params.centerXY = [0 0];
params.radiusM = 100;
params.startAngleDeg = -90;
params.endAngleDeg = 120;
params.altitudeStartM = 70;
params.altitudeEndM = 90;
[uavXYZ, tTrajectory] = generate_uav_trajectory('arc', 10, 20, params);
```

### Loiter

```matlab
params.centerXY = [0 0];
params.radiusM = 80;
params.revolutions = 1.5;
params.altitudeM = 75;
params.altitudeVariationM = 8;
params.verticalCycles = 2;
[uavXYZ, tTrajectory] = generate_uav_trajectory('loiter', 12, 25, params);
```

### Piecewise waypoints

```matlab
params.waypoints = [
    -120 -60 70
     -30  20 90
      40 -10 60
     130  70 85
];
[uavXYZ, tTrajectory] = generate_uav_trajectory('piecewise', 12, 20, params);
```

Segment time is allocated in proportion to three-dimensional segment length unless `params.waypointTimes` is supplied.

## Noise model

```matlab
settings.noise.type = 'white';      % white, pink, brown, wind, mixed
settings.noise.commonFraction = 0.05;
settings.noise.windFraction = 0;
settings.noise.windCutoffHz = 30;
settings.noise.ambientRms = [];
```

`commonFraction` is the approximate noise-power fraction shared across microphones. Independent noise still differs by channel. `ambientRms`, when supplied, acts as a minimum absolute noise RMS, so very weak channels can fall below the requested SNR.

Set `settings.snrDb = Inf` for noise-free output.

## Reflection model

```matlab
settings.enableReflection = true;
settings.reflection.coefficient = -0.35;
settings.reflection.groundOffsetM = 0.5;
settings.reflection.attenuationExponent = [];
```

The optional second arrival uses an image source reflected across a plane located `groundOffsetM` below the microphone coordinate plane. This has a useful physical interpretation when microphones are mounted above the local reflecting surface while their coordinate plane remains `z = 0`.

If the reflecting plane and microphones are both exactly at `z = 0`, the direct and image paths are geometrically degenerate. The nonzero default offset intentionally prevents that unhelpful special case. An empty reflection attenuation exponent reuses `settings.alpha`.

## Doppler interpretation

A time-varying delay line,

```text
s(t - r(t)/c)
```

already produces first-order Doppler because the propagation path changes with time. This is unavoidable if the core moving-delay model is retained for arbitrary source waveforms.

- With `enableDoppler = false`, propagation uses receiver-time range in the moving delay.
- With `enableDoppler = true`, the code iteratively estimates the retarded emission time and evaluates the UAV position there. This improves timing and Doppler accuracy for faster motion.

The `doppler` output reports radial velocity, conventional source-motion frequency ratio, and the equivalent shift at the source reference frequency.

## Dropout and missing nodes

```matlab
settings.dropout.rate = 0.05;
settings.dropout.mode = 'sensor';   % sensor, burst, or sample
settings.dropout.value = NaN;
settings.dropout.burstDurationSec = [0.2 1.0];
settings.dropout.storeMask = false;
```

- `sensor`: every complete channel fails independently with probability `rate`.
- `burst`: approximately `rate` of every channel is replaced by contiguous gaps.
- `sample`: every sample is independently missing with probability `rate`.

`NaN` is the default missing value so a failed measurement is not confused with an actual acoustic zero. `cleanY` remains the ideal propagated signal before dropout.

## Saving

```matlab
save_simulation('simulation.mat', Y, t, meta, ...
    cleanY, distances, delays, doppler);
```

The resulting MAT file stores variables with those exact names. Version 7.3 is selected automatically only for very large payloads.

## Plotting

```matlab
plot_simulation_results(Y, t, meta, ...
    'MicrophoneIndices', [1 3 5], ...
    'MaxWaveformSeconds', 3, ...
    'ShowSpectrogram', true, ...
    'SpectrogramMic', 3);
```

The spectrogram uses a local Hann-window FFT implementation and does not require an additional toolbox.

## Reproducibility

All stochastic source components, ambient noise, and dropout use `settings.randomSeed`. The simulator saves the caller's random-number state before running and restores it before returning.

Two calls with identical numeric inputs and the same settings should therefore generate identical signal matrices.

## Self-test

Run:

```matlab
results = run_self_test;
```

The self-test checks:

- five-microphone dimensions and finite outputs,
- exact repeatability with the same random seed,
- delay consistency with `distance / c`,
- channel differences,
- reflection behavior,
- a 20-microphone layout,
- full sensor dropout,
- MAT-file saving and loading.

## File layout

```text
simulate_uav_acoustics.m       Main entry point
generate_uav_trajectory.m      Path generator
generate_source_signal.m       Source waveform generator
propagate_to_microphones.m     Delay, attenuation, Doppler, reflection
add_noise.m                    Ambient noise and target SNR
apply_sensor_dropout.m         Missing-node and gap simulation
save_simulation.m              MAT-file writer
plot_simulation_results.m      Geometry, waveforms, spectrogram
uav_acoustic_defaults.m        Defaults and recursive overrides
install_uav_acoustic_emulator.m
examples/example_basic.m
examples/example_20_microphones.m
tests/run_self_test.m
```

## Modeling scope and limitations

This is an emulator, not a full outdoor acoustic propagation package. It intentionally omits detailed rotor aerodynamics, atmospheric absorption, terrain, buildings, diffraction, turbulence-driven refraction, microphone directivity, clock drift, sample-rate mismatch, and calibrated sound-pressure levels.

Signal amplitudes are relative pressure units unless you calibrate `source.amplitudeRms`, the reference distance, and the absolute noise floor to measured data. The included model is most useful when geometry-dependent timing, relative amplitude, SNR, Doppler behavior, and repeatable multichannel variability matter more than exact acoustic certification.
