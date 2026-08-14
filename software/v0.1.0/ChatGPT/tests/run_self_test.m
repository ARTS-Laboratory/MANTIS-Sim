function results = run_self_test()
%RUN_SELF_TEST Exercise the main simulator, optional effects, and saving.
%
%   results = RUN_SELF_TEST()
%
% This test intentionally uses short, low-rate simulations so it can run
% quickly. It requires only base MATLAB functionality.

rootFolder = fileparts(fileparts(mfilename('fullpath')));
addpath(rootFolder);

fprintf('Running MATLAB UAV Acoustic Emulator self-test...\n');

micXY = [-25 -20; 25 -20; 0 0; -25 20; 25 20];
trajectoryParams = struct('startXYZ', [-30 -15 45], ...
    'endXYZ', [30 15 55]);
[uavXYZ, tTrajectory] = generate_uav_trajectory( ...
    'straight', 1.2, 20, trajectoryParams);

settings = uav_acoustic_defaults();
settings.fs = 2000;
settings.snrDb = 6;
settings.randomSeed = 1234;
settings.enableDoppler = true;
settings.source.type = 'rotor';
settings.source.fundamentalHz = 80;
settings.source.numHarmonics = 5;
settings.source.broadbandFraction = 0.05;

[Y1, t1, meta1, clean1, distance1, delay1, doppler1] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings);
[Y2, t2] = simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings);

expectedSamples = floor((tTrajectory(end) - tTrajectory(1)) * settings.fs) + 1;
assert(isequal(size(Y1), [expectedSamples size(micXY, 1)]), ...
    'Unexpected signal dimensions.');
assert(isequal(t1, t2) && isequal(Y1, Y2), ...
    'Runs with the same seed are not exactly repeatable.');
assert(all(isfinite(Y1(:))) && all(isfinite(clean1(:))), ...
    'Unexpected nonfinite values without dropout.');
assert(max(abs(delay1(:) - distance1(:) / settings.c)) < 1e-12, ...
    'Delay is inconsistent with distance/c.');
assert(norm(Y1(:, 1) - Y1(:, 2)) > 0, ...
    'Different microphones unexpectedly produced identical channels.');
assert(isequal(size(doppler1.ratio), size(Y1)), ...
    'Doppler diagnostics have inconsistent dimensions.');
assert(meta1.numMics == 5 && meta1.numSamples == expectedSamples, ...
    'Metadata dimensions are inconsistent.');

% Reflection must change the clean signal when all other randomness is held.
noiseless = settings;
noiseless.snrDb = Inf;
noiseless.enableReflection = false;
[~, ~, ~, cleanNoReflection] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, noiseless);
noiseless.enableReflection = true;
[~, ~, ~, cleanReflection] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, noiseless);
assert(norm(cleanNoReflection(:) - cleanReflection(:)) > 0, ...
    'Enabling reflection did not change the clean signal.');

% Confirm arbitrary microphone counts with a 20-node grid.
[xGrid, yGrid] = meshgrid(linspace(-30, 30, 5), linspace(-20, 20, 4));
mic20 = [xGrid(:), yGrid(:)];
shortSettings = settings;
shortSettings.fs = 1200;
shortSettings.source.fundamentalHz = 60;
shortSettings.source.numHarmonics = 4;
[uavShort, tShort] = generate_uav_trajectory('arc', 0.8, 15, ...
    struct('centerXY', [0 0], 'radiusM', 8, 'altitudeM', 40));
[Y20, t20] = simulate_uav_acoustics(mic20, uavShort, tShort, shortSettings);
assert(size(Y20, 2) == 20 && size(Y20, 1) == numel(t20), ...
    'The 20-microphone test failed.');

% A sensor dropout rate of one must mark every output sample missing.
dropSettings = shortSettings;
dropSettings.snrDb = Inf;
dropSettings.dropout.mode = 'sensor';
dropSettings.dropout.rate = 1;
dropSettings.dropout.value = NaN;
[Ydrop, ~, metaDrop] = ...
    simulate_uav_acoustics(mic20, uavShort, tShort, dropSettings);
assert(all(isnan(Ydrop(:))), 'Full sensor dropout did not produce all NaNs.');
assert(numel(metaDrop.dropout.failedChannels) == 20, ...
    'Dropout metadata did not identify all failed channels.');

% Verify that the save helper writes loadable variables.
testFile = [tempname '.mat'];
save_simulation(testFile, Y1, t1, meta1, clean1, distance1, delay1, doppler1);
loaded = load(testFile);
assert(isfield(loaded, 'Y') && isfield(loaded, 'meta') ...
    && isequal(size(loaded.Y), size(Y1)), ...
    'Saved MAT file is missing expected variables.');
delete(testFile);

results = struct();
results.passed = true;
results.basicSignalSize = size(Y1);
results.twentyMicSignalSize = size(Y20);
results.message = 'All self-tests passed.';

fprintf('%s\n', results.message);
end
