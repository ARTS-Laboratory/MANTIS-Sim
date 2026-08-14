%% Twenty-microphone example with reflection, colored noise, and dropout

toolboxRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(toolboxRoot);

% A 5-by-4 distributed sensor grid.
[xGrid, yGrid] = meshgrid(linspace(-100, 100, 5), linspace(-75, 75, 4));
micXY = [xGrid(:), yGrid(:)];

trajectoryParams = struct();
trajectoryParams.centerXY = [10 -5];
trajectoryParams.radiusM = 125;
trajectoryParams.revolutions = 1.25;
trajectoryParams.altitudeM = 85;
trajectoryParams.altitudeVariationM = 12;
trajectoryParams.verticalCycles = 2;
[uavXYZ, tTrajectory] = generate_uav_trajectory( ...
    'loiter', 12, 25, trajectoryParams);

settings = uav_acoustic_defaults();
settings.fs = 8000;
settings.snrDb = 2;
settings.randomSeed = 77;
settings.alpha = 1.15;
settings.enableDoppler = true;
settings.enableReflection = true;
settings.reflection.coefficient = -0.28;
settings.reflection.groundOffsetM = 0.8;
settings.source.type = 'multitone';
settings.source.frequenciesHz = [95 190 285 380 570];
settings.source.relativeAmplitudes = [1 0.75 0.48 0.30 0.16];
settings.noise.type = 'mixed';
settings.noise.commonFraction = 0.12;
settings.noise.windFraction = 0.15;
settings.dropout.mode = 'burst';
settings.dropout.rate = 0.03;
settings.dropout.burstDurationSec = [0.10 0.35];

[Y, t, meta, cleanY, distances, delays, doppler] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings);

fprintf('Generated a %d-by-%d signal matrix.\n', size(Y, 1), size(Y, 2));
fprintf('Missing-sample fractions range from %.3f to %.3f.\n', ...
    min(meta.dropout.actualFractionByChannel), ...
    max(meta.dropout.actualFractionByChannel));

plot_simulation_results(Y, t, meta, ...
    'MicrophoneIndices', [1 6 11 16 20], ...
    'MaxWaveformSeconds', 3, ...
    'ShowSpectrogram', true, ...
    'SpectrogramMic', 11);

% Uncomment to save the dataset.
% save_simulation('twenty_mic_uav_dataset.mat', Y, t, meta, ...
%     cleanY, distances, delays, doppler);
