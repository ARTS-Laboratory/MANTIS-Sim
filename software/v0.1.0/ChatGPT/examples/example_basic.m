%% Basic five-microphone UAV acoustic simulation
% Run this script after adding the toolbox root folder to the MATLAB path.

toolboxRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(toolboxRoot);

% Five microphones in the z = 0 plane.
micXY = [-45 -35; 45 -35; 0 0; -45 35; 45 35];

% A straight path sampled at a modest trajectory rate. The acoustic output
% will be resampled internally at settings.fs.
trajectoryParams = struct();
trajectoryParams.startXYZ = [-140 -80 75];
trajectoryParams.endXYZ = [140 80 95];
[uavXYZ, tTrajectory] = generate_uav_trajectory( ...
    'straight', 8, 20, trajectoryParams);

settings = uav_acoustic_defaults();
settings.fs = 12000;
settings.snrDb = 8;
settings.randomSeed = 2026;
settings.enableDoppler = true;
settings.enableReflection = false;
settings.source.type = 'rotor';
settings.source.fundamentalHz = 115;
settings.source.numHarmonics = 6;
settings.source.broadbandFraction = 0.10;
settings.noise.type = 'white';

[Y, t, meta, cleanY, distances, delays, doppler] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings);

fprintf('Generated %d samples for %d microphones at %.0f Hz.\n', ...
    size(Y, 1), size(Y, 2), meta.fs);
fprintf('Distance range: %.1f to %.1f m.\n', ...
    min(distances(:)), max(distances(:)));
fprintf('Doppler ratio range: %.4f to %.4f.\n', ...
    min(doppler.ratio(:)), max(doppler.ratio(:)));

plot_simulation_results(Y, t, meta, ...
    'MicrophoneIndices', 1:5, ...
    'ShowSpectrogram', true, ...
    'SpectrogramMic', 3);

% Uncomment to save all outputs.
% save_simulation('basic_uav_acoustic_simulation.mat', Y, t, meta, ...
%     cleanY, distances, delays, doppler);
