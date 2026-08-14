function settings = uav_acoustic_defaults(overrides)
%UAV_ACOUSTIC_DEFAULTS Return default settings for the UAV acoustic emulator.
%
%   settings = UAV_ACOUSTIC_DEFAULTS()
%   settings = UAV_ACOUSTIC_DEFAULTS(overrides)
%
% The optional OVERRIDES structure is merged recursively into the defaults.
% Unknown fields are retained so users can attach application-specific
% metadata without modifying the simulator.

settings = struct();

% Core simulation settings.
settings.fs = 16000;
settings.c = 343;
settings.alpha = 1;
settings.snrDb = 10;
settings.enableDoppler = false;
settings.enableReflection = false;
settings.randomSeed = 42;
settings.trajectoryInterpolation = 'linear';
settings.sourceInterpolation = 'linear';
settings.referenceDistanceM = 1;
settings.minDistanceM = 1;
settings.retardedTimeIterations = 2;
settings.minDopplerRatio = 0.5;
settings.maxDopplerRatio = 2.0;

% Defaults used only when geometry or a trajectory is omitted.
settings.defaultDurationSec = 8;
settings.defaultTrajectoryRateHz = 20;

% Source model. amplitudeRms is in arbitrary source-pressure units at the
% reference distance before propagation loss is applied.
settings.source = struct();
settings.source.type = 'rotor';
settings.source.amplitudeRms = 1;
settings.source.frequencyHz = 180;
settings.source.phaseRad = 0;
settings.source.frequenciesHz = [120 240 360 480];
settings.source.relativeAmplitudes = [1 0.65 0.40 0.25];
settings.source.phasesRad = [];
settings.source.fundamentalHz = 120;
settings.source.numHarmonics = 6;
settings.source.harmonicRolloff = 1.1;
settings.source.modulationHz = 3.5;
settings.source.modulationDepth = 0.10;
settings.source.broadbandFraction = 0.08;
settings.source.noiseColor = 'pink';
settings.source.burstStartSec = [];
settings.source.burstDurationSec = [];
settings.source.userSignal = [];
settings.source.userFs = [];
settings.source.userTime = [];
settings.source.removeMean = true;

% Ambient noise model.
settings.noise = struct();
settings.noise.type = 'white';
settings.noise.ambientRms = [];
settings.noise.commonFraction = 0.05;
settings.noise.windFraction = 0;
settings.noise.windCutoffHz = 30;
settings.noise.removeMean = true;

% Simple image-source-like reflection. The effective ground is located
% groundOffsetM below the microphone coordinate plane. This avoids the
% degenerate two-ray case that occurs when both ground and microphones are
% exactly at z = 0.
settings.reflection = struct();
settings.reflection.coefficient = -0.35;
settings.reflection.groundOffsetM = 0.5;
settings.reflection.attenuationExponent = [];

% Missing-node and dropout model.
settings.dropout = struct();
settings.dropout.rate = 0;
settings.dropout.mode = 'sensor';       % 'sensor', 'burst', or 'sample'
settings.dropout.value = NaN;
settings.dropout.burstDurationSec = [0.20 1.00];
settings.dropout.storeMask = false;

% Metadata and diagnostic controls.
settings.metadata = struct();
settings.metadata.storeSampledTrajectory = true;
settings.metadata.storeSettings = true;

if nargin >= 1 && ~isempty(overrides)
    if ~isstruct(overrides) || ~isscalar(overrides)
        error('uav_acoustic_defaults:InvalidOverrides', ...
            'Overrides must be a scalar structure.');
    end
    settings = local_merge_struct(settings, overrides);
end

% A few convenient aliases keep scripts readable while preserving one
% canonical representation inside the simulator.
if isfield(settings, 'sourceType') && ~isempty(settings.sourceType)
    settings.source.type = settings.sourceType;
end
if isfield(settings, 'backgroundNoiseType') && ~isempty(settings.backgroundNoiseType)
    settings.noise.type = settings.backgroundNoiseType;
end
if isfield(settings, 'dropoutRate') && ~isempty(settings.dropoutRate)
    settings.dropout.rate = settings.dropoutRate;
end
end

function out = local_merge_struct(base, override)
out = base;
fields = fieldnames(override);
for k = 1:numel(fields)
    name = fields{k};
    value = override.(name);
    if isfield(out, name) && isstruct(out.(name)) && isscalar(out.(name)) ...
            && isstruct(value) && isscalar(value)
        out.(name) = local_merge_struct(out.(name), value);
    else
        out.(name) = value;
    end
end
end
