function handles = plot_simulation_results(Y, t, meta, varargin)
%PLOT_SIMULATION_RESULTS Plot geometry, waveforms, and an optional spectrogram.
%
%   handles = PLOT_SIMULATION_RESULTS(Y, t, meta)
%   handles = PLOT_SIMULATION_RESULTS(..., 'MicrophoneIndices', [1 3], ...
%       'ShowSpectrogram', true, 'SpectrogramMic', 1)
%
% Name-value options:
%   MicrophoneIndices  Channels to show; default is the first four
%   MaxWaveformSeconds Seconds shown in waveform plots; default 2
%   ShowSpectrogram    true/false; default false
%   SpectrogramMic     Channel used for the spectrogram; default 1
%   FigureVisible      'on' or 'off'; default 'on'
%
% The spectrogram is computed with a local FFT implementation, so no Signal
% Processing Toolbox is needed.

if nargin < 3
    error('plot_simulation_results:NotEnoughInputs', ...
        'Y, t, and meta are required.');
end
if ~isnumeric(Y) || ~isreal(Y) || ndims(Y) ~= 2 || isempty(Y)
    error('plot_simulation_results:InvalidSignal', ...
        'Y must be a nonempty real numeric matrix.');
end
if ~isnumeric(t) || ~isreal(t) || ~isvector(t) || numel(t) ~= size(Y, 1) ...
        || any(~isfinite(t(:))) || any(diff(t(:)) <= 0)
    error('plot_simulation_results:InvalidTimeVector', ...
        't must be a finite increasing vector with one value per row of Y.');
end
if ~isstruct(meta) || ~isscalar(meta) || ~isfield(meta, 'micXY')
    error('plot_simulation_results:InvalidMetadata', ...
        'meta must be a scalar structure containing micXY.');
end

t = t(:);
numMics = size(Y, 2);

parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'MicrophoneIndices', [], @isnumeric);
addParameter(parser, 'MaxWaveformSeconds', 2, ...
    @(x) isnumeric(x) && isscalar(x) && isreal(x) && x > 0 && ~isnan(x));
addParameter(parser, 'ShowSpectrogram', false, @local_is_logical_scalar);
addParameter(parser, 'SpectrogramMic', 1, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x == floor(x));
addParameter(parser, 'FigureVisible', 'on', @local_is_text);
parse(parser, varargin{:});
options = parser.Results;

micIndices = options.MicrophoneIndices;
if isempty(micIndices)
    micIndices = 1:min(4, numMics);
end
micIndices = micIndices(:).';
if any(~isfinite(micIndices)) || any(micIndices ~= floor(micIndices)) ...
        || any(micIndices < 1) || any(micIndices > numMics)
    error('plot_simulation_results:InvalidMicrophoneIndices', ...
        'MicrophoneIndices must contain valid integer channel indices.');
end
micIndices = unique(micIndices, 'stable');

spectrogramMic = options.SpectrogramMic;
if spectrogramMic < 1 || spectrogramMic > numMics
    error('plot_simulation_results:InvalidSpectrogramMic', ...
        'SpectrogramMic must be a valid channel index.');
end
figureVisible = local_to_char(options.FigureVisible);
if ~any(strcmpi(figureVisible, {'on', 'off'}))
    error('plot_simulation_results:InvalidFigureVisible', ...
        'FigureVisible must be ''on'' or ''off''.');
end

micXY = meta.micXY;
if size(micXY, 1) ~= numMics || size(micXY, 2) ~= 2
    error('plot_simulation_results:GeometryMismatch', ...
        'meta.micXY must be numMics-by-2.');
end

if isfield(meta, 'uavTrajectory') ...
        && isfield(meta.uavTrajectory, 'sampledXYZ')
    pathXYZ = meta.uavTrajectory.sampledXYZ;
elseif isfield(meta, 'uavTrajectory') ...
        && isfield(meta.uavTrajectory, 'inputXYZ')
    pathXYZ = meta.uavTrajectory.inputXYZ;
else
    pathXYZ = [];
end

handles = struct();
handles.trajectoryFigure = figure('Name', 'UAV trajectory and microphones', ...
    'Visible', figureVisible);
legendEntries = {};
pathHandle = [];
if ~isempty(pathXYZ)
    pathHandle = plot3(pathXYZ(:, 1), pathXYZ(:, 2), pathXYZ(:, 3), ...
        'LineWidth', 1.3);
    legendEntries{end + 1} = 'UAV trajectory'; %#ok<AGROW>
    hold on;
end
micHandle = scatter3(micXY(:, 1), micXY(:, 2), zeros(numMics, 1), ...
    45, 'filled');
legendEntries{end + 1} = 'Microphones';
for micIndex = 1:numMics
    text(micXY(micIndex, 1), micXY(micIndex, 2), 0, ...
        sprintf('  M%d', micIndex), 'HandleVisibility', 'off');
end
hold off;
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title('UAV path and microphone layout');
if isempty(pathHandle)
    legend(micHandle, legendEntries, 'Location', 'best');
else
    legend([pathHandle micHandle], legendEntries, 'Location', 'best');
end
view(3);

handles.waveformFigure = figure('Name', 'Microphone waveforms', ...
    'Visible', figureVisible);
if isinf(options.MaxWaveformSeconds)
    plotMask = true(size(t));
else
    plotMask = t <= t(1) + options.MaxWaveformSeconds;
end
for plotIndex = 1:numel(micIndices)
    micIndex = micIndices(plotIndex);
    subplot(numel(micIndices), 1, plotIndex);
    plot(t(plotMask), Y(plotMask, micIndex));
    grid on;
    ylabel(sprintf('Mic %d', micIndex));
    if plotIndex == 1
        title('Simulated microphone signals');
    end
    if plotIndex == numel(micIndices)
        xlabel('Time (s)');
    end
end

handles.spectrogramFigure = [];
if logical(options.ShowSpectrogram)
    fs = local_get_fs(meta, t);
    x = Y(:, spectrogramMic);
    x(~isfinite(x)) = 0;
    [powerDb, frequencyHz, frameTimeSec] = local_spectrogram(x, fs, t(1));

    handles.spectrogramFigure = figure('Name', 'Microphone spectrogram', ...
        'Visible', figureVisible);
    imagesc(frameTimeSec, frequencyHz, powerDb);
    axis xy;
    xlabel('Time (s)');
    ylabel('Frequency (Hz)');
    title(sprintf('Spectrogram: microphone %d', spectrogramMic));
    colorbar;
end
end

function fs = local_get_fs(meta, t)
if isfield(meta, 'fs') && isnumeric(meta.fs) && isscalar(meta.fs) ...
        && isfinite(meta.fs) && meta.fs > 0
    fs = meta.fs;
else
    fs = 1 / median(diff(t));
end
end

function [powerDb, frequencyHz, frameTimeSec] = local_spectrogram(x, fs, startTime)
numSamples = numel(x);
desiredWindow = max(32, round(0.064 * fs));
windowLength = 2 ^ floor(log2(desiredWindow));
windowLength = min(windowLength, numSamples);
windowLength = max(2, windowLength);
hopLength = max(1, round(windowLength / 4));
numFrames = max(1, 1 + floor((numSamples - windowLength) / hopLength));
nfft = 2 ^ nextpow2(windowLength);
numBins = floor(nfft / 2) + 1;

if windowLength == 2
    window = ones(windowLength, 1);
else
    window = 0.5 - 0.5 * cos(2 * pi * (0:windowLength - 1).' / ...
        (windowLength - 1));
end

spectrum = zeros(numBins, numFrames);
frameCenters = zeros(1, numFrames);
for frameIndex = 1:numFrames
    firstSample = 1 + (frameIndex - 1) * hopLength;
    lastSample = firstSample + windowLength - 1;
    frame = x(firstSample:lastSample) .* window;
    transform = fft(frame, nfft);
    spectrum(:, frameIndex) = abs(transform(1:numBins)) .^ 2;
    frameCenters(frameIndex) = firstSample - 1 + (windowLength - 1) / 2;
end

powerDb = 10 * log10(spectrum + eps);
frequencyHz = (0:numBins - 1).' * fs / nfft;
frameTimeSec = startTime + frameCenters / fs;
end

function tf = local_is_logical_scalar(value)
tf = (islogical(value) || isnumeric(value)) && isscalar(value) ...
    && isfinite(double(value)) && (value == 0 || value == 1);
end

function tf = local_is_text(value)
tf = ischar(value) || (isstring(value) && isscalar(value));
end

function text = local_to_char(value)
if isstring(value)
    text = char(value);
else
    text = value;
end
end
