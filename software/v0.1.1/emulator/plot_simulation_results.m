function plot_simulation_results(Y, t, meta, varargin)
%PLOT_SIMULATION_RESULTS Visualize UAV acoustic simulation results.
%
%   PLOT_SIMULATION_RESULTS(Y, t, meta) creates a figure with:
%       (1) UAV trajectory (top-down) and microphone layout,
%       (2) sample waveforms from a few microphones,
%       (3) a spectrogram of one microphone channel (if requested).
%
% OPTIONAL NAME-VALUE ARGS
%   'micIndices'      which microphone indices to plot waveforms for
%                      (default: up to first 4 mics)
%   'showSpectrogram'  true/false (default: true)
%   'spectrogramMic'   which mic index to spectrogram (default: 1)
%
% EXAMPLE
%   [Y, t, meta] = simulate_uav_acoustics(micXY, uavXYZ, tVec);
%   plot_simulation_results(Y, t, meta);

p = inputParser;
addParameter(p, 'micIndices', []);
addParameter(p, 'showSpectrogram', true);
addParameter(p, 'spectrogramMic', 1);
parse(p, varargin{:});
opt = p.Results;

numMics = size(meta.micXY, 1);
if isempty(opt.micIndices)
    opt.micIndices = 1:min(4, numMics);
end

figure('Name', 'UAV Acoustic Simulation Results', 'Color', 'w');

% --- (1) Top-down geometry: UAV path + mic layout ---------------------
subplot(2, 2, 1);
plot(meta.uavXYZ(:,1), meta.uavXYZ(:,2), 'b-', 'LineWidth', 1.5);
hold on;
scatter(meta.micXY(:,1), meta.micXY(:,2), 60, 'r', 'filled');
for i = 1:numMics
    text(meta.micXY(i,1), meta.micXY(i,2), sprintf('  M%d', i), 'FontSize', 8);
end
plot(meta.uavXYZ(1,1), meta.uavXYZ(1,2), 'go', 'MarkerFaceColor', 'g');
plot(meta.uavXYZ(end,1), meta.uavXYZ(end,2), 'ks', 'MarkerFaceColor', 'k');
xlabel('X (m)'); ylabel('Y (m)');
title('UAV Trajectory (top-down) and Microphone Layout');
legend({'UAV path', 'Microphones', '', 'Start', 'End'}, 'Location', 'best');
axis equal; grid on;
hold off;

% --- (2) Altitude profile ----------------------------------------------
subplot(2, 2, 2);
plot(t, meta.uavXYZ(:,3), 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('UAV Altitude Over Time');
grid on;

% --- (3) Sample waveforms -----------------------------------------------
subplot(2, 2, 3);
hold on;
for i = opt.micIndices
    plot(t, Y(:, i), 'DisplayName', sprintf('Mic %d', i));
end
xlabel('Time (s)'); ylabel('Amplitude');
title('Sample Microphone Waveforms');
legend show;
grid on;
hold off;

% --- (4) Spectrogram ------------------------------------------------
subplot(2, 2, 4);
if opt.showSpectrogram
    micIdx = opt.spectrogramMic;
    sig = Y(:, micIdx);
    windowLen = min(1024, floor(numel(sig)/8));
    windowLen = max(windowLen, 32);
    noverlap = round(windowLen * 0.5);
    nfft = max(256, 2^nextpow2(windowLen));
    if exist('spectrogram', 'file') == 2 || exist('spectrogram', 'builtin')
        spectrogram(sig, windowLen, noverlap, nfft, meta.fs, 'yaxis');
        title(sprintf('Spectrogram (Mic %d)', micIdx));
    else
        text(0.5, 0.5, 'Signal Processing Toolbox not available', ...
            'HorizontalAlignment', 'center');
        axis off;
    end
else
    axis off;
end

sgtitle('UAV Acoustic Emulator - Simulation Results');

end
