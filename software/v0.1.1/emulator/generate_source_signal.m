function s = generate_source_signal(t, fs, sourceOpts)
%GENERATE_SOURCE_SIGNAL Create the UAV acoustic source waveform s(t).
%
%   s = GENERATE_SOURCE_SIGNAL(t, fs, sourceOpts) returns a column vector
%   the same length as t, representing the acoustic emission "at the
%   source" before propagation, delay, attenuation, or noise are applied.
%
% INPUTS
%   t          : time vector (seconds), column or row.
%   fs         : sampling rate (Hz). Included for API symmetry / future
%                use (e.g. filter design); not required by every branch.
%   sourceOpts : struct with fields:
%       .type            'tone' | 'multitone' | 'rotor' | 'noiseburst' | 'custom'
%       .freq            fundamental frequency in Hz (default handled by caller)
%       .customWaveform  vector to use directly when type == 'custom'
%
% OUTPUT
%   s : numSamples-by-1 source waveform, roughly unit amplitude.
%
% NOTES ON SOURCE TYPES
%   'tone'       : single sinusoid at sourceOpts.freq.
%   'multitone'  : fundamental plus 2nd and 3rd harmonics with decaying
%                  amplitude, loosely mimicking a mechanical hum.
%   'rotor'      : fundamental "blade-pass" tone plus several harmonics
%                  with amplitude modulation, approximating the buzzy,
%                  harmonically-rich sound of UAV rotors.
%   'noiseburst' : broadband Gaussian noise shaped by a smooth envelope,
%                  useful for testing detection under low-tonality
%                  conditions.
%   'custom'     : user-supplied waveform, resampled/truncated/padded
%                  to match numel(t).

t = t(:);
numSamples = numel(t);
f0 = sourceOpts.freq;

switch lower(sourceOpts.type)

    case 'tone'
        s = sin(2*pi*f0*t);

    case 'multitone'
        s = sin(2*pi*f0*t) ...
          + 0.5*sin(2*pi*2*f0*t) ...
          + 0.25*sin(2*pi*3*f0*t);
        s = s / max(abs(s));

    case 'rotor'
        % Rotor-like harmonic stack: fundamental "blade-pass" frequency
        % plus several harmonics with decaying amplitude, modulated by a
        % slow, slightly irregular envelope to emulate motor/blade
        % fluctuation. Kept deterministic given a fixed rng seed
        % upstream in simulate_uav_acoustics.
        numHarmonics = 5;
        s = zeros(numSamples, 1);
        for k = 1:numHarmonics
            amp = 1 / k; % decaying harmonic amplitude
            s = s + amp * sin(2*pi*k*f0*t + 0.1*k);
        end
        % Slow amplitude modulation (~1-3 Hz) to emulate blade flutter.
        modFreq = 1.5;
        envelope = 1 + 0.15*sin(2*pi*modFreq*t);
        s = s .* envelope;
        s = s / max(abs(s));

    case 'noiseburst'
        raw = randn(numSamples, 1);
        % Smooth envelope via simple moving-average to avoid a harsh,
        % click-like broadband signal.
        winLen = max(3, round(0.01 * fs)); % ~10 ms smoothing window
        kernel = ones(winLen, 1) / winLen;
        envelope = filter(kernel, 1, abs(raw));
        envelope = envelope / max(envelope + eps);
        s = raw .* envelope;
        s = s / max(abs(s) + eps);

    case 'custom'
        w = sourceOpts.customWaveform;
        if isempty(w)
            error('generate_source_signal:missingCustomWaveform', ...
                'sourceOpts.customWaveform must be supplied when type = "custom".');
        end
        w = w(:);
        if numel(w) == numSamples
            s = w;
        elseif numel(w) > numSamples
            s = w(1:numSamples);
        else
            % Pad by tiling the waveform to cover the full duration.
            reps = ceil(numSamples / numel(w));
            wTiled = repmat(w, reps, 1);
            s = wTiled(1:numSamples);
        end
        maxAbs = max(abs(s));
        if maxAbs > 0
            s = s / maxAbs;
        end

    otherwise
        error('generate_source_signal:badType', ...
            'Unknown sourceOpts.type "%s".', sourceOpts.type);
end

end
