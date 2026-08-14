function Y = add_noise(cleanY, noiseOpts)
%ADD_NOISE Add ambient noise to each microphone channel at a target SNR.
%
%   Y = ADD_NOISE(cleanY, noiseOpts) adds noise independently to each
%   column (microphone channel) of cleanY, scaled so the resulting
%   per-channel signal-to-noise ratio matches noiseOpts.snrDb.
%
% INPUTS
%   cleanY    : numSamples-by-numMics noise-free signal matrix
%   noiseOpts : struct with fields
%       .snrDb  desired SNR in dB (scalar)
%       .type   'white' | 'pink' | 'wind'
%
% OUTPUT
%   Y : numSamples-by-numMics noisy signal matrix
%
% NOTES
%   - 'white' : flat-spectrum Gaussian noise.
%   - 'pink'  : approximate 1/f noise via simple IIR shaping of white
%               noise (Voss-McCartney-style single-pole approximation).
%   - 'wind'  : low-frequency-heavy noise intended to emulate gusting
%               wind, built from heavily low-pass-filtered white noise.
%   - If a channel is silent (all zeros, e.g. a dropped-out microphone
%     upstream), noise power is scaled from a small numerical floor
%     rather than the (zero) signal power, avoiding divide-by-zero.

[numSamples, numMics] = size(cleanY);
Y = zeros(numSamples, numMics);

for i = 1:numMics
    sig = cleanY(:, i);
    sigPower = mean(sig.^2);
    if sigPower <= 0
        sigPower = eps;
    end

    switch lower(noiseOpts.type)
        case 'white'
            noise = randn(numSamples, 1);

        case 'pink'
            noise = pink_noise(numSamples);

        case 'wind'
            noise = wind_noise(numSamples);

        otherwise
            error('add_noise:badType', 'Unknown noiseOpts.type "%s".', noiseOpts.type);
    end

    noisePower = mean(noise.^2);
    if noisePower <= 0
        noisePower = eps;
    end

    % Scale noise so that 10*log10(sigPower / scaledNoisePower) == snrDb
    targetNoisePower = sigPower / (10 ^ (noiseOpts.snrDb / 10));
    scale = sqrt(targetNoisePower / noisePower);

    Y(:, i) = sig + scale * noise;
end

end

% -------------------------------------------------------------------
function noise = pink_noise(numSamples)
%PINK_NOISE Approximate 1/f noise using a simple first-order IIR filter
%   applied to white Gaussian noise. Not a rigorous fractal generator,
%   but adequate for adding low-frequency-emphasized ambient texture.
    white = randn(numSamples, 1);
    b = [0.049922035 -0.095993537 0.050612699 -0.004408786];
    a = [1 -2.494956002 2.017265875 -0.522189400];
    noise = filter(b, a, white);
    noise = noise / (std(noise) + eps);
end

% -------------------------------------------------------------------
function noise = wind_noise(numSamples)
%WIND_NOISE Low-frequency-heavy noise emulating gusting wind, built by
%   strongly low-pass filtering white noise with a simple moving-average
%   cascade.
    white = randn(numSamples, 1);
    winLen = max(5, round(numSamples / 200));
    kernel = ones(winLen, 1) / winLen;
    noise = filter(kernel, 1, white);
    noise = filter(kernel, 1, noise); % cascade for a steeper rolloff
    noise = noise / (std(noise) + eps);
end
