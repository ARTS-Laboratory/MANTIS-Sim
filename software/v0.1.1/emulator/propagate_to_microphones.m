function [cleanY, distances, delays, dopplerShift] = propagate_to_microphones( ...
    micXY, uavXYZ, t, fs, s, propOpts)
%PROPAGATE_TO_MICROPHONES Propagate a source waveform to each microphone.
%
%   [cleanY, distances, delays, dopplerShift] = PROPAGATE_TO_MICROPHONES( ...
%       micXY, uavXYZ, t, fs, s, propOpts)
%
%   Computes, for every microphone i, the noise-free received signal
%
%       y_i(t) = (1 / r_i(t)^alpha) * s(t - tau_i(t))
%
%   plus an optional ground-reflected second arrival, where r_i(t) is the
%   UAV-to-microphone distance and tau_i(t) = r_i(t)/c is the propagation
%   delay. Attenuation is evaluated at the receive-time distance, which
%   is a standard simplification for this realism level.
%
% INPUTS
%   micXY    : N-by-2 microphone coordinates (z = 0 plane assumed)
%   uavXYZ   : T-by-3 UAV positions, same T as t
%   t        : T-by-1 time vector (seconds)
%   fs       : sampling rate (Hz)
%   s        : T-by-1 source waveform (as returned by generate_source_signal)
%   propOpts : struct with fields
%       .c                 speed of sound (m/s)
%       .alpha             attenuation exponent
%       .enableDoppler     true/false
%       .enableReflection  true/false
%       .reflectionGain    relative amplitude of the reflected arrival
%
% OUTPUTS
%   cleanY       : T-by-N noise-free signal matrix (direct + optional
%                  reflection), one column per microphone
%   distances    : T-by-N UAV-to-mic distance over time (meters)
%   delays       : T-by-N propagation delay over time (seconds)
%   dopplerShift : T-by-N estimated Doppler frequency shift (Hz),
%                  reported for diagnostics regardless of enableDoppler
%
% DOPPLER HANDLING
%   Physically, a smoothly time-varying delay applied via interpolation
%   ("retarded time") already reproduces the compression/stretching of
%   the waveform that constitutes Doppler shift. To make 'enableDoppler'
%   a meaningful toggle without adding a second, redundant model:
%     - enableDoppler = true  -> delay is applied at full sample
%       resolution using continuous interpolation, so motion-induced
%       frequency shift comes through naturally.
%     - enableDoppler = false -> delay is applied as a piecewise-constant
%       ("block quantized") value, updated every ~20 ms. Timing still
%       tracks the UAV's range (important for TDOA-style downstream
%       work) but the within-block waveform is not time-warped, so no
%       continuous pitch shift is introduced.
%   In both cases a Doppler shift ESTIMATE (from radial velocity) is
%   still computed and returned in dopplerShift for diagnostic use.
%
% See also SIMULATE_UAV_ACOUSTICS, GENERATE_SOURCE_SIGNAL

t = t(:);
numSamples = numel(t);
numMics = size(micXY, 1);
c = propOpts.c;
alpha = propOpts.alpha;

distances = zeros(numSamples, numMics);
delays = zeros(numSamples, numMics);
dopplerShift = zeros(numSamples, numMics);
cleanY = zeros(numSamples, numMics);

% Reflection geometry: assume a small effective microphone capsule
% height above the ground so the reflected path is geometrically
% distinct from the direct path. This is a simplification, not a full
% acoustic reflection model.
reflMicHeight = 0.3; % meters

for i = 1:numMics
    dx = uavXYZ(:,1) - micXY(i,1);
    dy = uavXYZ(:,2) - micXY(i,2);
    dz = uavXYZ(:,3); % mic at z = 0

    r = sqrt(dx.^2 + dy.^2 + dz.^2);
    r = max(r, 1e-3); % avoid singularity if UAV passes directly over mic at z=0
    tau = r / c;

    distances(:, i) = r;
    delays(:, i) = tau;

    % --- Doppler shift estimate (always computed, for diagnostics) ---
    % Radial velocity: rate of change of range. Positive = moving away.
    if numSamples > 1
        vr = gradient(r, t);
    else
        vr = zeros(size(r));
    end
    % Approximate emitted-frequency Doppler shift using the source
    % fundamental if available; fall back to a nominal 100 Hz reference
    % tone for the estimate if not supplied upstream.
    % (This is a diagnostic estimate, not used to alter s itself.)
    fRef = 100; % nominal reference frequency for the diagnostic estimate
    dopplerShift(:, i) = -fRef * (vr / c);

    % --- Delay application ---
    if propOpts.enableDoppler
        sDelayed = apply_delay_continuous(s, t, tau);
    else
        blockLen = max(1, round(0.02 * fs)); % ~20 ms blocks
        sDelayed = apply_delay_blockwise(s, t, tau, blockLen);
    end

    atten = 1 ./ (r .^ alpha);
    yDirect = atten .* sDelayed;

    % --- Optional ground reflection (single extra arrival) ---
    if propOpts.enableReflection
        dzRefl = uavXYZ(:,3) + reflMicHeight; % image-source path
        rRefl = sqrt(dx.^2 + dy.^2 + dzRefl.^2);
        rRefl = max(rRefl, 1e-3);
        tauRefl = rRefl / c;

        if propOpts.enableDoppler
            sReflDelayed = apply_delay_continuous(s, t, tauRefl);
        else
            sReflDelayed = apply_delay_blockwise(s, t, tauRefl, blockLen);
        end

        attenRefl = 1 ./ (rRefl .^ alpha);
        yRefl = propOpts.reflectionGain * attenRefl .* sReflDelayed;
    else
        yRefl = 0;
    end

    cleanY(:, i) = yDirect + yRefl;
end

end

% -------------------------------------------------------------------
function sDelayed = apply_delay_continuous(s, t, tau)
%APPLY_DELAY_CONTINUOUS Resample s at the exact retarded time t - tau(t).
    tRetarded = t - tau;
    sDelayed = interp1(t, s, tRetarded, 'linear', 0);
end

% -------------------------------------------------------------------
function sDelayed = apply_delay_blockwise(s, t, tau, blockLen)
%APPLY_DELAY_BLOCKWISE Apply a piecewise-constant delay, updated every
%   blockLen samples, to avoid introducing continuous Doppler warping
%   while still tracking coarse range-dependent timing.
    numSamples = numel(t);
    sDelayed = zeros(numSamples, 1);
    for startIdx = 1:blockLen:numSamples
        endIdx = min(startIdx + blockLen - 1, numSamples);
        idx = startIdx:endIdx;
        blockDelay = mean(tau(idx));
        tRetarded = t(idx) - blockDelay;
        sDelayed(idx) = interp1(t, s, tRetarded, 'linear', 0);
    end
end
