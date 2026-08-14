function [bg, sky] = render_sky_background(cam, sky, tNow)
%RENDER_SKY_BACKGROUND Procedural upward-looking sky for a fisheye camera.
%
%   [bg, sky] = RENDER_SKY_BACKGROUND(cam, sky, tNow) returns an
%   H-by-W-by-C image of the sky as seen by cam at time tNow, in
%   normalized scene radiance (0 = black, 1 = full-scale). The returned
%   sky struct carries cached fields (cloud noise, star field, angle
%   maps) so it should be passed back in on the next call.
%
% SKY OPTIONS (fields of the input struct, all optional)
%   .type          'day' (default) | 'cloudy' | 'dusk' | 'night'
%   .channels      1 for monochrome (default) or 3 for RGB
%   .zenithLevel   radiance straight up (default set per type)
%   .horizonLevel  radiance at the FOV edge (default set per type)
%   .cloudCover    fraction of sky covered, 0..1 (default 0.55 cloudy)
%   .cloudHeight   cloud base height in meters (default 900)
%   .cloudScale    horizontal feature size in meters (default 600)
%   .cloudDrift    [vx vy] cloud drift in m/s (default [8 3])
%   .cloudSeed     seed for the cloud field; share it across cameras so
%                  every node sees the same sky (default 11)
%   .sunAzElDeg    [az el] sun direction, az CCW from world +x, el above
%                  the horizon. Empty disables the sun (default [50 55]
%                  for day/cloudy, [] otherwise)
%   .sunGain       peak brightness of the solar disc (default 3.0)
%   .starCount     number of stars for 'night' (default 220)
%   .seed          seed for stars and static texture (default 5)
%
% WHY THIS MATTERS
%   The sky is the entire background against which a small dark target
%   must be found, so its statistics set the detection problem. Bright
%   uniform day sky is the easy case; broken cloud puts high-contrast
%   edges of roughly target size all over the frame; night removes the
%   silhouette entirely and leaves only the nav lights.
%
% See also MAKE_FISHEYE_CAMERA, SIMULATE_UAV_CAMERAS

if nargin < 3 || isempty(tNow), tNow = 0; end
if nargin < 2 || isempty(sky), sky = struct(); end

sky = set_default(sky, 'type', 'day');
sky = set_default(sky, 'channels', 1);
sky = set_default(sky, 'cloudHeight', 900);
sky = set_default(sky, 'cloudScale', 600);
sky = set_default(sky, 'cloudDrift', [8 3]);
sky = set_default(sky, 'cloudSeed', 11);
sky = set_default(sky, 'sunGain', 3.0);
sky = set_default(sky, 'starCount', 220);
sky = set_default(sky, 'seed', 5);

H = cam.imageSize(1);
W = cam.imageSize(2);
theta = cam.thetaMap;
phi = cam.phiMap;
mask = double(cam.circleMask);

% --- per-type defaults -----------------------------------------------
switch lower(sky.type)
    case 'day'
        sky = set_default(sky, 'zenithLevel', 0.55);
        sky = set_default(sky, 'horizonLevel', 0.85);
        sky = set_default(sky, 'cloudCover', 0);   % clear
        sky = set_default(sky, 'sunAzElDeg', [50 55]);
        tint = [0.62 0.78 1.00];
    case 'cloudy'
        sky = set_default(sky, 'zenithLevel', 0.50);
        sky = set_default(sky, 'horizonLevel', 0.72);
        sky = set_default(sky, 'cloudCover', 0.55);
        sky = set_default(sky, 'sunAzElDeg', [50 55]);
        tint = [0.80 0.85 0.95];
    case 'dusk'
        sky = set_default(sky, 'zenithLevel', 0.16);
        sky = set_default(sky, 'horizonLevel', 0.48);
        sky = set_default(sky, 'cloudCover', 0.25);
        sky = set_default(sky, 'sunAzElDeg', [200 4]);
        sky = set_default(sky, 'sunGain', 1.2);
        tint = [1.00 0.72 0.48];
    case 'night'
        sky = set_default(sky, 'zenithLevel', 0.020);
        sky = set_default(sky, 'horizonLevel', 0.055);
        sky = set_default(sky, 'cloudCover', 0.0);
        sky = set_default(sky, 'sunAzElDeg', []);
        tint = [0.55 0.65 1.00];
    otherwise
        error('render_sky_background:badType', ...
            'sky.type must be day, cloudy, dusk, or night.');
end

% --- base gradient: brighter toward the horizon -----------------------
frac = min(1, theta / cam.thetaMax);
L = sky.zenithLevel + (sky.horizonLevel - sky.zenithLevel) * frac.^1.5;

% --- clouds -----------------------------------------------------------
if sky.cloudCover > 0
    if ~isfield(sky, 'cache') || ~isfield(sky.cache, 'cloudField')
        sky.cache.cloudField = make_noise_field(sky.cloudSeed);
    end
    % Ray/cloud-layer intersection: a pixel at zenith angle theta hits
    % the cloud deck at a horizontal offset h*tan(theta). Clamping theta
    % keeps the horizon finite and naturally piles cloud up at the rim.
    tanTh = tan(min(theta, 72*pi/180));
    xs = sky.cloudHeight * tanTh .* cos(phi) + sky.cloudDrift(1)*tNow;
    ys = sky.cloudHeight * tanTh .* sin(phi) + sky.cloudDrift(2)*tNow;

    nf = sample_noise_field(sky.cache.cloudField, xs, ys, sky.cloudScale);
    % Threshold into cloud / clear with a soft edge.
    thr = 1 - sky.cloudCover;
    cl = smoothstep((nf - thr) / 0.18);
    % Beyond about 60 deg the cloud deck is far enough away that
    % individual cells are unresolvable and merge into haze. Fading the
    % modulation out also removes the aliasing that sampling a noise
    % field at tan(theta) would otherwise produce near the rim.
    haze = 1 - smoothstep((theta - 60*pi/180) / (22*pi/180));
    cl = cl .* haze;
    L = L .* (1 - 0.35*cl) + cl .* (0.92 * (0.75 + 0.25*nf));
    sky.lastCloud = cl;
end

% --- sun / glare ------------------------------------------------------
if ~isempty(sky.sunAzElDeg)
    az = sky.sunAzElDeg(1)*pi/180;
    el = sky.sunAzElDeg(2)*pi/180;
    cosAng = sin(el)*cos(theta) + cos(el)*sin(theta).*cos(phi - az);
    ang = acos(max(-1, min(1, cosAng)));
    disc  = sky.sunGain * exp(-(ang/(1.2*pi/180)).^2);
    halo  = 0.30 * sky.sunGain * exp(-(ang/(14*pi/180)).^2);
    L = L + disc + halo;
end

% --- stars (night only) ----------------------------------------------
if strcmpi(sky.type, 'night') && sky.starCount > 0
    if ~isfield(sky, 'cache') || ~isfield(sky.cache, 'stars') || ...
            ~isequal(size(sky.cache.stars), [H W])
        sky.cache.stars = make_star_field(H, W, cam, sky.starCount, sky.seed);
    end
    L = L + sky.cache.stars;
end

L = max(L, 0);

% --- colorize and mask ------------------------------------------------
if sky.channels == 3
    % Normalize the tint so switching between mono and RGB changes the
    % color balance without changing the exposure level.
    tint = tint / mean(tint);
    bg = zeros(H, W, 3);
    for c = 1:3
        bg(:,:,c) = L * tint(c);
    end
else
    % Monochrome sensors, as in the ground camera packages, respond
    % roughly to luminance.
    bg = L;
end
bg = bg .* repmat(mask, [1 1 size(bg, 3)]);

end

% =====================================================================
function s = set_default(s, field, value)
if ~isfield(s, field) || isempty(s.(field))
    if ~(strcmp(field, 'sunAzElDeg') && isfield(s, field))
        s.(field) = value;
    end
end
end

% =====================================================================
function F = make_noise_field(seed)
%MAKE_NOISE_FIELD Three-octave value noise on a unit grid, in [0,1].
oldState = rng_state_save();
rng(seed);
N = 96;
F = struct();
F.N = N;
F.octaves = {rand(N, N), rand(2*N, 2*N), rand(4*N, 4*N)};
F.weights = [0.55 0.30 0.15];
rng_state_restore(oldState);
end

% =====================================================================
function out = sample_noise_field(F, xs, ys, cellSize)
%SAMPLE_NOISE_FIELD Bilinear multi-octave lookup, wrapped periodically.
out = zeros(size(xs));
for k = 1:numel(F.octaves)
    G = F.octaves{k};
    n = size(G, 1);
    scale = cellSize / (2^(k-1));
    xi = mod(xs / scale, n) + 1;
    yi = mod(ys / scale, n) + 1;
    % Wrap by padding one row/column so interp2 never runs off the edge.
    Gp = [G, G(:,1); G(1,:), G(1,1)];
    out = out + F.weights(k) * interp2(Gp, xi, yi, 'linear', 0.5);
end
out = (out - min(out(:))) / max(eps, (max(out(:)) - min(out(:))));
end

% =====================================================================
function S = make_star_field(H, W, cam, count, seed)
%MAKE_STAR_FIELD Static point stars inside the image circle.
oldState = rng_state_save();
rng(seed);
S = zeros(H, W);
placed = 0;
guard = 0;
while placed < count && guard < 20*count
    guard = guard + 1;
    r = randi(H); c = randi(W);
    if ~cam.circleMask(r, c), continue; end
    S(r, c) = S(r, c) + 0.04 + 0.22*rand^3;
    placed = placed + 1;
end
% Small point-spread so stars read as stars rather than salt noise.
k = [0.05 0.12 0.05; 0.12 0.32 0.12; 0.05 0.12 0.05];
S = conv2(S, k, 'same');
rng_state_restore(oldState);
end

% =====================================================================
function y = smoothstep(x)
x = max(0, min(1, x));
y = x.^2 .* (3 - 2*x);
end

% =====================================================================
function s = rng_state_save()
try
    s = rng;
catch
    s = [];   % Octave fallback
end
end

function rng_state_restore(s)
if ~isempty(s)
    try
        rng(s);
    catch
    end
end
end
