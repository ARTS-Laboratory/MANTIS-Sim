function frameDN = add_image_noise(img, photOpts)
%ADD_IMAGE_NOISE Convert scene radiance to a noisy 8-bit sensor frame.
%
%   frameDN = ADD_IMAGE_NOISE(img, photOpts) takes a normalized radiance
%   image (0..1, as produced by the renderer) and returns a uint8 image
%   after a first-order photon-transfer sensor model:
%
%       electrons  = radiance * fullWell
%       electrons += shot noise, sqrt(electrons) * N(0,1)
%       electrons += read noise, readNoise * N(0,1)
%       DN         = gain * (electrons + blackLevel) * 255 / fullWell
%
%   The point of modeling it this way rather than adding a fixed SNR is
%   that the noise then follows the scene: a bright day frame is nearly
%   noise free, and a night frame pushed up with analog gain is visibly
%   grainy, which is exactly the regime where vision stops carrying the
%   detection and the microphones have to.
%
% INPUTS
%   img      : H-by-W-by-C normalized radiance in [0, 1+]
%   photOpts : struct with fields
%       .fullWell    electrons at full scale (default 8000)
%       .readNoise   read noise in electrons rms (default 6)
%       .gain        analog gain applied after the sensor (default 1)
%       .blackLevel  offset in DN (default 4)
%       .quantize    true/false 8-bit quantization (default true)
%
% OUTPUT
%   frameDN : H-by-W-by-C uint8 image
%
% See also SIMULATE_UAV_CAMERAS, RENDER_SKY_BACKGROUND

if nargin < 2, photOpts = struct(); end
if ~isfield(photOpts, 'fullWell'),   photOpts.fullWell = 8000; end
if ~isfield(photOpts, 'readNoise'),  photOpts.readNoise = 6;   end
if ~isfield(photOpts, 'gain'),       photOpts.gain = 1;        end
if ~isfield(photOpts, 'blackLevel'), photOpts.blackLevel = 4;  end
if ~isfield(photOpts, 'quantize'),   photOpts.quantize = true; end

e = max(0, img) * photOpts.fullWell;
e = min(e, photOpts.fullWell);                     % sensor saturation

shot = sqrt(e) .* randn(size(e));
read = photOpts.readNoise * randn(size(e));
e = e + shot + read;

dn = photOpts.gain * e * (255 / photOpts.fullWell) + photOpts.blackLevel;
dn = max(0, min(255, dn));

if photOpts.quantize
    dn = round(dn);
end
frameDN = uint8(dn);

end
