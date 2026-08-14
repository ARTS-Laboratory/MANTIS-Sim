function nodeXY = generate_node_array(numNodes, layout, varargin)
%GENERATE_NODE_ARRAY Lay out ground sensor nodes on the x-y plane.
%
%   nodeXY = GENERATE_NODE_ARRAY(numNodes, layout, ...) returns an
%   N-by-2 array of node coordinates in meters. Each node is assumed to
%   carry both a microphone and an upward-looking camera, so the same
%   array is passed to SIMULATE_UAV_ACOUSTICS (as micXY) and to
%   SIMULATE_UAV_CAMERAS.
%
% LAYOUTS
%   'ring'    nodes evenly spaced on a circle
%       'radius'      circle radius in meters (default 50)
%       'center'      [x y] (default [0 0])
%       'phaseDeg'    angular offset of the first node (default 0)
%   'grid'    nodes on a roughly square grid
%       'spacing'     node spacing in meters (default 50)
%       'center'      [x y] (default [0 0])
%   'line'    nodes along a straight baseline
%       'spacing'     node spacing in meters (default 40)
%       'headingDeg'  baseline direction (default 0)
%       'center'      [x y] (default [0 0])
%   'random'  nodes dropped at random inside a disc, which is closer to
%             an air-dropped or hand-placed deployment
%       'radius'      disc radius in meters (default 80)
%       'center'      [x y] (default [0 0])
%       'minSep'      minimum separation between nodes (default 10)
%       'seed'        random seed (default 1)
%
% EXAMPLE
%   nodeXY = generate_node_array(6, 'ring', 'radius', 50);
%
% See also SIMULATE_UAV_SCENE, SIMULATE_UAV_CAMERAS

p = inputParser;
p.FunctionName = 'generate_node_array';
addRequired(p, 'numNodes');
addRequired(p, 'layout');
addParameter(p, 'radius', 50);
addParameter(p, 'center', [0 0]);
addParameter(p, 'phaseDeg', 0);
addParameter(p, 'spacing', 50);
addParameter(p, 'headingDeg', 0);
addParameter(p, 'minSep', 10);
addParameter(p, 'seed', 1);
parse(p, numNodes, layout, varargin{:});
opt = p.Results;

if numNodes < 1 || mod(numNodes, 1) ~= 0
    error('generate_node_array:badCount', 'numNodes must be a positive integer.');
end
c = opt.center(:)';

switch lower(layout)
    case 'ring'
        a = opt.phaseDeg*pi/180 + (0:numNodes-1)' * 2*pi/numNodes;
        nodeXY = [c(1) + opt.radius*cos(a), c(2) + opt.radius*sin(a)];

    case 'grid'
        side = ceil(sqrt(numNodes));
        [gx, gy] = meshgrid(0:side-1, 0:side-1);
        pts = [gx(:), gy(:)] * opt.spacing;
        pts = pts - repmat(mean(pts, 1), size(pts, 1), 1);
        nodeXY = pts(1:numNodes, :) + repmat(c, numNodes, 1);

    case 'line'
        s = ((0:numNodes-1)' - (numNodes-1)/2) * opt.spacing;
        h = opt.headingDeg*pi/180;
        nodeXY = [c(1) + s*cos(h), c(2) + s*sin(h)];

    case 'random'
        oldRand = rng_state_save();
        rng(opt.seed);
        nodeXY = zeros(numNodes, 2);
        placed = 0;
        guard = 0;
        while placed < numNodes && guard < 10000
            guard = guard + 1;
            r = opt.radius * sqrt(rand);
            a = 2*pi*rand;
            cand = [c(1) + r*cos(a), c(2) + r*sin(a)];
            if placed > 0
                d = sqrt(sum((nodeXY(1:placed,:) - repmat(cand, placed, 1)).^2, 2));
                if min(d) < opt.minSep, continue; end
            end
            placed = placed + 1;
            nodeXY(placed, :) = cand;
        end
        if placed < numNodes
            error('generate_node_array:packingFailed', ...
                ['Could not place %d nodes with minSep %.1f m inside ', ...
                 'radius %.1f m.'], numNodes, opt.minSep, opt.radius);
        end
        rng_state_restore(oldRand);

    otherwise
        error('generate_node_array:badLayout', ...
            'layout must be ring, grid, line, or random.');
end

end

% =====================================================================
function s = rng_state_save()
try
    s = rng;
catch
    s = [];
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
