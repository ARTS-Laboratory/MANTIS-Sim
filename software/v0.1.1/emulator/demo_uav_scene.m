%DEMO_UAV_SCENE Worked examples for the joint acoustic + vision emulator.
%
%   Run this script directly (F5, or `demo_uav_scene` at the command
%   line) after adding this folder to your MATLAB path.
%
%   Every example builds a network of ground sensor nodes, each holding
%   a microphone and an upward-looking fisheye camera, and simulates
%   both streams for one UAV overflight. The audio pipeline is the v0.1
%   pipeline, unchanged.
%
%   Rough runtime for the whole script: a couple of minutes.

clear; clc; close all;

%% ------------------------------------------------------------------
%  Example 1: SWIFT fixed-wing overflight, six nodes, clear day
%  ------------------------------------------------------------------
%  A 2.6 m fixed wing crosses the array at 80 m and 25 m/s. Six nodes
%  sit on a 50 m ring. Cameras run at 1 fps and 240x240 px, which is
%  what a low-power node can sustain; the audio still runs at 4 kHz.

fs = 4000;
t = (0:1/fs:24)';

nodeXY = generate_node_array(6, 'ring', 'radius', 50);

uavXYZ = generate_uav_trajectory(t, 'straight', ...
    'startXY', [-300 -40], 'endXY', [300 40], 'altitude', 80);

scene1 = simulate_uav_scene(nodeXY, uavXYZ, t, ...
    'platform', 'swift', ...
    'sky', 'day', ...
    'fps', 1, ...
    'imageSize', [240 240], ...
    'snrDb', 10, ...
    'enableDoppler', true, ...
    'seed', 1);

save_scene('demo_scene_swift_day.mat', scene1, 'splitVideo', true);
plot_scene_results(scene1);

%% ------------------------------------------------------------------
%  Example 2: F550 hexacopter, broken cloud, one node knocked out
%  ------------------------------------------------------------------
%  A smaller, slower target loitering at 30 m over a tighter array.
%  Node 4 is completely offline (microphone and camera), and node 2 has
%  lost only its microphone: the case the self-arranging network is
%  supposed to survive.

t2 = (0:1/fs:30)';
nodeXY2 = generate_node_array(6, 'ring', 'radius', 25);

uavXYZ2 = generate_uav_trajectory(t2, 'loiter', ...
    'center', [10 0], 'radius', 35, 'numLoops', 2, 'altitude', 30);

scene2 = simulate_uav_scene(nodeXY2, uavXYZ2, t2, ...
    'platform', 'f550', ...
    'sky', 'cloudy', ...
    'fps', 1, ...
    'imageSize', [240 240], ...
    'snrDb', 8, ...
    'noiseType', 'wind', ...
    'deadNodes', 4, ...
    'deadMics', 2, ...
    'seed', 7);

plot_scene_results(scene2);

%% ------------------------------------------------------------------
%  Example 3: night pass, nav lights only
%  ------------------------------------------------------------------
%  At night the silhouette disappears and the only visual signature is
%  the nav and strobe lights, while the acoustic channel is unchanged.
%  This is the regime where fusing the two streams earns its keep.

t3 = (0:1/fs:40)';
nodeXY3 = generate_node_array(6, 'random', 'radius', 70, 'minSep', 20, 'seed', 3);

uavXYZ3 = generate_uav_trajectory(t3, 'loiter', ...
    'center', [0 0], 'radius', 120, 'numLoops', 1, 'altitude', 100);

scene3 = simulate_uav_scene(nodeXY3, uavXYZ3, t3, ...
    'platform', 'swift', ...
    'sky', 'night', ...
    'fps', 1, ...
    'imageSize', [320 320], ...
    'snrDb', 6, ...
    'seed', 11);

plot_scene_results(scene3, 'trackNode', 1);

%% ------------------------------------------------------------------
%  Example 4: how many pixels does the camera actually need?
%  ------------------------------------------------------------------
%  A 185 deg fisheye spreads the whole sky across the sensor, so
%  angular resolution is what limits detection range in daylight, not
%  signal level. Print the budget, then re-render the Example 1
%  geometry at a higher resolution to see the difference.

pixel_range_budget('swift', 'imageSize', [240 480 960]);
pixel_range_budget('f550',  'imageSize', [240 480 960]);

t4 = (0:1/fs:24)';
scene4 = simulate_uav_scene(nodeXY, uavXYZ, t4, ...
    'platform', 'swift', ...
    'sky', 'day', ...
    'fps', 0.5, ...
    'imageSize', [960 960], ...
    'snrDb', 10, ...
    'seed', 1);

plot_scene_results(scene4);

%% ------------------------------------------------------------------
%  Example 5: export a labeled detection dataset
%  ------------------------------------------------------------------
%  Boxes come from the renderer, so the labels are exact. Frames with
%  no resolvable target are written as negatives.

export_yolo_dataset(scene4, 'demo_dataset_swift_day', 'minPixelArea', 2);

fprintf('\nDemo complete. Scenes are in the workspace as scene1..scene4.\n');
