% MATLAB UAV Acoustic Emulator
%
% Core simulation
%   simulate_uav_acoustics       - Main entry point.
%   uav_acoustic_defaults        - Default and override settings.
%   generate_uav_trajectory      - Straight, arc, loiter, and waypoint paths.
%   generate_source_signal       - Tone, multitone, rotor, broadband, or user source.
%   propagate_to_microphones     - Delay, attenuation, Doppler, and reflection.
%   add_noise                    - White or colored ambient noise at a target SNR.
%   apply_sensor_dropout         - Missing sensor, burst, or sample simulation.
%
% Utilities
%   save_simulation              - Save output arrays and metadata to a MAT file.
%   plot_simulation_results      - Plot geometry, waveforms, and spectrograms.
%   install_uav_acoustic_emulator- Add toolbox folders to the MATLAB path.
%
% Examples and tests
%   examples/example_basic.m
%   examples/example_20_microphones.m
%   tests/run_self_test.m
