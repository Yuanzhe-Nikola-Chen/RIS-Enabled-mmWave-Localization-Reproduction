clear; clc; close all;
addpath(genpath(pwd));

fprintf('=========================\n')
fprintf(' RIS-Enabled ILAC for Autonomous Vehicles\n');
fprintf(' Reproduction of Paper Simulations\n');
fprintf('==========================================\n\n');

%% 1. Setup System Parameters
fprintf('[1/6] Setting up system parameters...\n');
params = setup_params();

%% 2. Setup Urban Environment (City of Sydney)
fprintf('[2/6] Setting up dense urban enviroment...\n');
env = setup_urban_environment(params);

%% 3. Simulation A: Beam Steering and Signal Strength (Fig. 3d)
fprintf('[3/6] Simulating beam steering and signal strength...\n');
results_beam = simulate_beam_steering(params, env);
plot_beam_steering_results(results_beam, params, env);

%% 4. Simulation B: Particle Filtering + Estimator Convergence (Fig. 4)
fprintf('[4/6] Running particle filtering and estimator convergence...\n');
results_localization = simulate_localization(params, env);
plot_localization_results(results_localization, params, env);

%% 5. Simulation C: Multipath Impact (Fig. 5)
fprintf('[5/6] Evaluating multipath impact on localization...\n');
results_multipath = simulate_multipath_impact(params, env);
plot_multipath_results(results_multipath);

%% 6. Simulation D: Pose Graph SLAM with Virtual Loop Closure
fprintf('[6/6] Running pose graph SLAM with virtual loop closure...\n');
results_slam = simulate_pose_graph_slam(params, env);
plot_slam_results(results_slam);

fprintf("\n===================================\n")
results_slam = simulate_pose_graph_slam(params, env);
plot_slam_results(results_slam);

fprintf('\n===========================\n')
fprintf('All simulations completed successfully\n');
fprintf('===============\n')
