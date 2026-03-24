function env = setup_urban_environment(params)
% SETUP_URBAN_ENVIRONMENT  Configure the dense urban environment
%
% Sets up BS positions, buildings, and RV trajectory for the City of Sydney
% simulation scenario as described in Section V.
%
% NOTE: For real deployment, replace with actual OpenStreetMap data loaded
%       via MATLAB's siteviewer. The interface is provided below.
%
% Input:  params - system parameters struct
% Output: env    - environment struct with positions and trajectory

    %% =====================================================================
    %  DATA INTERFACE: Real Map Data
    %  Uncomment and modify to load actual OpenStreetMap data
    % =====================================================================
    % % Load OpenStreetMap data for Sydney
    % osmFile = 'sydney_map.osm';  % Download from openstreetmap.org
    % if exist(osmFile, 'file')
    %     viewer = siteviewer('Buildings', osmFile);
    %     % Extract building footprints, heights, materials
    %     env.buildings = extractBuildingsFromOSM(osmFile);
    % end
    %
    % % Alternative: Load pre-processed map data
    % if exist('data/sydney_map_data.mat', 'file')
    %     load('data/sydney_map_data.mat', 'buildings', 'roads');
    %     env.buildings = buildings;
    %     env.roads = roads;
    % end
    % =====================================================================

    %% BS Positions (based on Sydney simulation in Section V)
    % BSTx position is the origin of local coordinates
    env.p_BS_Tx = [0; 0; 30];          % BSTx at origin, height 30m
    % BSRx position in local Cartesian coordinates
    env.p_BS_Rx = [-5; 55; 35];        % BSRx position, height 35m

    %% RV (Vehicle) Actual Pose - ground truth
    % Based on Fig. 4(b), the actual RV position
    env.p_RV_actual = [-13.5; 43.5; 1.5];  % Actual RV position (roof height ~1.5m)
    env.theta_RV_actual = deg2rad(45);       % Actual RV heading (radians)

    %% Building Footprints (simplified Sydney CBD representation)
    % Each building: [x_center, y_center, width, depth, height]
    env.buildings = [
        -30, 15, 15, 10, 45;    % Building 1
        -15, 10, 12, 12, 60;    % Building 2
        -35, 40, 10, 15, 50;    % Building 3
        -10, 55, 8,  10, 40;    % Building 4
        5,   30, 12, 8,  55;    % Building 5
        -25, 55, 10, 8,  35;    % Building 6
        10,  10, 8,  12, 70;    % Building 7
        -40, 25, 10, 10, 30;    % Building 8
    ];

    %% Ground Plane
    env.ground_z = 0;          % Ground level
    env.area_x = [-45, 15];    % X range of simulation area (meters)
    env.area_y = [-5, 65];     % Y range of simulation area (meters)

    %% Road/Trajectory for Pose Graph SLAM
    % Define a vehicle trajectory through the urban area
    % Waypoints: [x, y, z, theta, time]
    env.trajectory.waypoints = [
        -35, 5,  1.5, deg2rad(90),   0;     % Start
        -35, 15, 1.5, deg2rad(90),   2;
        -30, 25, 1.5, deg2rad(60),   4;
        -25, 35, 1.5, deg2rad(45),   6;
        -20, 40, 1.5, deg2rad(30),   8;
        -13.5, 43.5, 1.5, deg2rad(45), 10;  % Target pose (Fig. 4)
        -10, 48, 1.5, deg2rad(60),  12;
        -8,  55, 1.5, deg2rad(80),  14;
        -5,  60, 1.5, deg2rad(90),  16;     % End
    ];

    % Interpolate trajectory for continuous path
    t_wp = env.trajectory.waypoints(:,5);
    dt_traj = 0.1;  % 10 Hz trajectory update
    t_interp = (t_wp(1):dt_traj:t_wp(end))';
    env.trajectory.t = t_interp;
    env.trajectory.x = interp1(t_wp, env.trajectory.waypoints(:,1), t_interp, 'pchip');
    env.trajectory.y = interp1(t_wp, env.trajectory.waypoints(:,2), t_interp, 'pchip');
    env.trajectory.z = interp1(t_wp, env.trajectory.waypoints(:,3), t_interp, 'pchip');
    env.trajectory.theta = interp1(t_wp, env.trajectory.waypoints(:,4), t_interp, 'pchip');
    env.trajectory.N_poses = length(t_interp);

    %% Material Properties (for ray tracing)
    env.materials.concrete_loss_dB = 15;    % Concrete reflection loss
    env.materials.glass_loss_dB = 5;        % Glass reflection loss
    env.materials.metal_loss_dB = 3;        % Metal reflection loss

    %% =====================================================================
    %  DATA INTERFACE: External Sensor Data
    % =====================================================================
    % % Load wheel odometry data
    % if exist('data/odometry.mat', 'file')
    %     load('data/odometry.mat', 'odom_data');
    %     env.odometry = odom_data;
    % end
    %
    % % Load vSLAM pose estimates
    % if exist('data/vslam_poses.mat', 'file')
    %     load('data/vslam_poses.mat', 'vslam_poses');
    %     env.vslam_poses = vslam_poses;
    % end
    %
    % % Load GPS data (if available)
    % if exist('data/gps_data.mat', 'file')
    %     load('data/gps_data.mat', 'gps_data');
    %     env.gps = gps_data;
    % end
    % =====================================================================

    fprintf('  Environment configured:\n');
    fprintf('    BSTx: [%.1f, %.1f, %.1f] m\n', env.p_BS_Tx);
    fprintf('    BSRx: [%.1f, %.1f, %.1f] m\n', env.p_BS_Rx);
    fprintf('    RV actual: [%.1f, %.1f, %.1f] m, heading: %.1f deg\n', ...
        env.p_RV_actual, rad2deg(env.theta_RV_actual));
    fprintf('    Trajectory: %d poses over %.1f seconds\n', ...
        env.trajectory.N_poses, env.trajectory.t(end));
end
