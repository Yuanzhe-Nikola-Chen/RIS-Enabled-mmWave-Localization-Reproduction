classdef DataInterface
% DATAINTERFACE  Interface for loading real-world datasets
%
% This class provides standardized interfaces for:
%   1. OpenStreetMap data (buildings, roads)
%   2. Wheel odometry data
%   3. vSLAM pose estimates
%   4. GPS measurements
%   5. LiDAR point clouds
%   6. RIS hardware measurements
%   7. Channel measurement data
%
% Usage:
%   di = DataInterface('data/');
%   map_data = di.load_osm('sydney.osm');
%   odom = di.load_odometry('odom_log.csv');

    properties
        data_dir    % Base directory for data files
    end

    methods
        function obj = DataInterface(data_dir)
            if nargin < 1
                data_dir = 'data/';
            end
            obj.data_dir = data_dir;
            if ~exist(data_dir, 'dir')
                mkdir(data_dir);
            end
        end

        %% ============================================================
        %  1. OpenStreetMap Data
        % =============================================================
        function map = load_osm(obj, osm_file)
        % LOAD_OSM  Load OpenStreetMap .osm file
        %
        % Returns struct with:
        %   map.buildings - [Nx5] [x, y, width, depth, height]
        %   map.roads     - cell array of road polylines
        %   map.bbox      - [lat_min, lat_max, lon_min, lon_max]
        %
        % For MATLAB siteviewer integration:
        %   viewer = siteviewer('Buildings', osm_file);

            filepath = fullfile(obj.data_dir, osm_file);
            if exist(filepath, 'file')
                fprintf('Loading OSM data from: %s\n', filepath);
                % Parse OSM XML file
                % TODO: Implement OSM parser or use MATLAB siteviewer
                % doc = xmlread(filepath);
                % ... parse buildings, roads, etc.
                map = struct('buildings', [], 'roads', {{}}, 'bbox', []);
                warning('OSM parser not yet implemented. Use MATLAB siteviewer.');
            else
                warning('OSM file not found: %s', filepath);
                map = [];
            end
        end

        function [lat, lon] = cartesian_to_gps(~, x, y, ref_lat, ref_lon)
        % CARTESIAN_TO_GPS  Convert local cartesian to GPS coordinates
            R_earth = 6378137;  % Earth radius (m)
            lat = ref_lat + (y / R_earth) * (180 / pi);
            lon = ref_lon + (x / (R_earth * cosd(ref_lat))) * (180 / pi);
        end

        function [x, y] = gps_to_cartesian(~, lat, lon, ref_lat, ref_lon)
        % GPS_TO_CARTESIAN  Convert GPS to local cartesian coordinates
            R_earth = 6378137;
            x = (lon - ref_lon) * (pi / 180) * R_earth * cosd(ref_lat);
            y = (lat - ref_lat) * (pi / 180) * R_earth;
        end

        %% ============================================================
        %  2. Wheel Odometry Data
        % =============================================================
        function odom = load_odometry(obj, filename)
        % LOAD_ODOMETRY  Load wheel odometry measurements
        %
        % Expected CSV format:
        %   timestamp, dx, dy, dz, dtheta, v, delta
        %
        % Returns struct:
        %   odom.t      - [Nx1] timestamps
        %   odom.dx     - [Nx1] x displacement
        %   odom.dy     - [Nx1] y displacement
        %   odom.dtheta - [Nx1] heading change
        %   odom.v      - [Nx1] velocity
        %   odom.delta  - [Nx1] front wheel angle

            filepath = fullfile(obj.data_dir, filename);
            if exist(filepath, 'file')
                data = readmatrix(filepath);
                odom.t = data(:, 1);
                odom.dx = data(:, 2);
                odom.dy = data(:, 3);
                odom.dz = data(:, 4);
                odom.dtheta = data(:, 5);
                odom.v = data(:, 6);
                odom.delta = data(:, 7);
            else
                warning('Odometry file not found: %s', filepath);
                odom = [];
            end
        end

        %% ============================================================
        %  3. vSLAM Pose Estimates
        % =============================================================
        function poses = load_vslam_poses(obj, filename)
        % LOAD_VSLAM_POSES  Load visual SLAM estimated poses
        %
        % Expected format (TUM format):
        %   timestamp, tx, ty, tz, qx, qy, qz, qw
        %
        % Returns struct:
        %   poses.t        - [Nx1] timestamps
        %   poses.position - [Nx3] [x, y, z] positions
        %   poses.heading  - [Nx1] heading angle (from quaternion)

            filepath = fullfile(obj.data_dir, filename);
            if exist(filepath, 'file')
                data = readmatrix(filepath);
                poses.t = data(:, 1);
                poses.position = data(:, 2:4);
                % Extract heading from quaternion
                qw = data(:, 8); qz = data(:, 7);
                poses.heading = 2 * atan2(qz, qw);
            else
                warning('vSLAM file not found: %s', filepath);
                poses = [];
            end
        end

        %% ============================================================
        %  4. GPS Measurements
        % =============================================================
        function gps = load_gps(obj, filename, ref_lat, ref_lon)
        % LOAD_GPS  Load GPS measurement data
        %
        % Expected CSV format:
        %   timestamp, latitude, longitude, altitude, hdop
        %
        % Returns struct with both GPS and local cartesian coordinates

            filepath = fullfile(obj.data_dir, filename);
            if exist(filepath, 'file')
                data = readmatrix(filepath);
                gps.t = data(:, 1);
                gps.lat = data(:, 2);
                gps.lon = data(:, 3);
                gps.alt = data(:, 4);
                gps.hdop = data(:, 5);

                % Convert to local cartesian
                if nargin >= 4
                    [gps.x, gps.y] = obj.gps_to_cartesian(gps.lat, gps.lon, ref_lat, ref_lon);
                    gps.z = gps.alt;
                end
            else
                warning('GPS file not found: %s', filepath);
                gps = [];
            end
        end

        %% ============================================================
        %  5. LiDAR Point Cloud
        % =============================================================
        function pc = load_pointcloud(obj, filename)
        % LOAD_POINTCLOUD  Load LiDAR point cloud data
        %
        % Supports: .pcd, .ply, .mat (MATLAB pointCloud object)

            filepath = fullfile(obj.data_dir, filename);
            if exist(filepath, 'file')
                [~, ~, ext] = fileparts(filepath);
                switch ext
                    case '.mat'
                        loaded = load(filepath);
                        fnames = fieldnames(loaded);
                        pc = loaded.(fnames{1});
                    case {'.pcd', '.ply'}
                        pc = pcread(filepath);
                    otherwise
                        warning('Unsupported point cloud format: %s', ext);
                        pc = [];
                end
            else
                warning('Point cloud file not found: %s', filepath);
                pc = [];
            end
        end

        %% ============================================================
        %  6. RIS Hardware Configuration
        % =============================================================
        function ris = load_ris_config(obj, filename)
        % LOAD_RIS_CONFIG  Load RIS hardware configuration
        %
        % Expected JSON or MAT format with fields:
        %   M_l, M_w       - Array dimensions
        %   d_l, d_w       - Element sizes
        %   phase_bits     - Phase quantization bits
        %   max_phase_error- Maximum phase shift error (rad)

            filepath = fullfile(obj.data_dir, filename);
            if exist(filepath, 'file')
                [~, ~, ext] = fileparts(filepath);
                if strcmp(ext, '.mat')
                    ris = load(filepath);
                elseif strcmp(ext, '.json')
                    raw = fileread(filepath);
                    ris = jsondecode(raw);
                end
            else
                warning('RIS config file not found: %s', filepath);
                ris = [];
            end
        end

        %% ============================================================
        %  7. Channel Measurement Data
        % =============================================================
        function ch = load_channel_measurements(obj, filename)
        % LOAD_CHANNEL_MEASUREMENTS  Load channel measurement data
        %
        % Expected MAT format:
        %   ch.signal_strength - [NxK] received signal strength matrix
        %   ch.positions       - [Nx3] measurement positions
        %   ch.headings        - [Nx1] headings
        %   ch.timestamps      - [Nx1] timestamps

            filepath = fullfile(obj.data_dir, filename);
            if exist(filepath, 'file')
                ch = load(filepath);
            else
                warning('Channel measurement file not found: %s', filepath);
                ch = [];
            end
        end

        %% ============================================================
        %  Utility: Generate Synthetic Dataset
        % =============================================================
        function generate_synthetic_dataset(obj, params, env, filename)
        % GENERATE_SYNTHETIC_DATASET  Create synthetic test data
        %
        % Generates a complete synthetic dataset for testing the algorithms

            fprintf('Generating synthetic dataset...\n');

            % Trajectory
            N_poses = 100;
            t = linspace(0, 20, N_poses)';

            % Circular trajectory
            radius = 30;
            cx = -15; cy = 30;
            x = cx + radius * cos(2*pi*t/20);
            y = cy + radius * sin(2*pi*t/20);
            z = 1.5 * ones(N_poses, 1);
            theta = 2*pi*t/20 + pi/2;

            % Odometry with noise
            odom_noise = 0.05;
            dx = diff(x) + odom_noise * randn(N_poses-1, 1);
            dy = diff(y) + odom_noise * randn(N_poses-1, 1);
            dtheta = diff(theta) + 0.01 * randn(N_poses-1, 1);

            % GPS with noise
            gps_noise = 2.0;  % meters
            gps_x = x + gps_noise * randn(N_poses, 1);
            gps_y = y + gps_noise * randn(N_poses, 1);

            % Save
            dataset.trajectory.t = t;
            dataset.trajectory.x = x;
            dataset.trajectory.y = y;
            dataset.trajectory.z = z;
            dataset.trajectory.theta = theta;
            dataset.odometry.dx = dx;
            dataset.odometry.dy = dy;
            dataset.odometry.dtheta = dtheta;
            dataset.gps.x = gps_x;
            dataset.gps.y = gps_y;
            dataset.params = params;

            save(fullfile(obj.data_dir, filename), '-struct', 'dataset');
            fprintf('Synthetic dataset saved to: %s\n', ...
                fullfile(obj.data_dir, filename));
        end
    end
end
