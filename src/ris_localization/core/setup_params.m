function params = setup_params()
% SETUP_PARAMS  Initialize all system parameters from the paper
%
% Returns a struct containing:
%   - Carrier frequency and wavelength
%   - BS and RIS array configurations
%   - Transmit power and noise
%   - Estimator constants
%   - Simulation grid parameters

    %% Wireless Communication Parameters
    params.fc = 30e9;               % Carrier frequency: 30 GHz (mmWave)
    params.c = 3e8;                 % Speed of light (m/s)
    params.lambda = params.c / params.fc;  % Wavelength
    params.d_spacing = params.lambda / 2;  % Antenna/element spacing = lambda/2

    %% BS Antenna Array (UPA: Uniform Planar Array)
    params.N_h = 20;                % Horizontal antennas per BS
    params.N_v = 20;                % Vertical antennas per BS
    params.N = params.N_h * params.N_v;  % Total antennas per BS (400)
    params.d_h = params.d_spacing;  % Horizontal antenna size
    params.d_v = params.d_spacing;  % Vertical antenna size
    params.d_s = params.d_spacing;  % Antenna spacing

    %% RIS Array Configuration
    params.M_l = 20;                % RIS elements along length
    params.M_w = 20;                % RIS elements along width
    params.M = params.M_l * params.M_w;  % Total RIS elements (400)
    params.d_l = params.d_spacing;  % RIS element length
    params.d_w = params.d_spacing;  % RIS element width

    %% Transmit Power and Noise
    params.P_Tx_dBm = 30;          % Transmit power in dBm (1 W)
    params.P_Tx = 10^((params.P_Tx_dBm - 30) / 10);  % Transmit power in W
    params.noise_power_dBm = -90;   % Noise power in dBm
    params.sigma2 = 10^((params.noise_power_dBm - 30) / 10);  % Noise variance

    %% Antenna and RIS Gains (dBi)
    params.G_BS_T_dBi = 25;        % BSTx antenna gain
    params.G_BS_R_dBi = 25;        % BSRx antenna gain
    params.G_RIS_dBi = 5;          % RIS reflection gain
    params.G_BS_T = 10^(params.G_BS_T_dBi / 10);
    params.G_BS_R = 10^(params.G_BS_R_dBi / 10);
    params.G_RIS = 10^(params.G_RIS_dBi / 10);

    %% Estimator Parameters (Eq. 26, 29)
    params.c1 = 0.3;               % Estimator constant for azimuth
    params.c2 = 0.3;               % Estimator constant for elevation
    params.max_iter = 20;           % Max estimator iterations
    params.epsilon = 1e-4;          % Convergence threshold

    %% Particle Filtering Parameters
    params.N_particles = 100;       % Number of random poses (as in paper)
    params.pf_range_xy = 40;        % Search range in XY (meters)
    params.pf_range_z = 5;          % Search range in Z (meters)
    params.pf_range_theta = 2*pi;   % Search range for heading

    %% Pose Graph SLAM Parameters
    params.gamma = 10;              % Scaling hyperparameter (Eq. 30)
    params.tau = 0.1;               % Time constant for convergence (seconds)
    params.dt = 0.01;               % MIMO/RIS response time step (seconds)

    %% Vehicle Kinematic Parameters (Eq. 33)
    params.L_RV = 2.7;             % Vehicle wheelbase (meters)
    params.v_max = 15;             % Max vehicle speed (m/s, ~54 km/h)

    %% Ray Tracing / Multipath Parameters
    params.num_rays_list = [1, 2, 3, 4, 5];  % Number of rays for Fig. 5
    params.scatter_sigma = 0.01;    % Scattering coefficient

    %% Environment Losses
    params.weather_loss_dB = 2;     % Weather attenuation (dB)
    params.reflection_loss_dB = 3;  % Material reflection loss (dB)
    params.polarization_loss_dB = 1;% Polarization mismatch loss (dB)

    fprintf('  System parameters initialized:\n');
    fprintf('    Carrier freq: %.1f GHz, lambda: %.4f m\n', params.fc/1e9, params.lambda);
    fprintf('    BS array: %dx%d = %d antennas\n', params.N_h, params.N_v, params.N);
    fprintf('    RIS array: %dx%d = %d elements\n', params.M_l, params.M_w, params.M);
    fprintf('    Element spacing: %.4f m (lambda/2)\n', params.d_spacing);
end
