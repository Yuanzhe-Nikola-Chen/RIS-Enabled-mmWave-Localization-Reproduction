# RIS-Enabled Energy Efficient ILAC for Autonomous Vehicles

## Paper Reference

> M. Eskandari, A. V. Savkin, and M. Deghat,
> "RIS-Enabled Energy Efficient Integrated Localization and Communication for Autonomous Vehicles: Complementary to Visual SLAM,"
> *IEEE Trans. Green Commun. Netw.*, vol. 10, 2026.

## Overview

Complete MATLAB reproduction of all algorithms and simulations from the paper, including:

- **Cascaded channel model** (BSTx → RIS → BSRx) with mmWave beamforming
- **Geometry-based beam steering** for BS-BF and RIS-PS
- **Differential-based estimator** with Lyapunov stability proof
- **Particle filtering** initialization
- **Pose graph SLAM** with virtual loop closure
- **Multipath impact analysis**

## Directory Structure

```
ris_localization/
├── main_simulation.m              # Entry point: runs all simulations
│
├── core/                          # Core algorithms (channel model, arrays)
│   ├── setup_params.m             # System parameters (Section V)
│   ├── setup_urban_environment.m  # Urban environment: Sydney CBD
│   ├── compute_antenna_coords.m   # Appendix A: Eq. (A-1), (A-2)
│   ├── compute_angles.m           # Appendix B: Eq. (B-1)–(B-4)
│   ├── compute_wave_vector.m      # Eq. (5): Normalized wave vector
│   ├── compute_steering_vector.m  # Eq. (3)–(7): Array steering vectors
│   ├── design_ris_phase_shift.m   # Eq. (17)–(18): RIS phase shift design
│   ├── compute_path_loss.m        # Eq. (9)–(16): Path loss and radiation
│   ├── compute_cosine_function.m  # Eq. (19)–(24): Cosine shape function
│   └── compute_received_signal.m  # Eq. (1)–(2): Full cascaded channel
│
├── estimator/                     # Localization estimator
│   ├── particle_filter_init.m     # Section IV.A: Particle filtering init
│   ├── run_estimator.m            # Eq. (26)–(29): 3-stage estimator
│   ├── verify_lyapunov_stability.m# Eq. (27): Lyapunov stability check
│   └── pose_graph_optimization.m  # Eq. (30)–(34): Virtual loop closure
│
├── simulation/                    # Simulation runners
│   ├── simulate_beam_steering.m   # Fig. 3(d): Beam steering evaluation
│   ├── simulate_localization.m    # Fig. 4: Full localization pipeline
│   ├── simulate_multipath_impact.m# Fig. 5: Multipath analysis (1–5 rays)
│   └── simulate_pose_graph_slam.m # Pose graph SLAM with drift correction
│
├── utils/                         # Plotting and visualization
│   ├── plot_beam_steering_results.m   # Fig. 3 plots
│   ├── plot_localization_results.m    # Fig. 4 plots + Lyapunov surface
│   ├── plot_multipath_results.m       # Fig. 5(b) plots
│   └── plot_slam_results.m           # Pose graph SLAM plots
│
├── data/                          # Data interface and datasets
│   └── DataInterface.m            # Class for loading external data
│
└── README.md                      # This file
```

## Equation-to-Code Mapping

| Equation | File | Function |
|----------|------|----------|
| Eq. (1)–(2) | `compute_received_signal.m` | Full cascaded channel |
| Eq. (3)–(4) | `compute_steering_vector.m` | BS steering vector |
| Eq. (5) | `compute_wave_vector.m` | Normalized wave vector |
| Eq. (6)–(7) | `compute_steering_vector.m` | RIS array response |
| Eq. (8) | `design_ris_phase_shift.m` | RIS phase shift matrix |
| Eq. (9)–(16) | `compute_path_loss.m` | Path loss and radiation |
| Eq. (17)–(18) | `design_ris_phase_shift.m` | Directional RIS-PS |
| Eq. (19)–(24) | `compute_cosine_function.m` | Cosine shape function |
| Eq. (25) | `run_estimator.m` (Stage 1) | Incident-only mode |
| Eq. (26)–(27) | `verify_lyapunov_stability.m` | Estimator + Lyapunov |
| Eq. (28) | `run_estimator.m` (Stage 2/3) | Reflection/heading mode |
| Eq. (29) | `run_estimator.m` | Discretized estimator Ed |
| Eq. (30) | `pose_graph_optimization.m` | Weighting coefficient Ωi |
| Eq. (31)–(34) | `pose_graph_optimization.m` | Pose graph optimization |
| Eq. (33) | `pose_graph_optimization.m` | Vehicle kinematics |
| Eq. (A-1) | `compute_antenna_coords.m` | BS antenna coordinates |
| Eq. (A-2) | `compute_antenna_coords.m` | RIS element coordinates |
| Eq. (B-1)–(B-4) | `compute_angles.m` | Azimuth/elevation angles |
| Eq. (C-1) | `compute_received_signal.m` | Matrix form derivation |

## Quick Start

```matlab
cd ris_localization
main_simulation
```

This runs all six simulation stages and generates figures matching the paper.

## Using Real Data

The `DataInterface` class provides standardized loaders:

```matlab
% Initialize
di = DataInterface('data/');

% Load OpenStreetMap
map = di.load_osm('sydney.osm');

% Load odometry
odom = di.load_odometry('wheel_odom.csv');

% Load vSLAM poses (TUM format)
poses = di.load_vslam_poses('orb_slam3_poses.txt');

% Load GPS
gps = di.load_gps('gps_log.csv', -33.8688, 151.2093);  % Sydney ref

% Load RIS hardware config
ris = di.load_ris_config('ris_hardware.mat');

% Load channel measurements
ch = di.load_channel_measurements('channel_data.mat');

% Generate synthetic test dataset
di.generate_synthetic_dataset(params, env, 'synthetic.mat');
```

### Expected Data Formats

| Data | Format | Columns |
|------|--------|---------|
| Odometry | CSV | `timestamp, dx, dy, dz, dtheta, v, delta` |
| vSLAM | TUM TXT | `timestamp, tx, ty, tz, qx, qy, qz, qw` |
| GPS | CSV | `timestamp, lat, lon, alt, hdop` |
| RIS Config | MAT/JSON | `M_l, M_w, d_l, d_w, phase_bits, max_phase_error` |
| Channel | MAT | `signal_strength, positions, headings, timestamps` |

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Carrier frequency | 30 GHz | mmWave band |
| BS array | 20×20 UPA | 400 antennas |
| RIS array | 20×20 UPA | 400 elements |
| Element spacing | λ/2 = 5 mm | Half-wavelength |
| Transmit power | 30 dBm | 1 W |
| Localization accuracy | ~0.2 m | 20 cm (LoS, 1 ray) |

## Requirements

- MATLAB R2021b or later
- Signal Processing Toolbox (optional, for advanced beamforming)
- Phased Array System Toolbox (optional, for Sensor Array Analyzer)
- Mapping Toolbox (optional, for siteviewer with OSM)
