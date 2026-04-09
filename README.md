# RIS-Enabled mmWave Spatial Sensing and Localization for Autonomous Systems

This repository provides a research-oriented reproduction and structured implementation of **RIS-enabled integrated localization and communication (ILAC)** for autonomous systems operating in mmWave environments.

Based on the work:  
**Eskandari, Savkin, Deghat** (*IEEE Transactions on Green Communications and Networking*, 2026)

---

## Overview

This project investigates how **reconfigurable intelligent surfaces (RIS)** — passive structures that reshape wireless propagation — can enable **spatial sensing and localization** in autonomous systems, particularly in mmWave bands.

The core idea shares a fundamental principle with **structure-assisted sensing**: a carefully designed passive element (here, a RIS panel) introduces **spatially diverse signal patterns** that encode directional and distance information into reflected signals. By estimating the channel matrix and applying sparse recovery techniques, the receiver can reconstruct spatial information about the environment — drawing on the same **computational sensing and channel inversion** principles used in coded-aperture imaging and low-power depth-map reconstruction.

The focus is on:

- Modeling cascaded RIS-assisted channels as **spatially coded measurement matrices**
- Designing geometry-aware beamforming and phase-shift strategies for **signal diversity maximization**
- Developing **power-aware localization estimators** with stability guarantees
- Integrating communication and localization into a unified framework with **iterative scene refinement**

---

## Method

The system combines communication channel modeling, control-inspired estimation, and SLAM techniques:

- **Cascaded BS → RIS → Receiver channel model**: models signal propagation through the RIS panel with per-element phase control, forming a spatially coded channel matrix
- **Geometry-based beam steering**: directional beamforming design to maximize signal diversity across the sensing region
- **Channel matrix estimation and sparse signal recovery**: reconstructing spatial information from received signal measurements via matrix inversion techniques
- **Differential estimator with Lyapunov stability analysis**: provides convergence guarantees for the localization pipeline
- **Particle filter initialization**: coarse-grained probabilistic estimation of target locations before iterative refinement
- **Pose graph SLAM for drift correction**: fuses sequential estimates to suppress accumulated errors during motion

All modules are implemented in **MATLAB** with explicit equation-to-code mapping.

---

## Key Contributions

- Full reproduction of the RIS-enabled ILAC pipeline with modular, readable code
- Explicit mapping from theoretical equations to implementation for reproducibility
- **Channel matrix conditioning analysis**: evaluating signal diversity quality for spatial reconstruction
- **Sparse recovery and iterative refinement**: power-aware scene reconstruction that leverages signal sparsity
- Integration of communication, localization, and SLAM in a unified sensing framework
- Systematic evaluation of **multipath effects** and beam steering performance in mmWave environments

---

## Simulation Pipeline

Run the full simulation:

```matlab
cd ris_localization
main_simulation
```

This executes all stages including:

- Channel matrix construction and conditioning evaluation
- Beam steering validation and signal diversity analysis
- Localization estimation with stability verification
- Multipath impact analysis
- Pose graph SLAM refinement and spatial reconstruction accuracy assessment

---

## System Structure

Core components:

| Module | Description |
|---|---|
| **Channel Model** | Cascaded mmWave + RIS propagation with per-element phase shifts |
| **Beamforming** | Geometry-aware phase shift design for spatial diversity |
| **Sparse Recovery** | Channel matrix inversion and signal reconstruction |
| **Localization Estimator** | Differential estimator with Lyapunov-based stability |
| **SLAM Correction** | Pose graph optimization for drift suppression |
| **Evaluation** | Signal diversity metrics, reconstruction accuracy, conditioning analysis |

---

## Research Context

This project reflects key research challenges at the intersection of **wireless sensing, computational perception, and embedded systems**:

- **Structure-assisted spatial sensing**: using passive physical structures (RIS) to create signal diversity without additional power — analogous to acoustic metamaterial-based sensing approaches
- **Computational sensing and sparse reconstruction**: recovering spatial information from coded measurements via channel matrix inversion, related to compressed sensing and coded-aperture techniques
- **Low-power and resource-aware design**: power-conscious algorithm design suitable for embedded and IoT platforms
- **Bridging theoretical models with implementable systems**: reproducible equation-to-code mapping for real-world deployment

---

## Requirements

- MATLAB R2021b or later

---

## Notes

This repository is intended as a research-oriented implementation, focusing on system understanding, theoretical grounding, and reproducibility rather than production deployment.

---

## Author

**Yuanzhe (Nikola) Chen**

PhD Applicant (Fall 2027) · M.Eng in Electrical Engineering @ UNSW Sydney

---

## Contact

- **Email**: yuanzhe.chen@student.unsw.edu.au
- **LinkedIn**: [www.linkedin.com/in/yuanzhe-chen-6b2158351](http://www.linkedin.com/in/yuanzhe-chen-6b2158351)
- **Google Scholar**: [scholar.google.com/citations?hl=en&user=iz5iX38AAAAJ](https://scholar.google.com/citations?hl=en&user=iz5iX38AAAAJ)

---

## License

MIT License
