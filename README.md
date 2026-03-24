# RIS-Enabled Integrated Localization and Communication for Autonomous Systems

This repository provides a research-oriented reproduction and structured implementation of RIS-enabled integrated localization and communication (ILAC) for autonomous vehicles.

Based on the work:
Eskandari, Savkin, Deghat (IEEE TGCN, 2026)

---

## Overview

This project investigates how reconfigurable intelligent surfaces (RIS) can enhance localization and communication performance in autonomous systems, particularly in mmWave environments.

The focus is on:

- Modeling cascaded RIS-assisted communication channels  
- Designing beamforming and phase-shift strategies  
- Developing localization estimators with stability guarantees  
- Integrating communication and localization into a unified framework  

---

## Method

The system combines communication modeling, control-inspired estimation, and SLAM techniques:

- Cascaded BS → RIS → Receiver channel model  
- Geometry-based beam steering for directional communication  
- Differential estimator with Lyapunov stability analysis  
- Particle filter for initialization  
- Pose graph SLAM for drift correction  

All modules are implemented in MATLAB with explicit equation-to-code mapping.

---

## Key Contributions

- Full reproduction of RIS-enabled ILAC pipeline  
- Explicit mapping from theoretical equations to implementation  
- Integration of communication, localization, and SLAM  
- Stability-aware estimator design  
- Analysis of multipath effects in mmWave environments  

---

## Simulation Pipeline

Run the full simulation:

cd ris_localization
main_simulation

This executes all stages including:

- Beam steering validation  
- Localization estimation  
- Multipath impact analysis  
- Pose graph SLAM refinement  

---

## System Structure

Core components:

- Channel model (mmWave + RIS)  
- Beamforming and phase shift design  
- Localization estimator  
- SLAM-based correction  
- Data interface for real-world datasets  

---

## Research Context

This project reflects key research challenges in autonomous systems:

- Integration of communication and localization  
- Robust estimation under uncertainty  
- System-level coordination in cyber-physical systems  
- Bridging theoretical models with implementable algorithms  

---

## Requirements

- MATLAB R2021b or later  

---

## Notes

This repository is intended as a research-oriented implementation, focusing on system understanding, theoretical grounding, and reproducibility rather than production deployment.

---

## License

MIT License
