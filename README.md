# RIS-mmWave-Localization-Reproduction

[![MATLAB](https://img.shields.io/badge/MATLAB-R2026a-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Location](https://img.shields.io/badge/Environment-Sydney_CBD-orange.svg)](https://www.openstreetmap.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

This repository contains a high-fidelity MATLAB implementation of the **RIS-enabled integrated localization and communication (ILAC)** framework. The project focuses on achieving centimeter-level localization accuracy for autonomous vehicles in dense urban environments by leveraging B5G/6G wireless networks.

## 🚀 Highlights
* **High Precision**: Achieves a 3D positioning accuracy of **20 cm** in simulated dense urban settings.
* **Energy Efficient**: Utilizes passive Reconfigurable Intelligent Surfaces (RIS) mounted on the vehicle roof, requiring minimal energy and computing resources.
* **Robust Control**: Implements a non-linear estimator verified by the **Lyapunov direct method** to ensure global stability in dynamic environments.
* **Real-world Context**: Built using OpenStreetMap (OSM) data of **Sydney, Australia**, with 3D ray-tracing channel modeling.

---

## 🛠️ System Architecture

### 1. Environment & Channel Modeling
The simulation is grounded in a 3D reconstruction of the Sydney CBD.
* **Carrier Frequency**: 30 GHz (mmWave).
* **Array Configuration**: $20 \times 20$ Massive MIMO Base Stations ($BS_{Tx}$, $BS_{Rx}$) and a roof-mounted RIS.
* **Propagation**: Includes free-space loss, reflection loss, and multi-path scattering effects simulated via MATLAB's ray-tracing engine.

### 2. Localization Strategy
The algorithm operates in two distinct phases:
* **Initial Estimation**: 100-sample Particle Filtering to obtain a coarse pose estimate within a few meters.
* **Accurate Tracking**: A differential-based estimator $\mathfrak{E}$ that iteratively optimizes beamforming and phase shifts.

The estimator is defined as:
$$\mathfrak{E}(x_{1},x_{2}):\begin{cases}\dot{x}_{1}=-c_{1}sin~x_{1} \\ \dot{x}_{2}=-c_{2}sin~x_{2}\end{cases}$$

Stability is proven using the following Lyapunov function:
$$\mathcal{V}=2-cos^{2}x_{1}-cos^{2}x_{2}\ge0$$

---

## 📈 Performance
In simulated urban canyons, the estimator converges to an accuracy of a few tens of centimeters within **10-20 iterations**. Even under multi-path interference (up to 5 rays), the system maintains robust convergence through the use of virtual loop closures in a pose-graph SLAM framework.

---

## 💻 Getting Started

### Prerequisites
* MATLAB R2024b or later.
* **Toolboxes**: Communications Toolbox, Phased Array System Toolbox, and Mapping Toolbox.

### Execution
1. Clone the repository and ensure `sydney.osm` is in the working directory.
2. Run `main_simulation.m` to initialize the 3D environment and start the localization loop.
3. Observe the convergence of signal strength and reduction in localization error in the generated figures.

---

## 📚 Acknowledgments & Citations
This code is an independent reproduction of research conducted at the **School of Electrical Engineering and Telecommunications, UNSW Sydney**.

**Original Paper:**
> M. Eskandari, A. V. Savkin, and M. Deghat, "RIS-Enabled Energy Efficient Integrated Localization and Communication for Autonomous Vehicles: Complementary to Visual SLAM," *IEEE Transactions on Green Communications and Networking*, vol. 10, 2026.

---

## 👤 Author
**Yuanzhe (Nikola) Chen**
* M.Eng in Electrical Engineering student @ UNSW Sydney.
* Tesla Engineering Key Opinion Leader (KOL).
* Interested in Robotics, Path Planning, and Embodied AI.
