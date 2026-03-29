# Stability Control System
A high-fidelity cart-pole simulation using RK4 integration and a non-linear control law to stabilize a 9-degree initial perturbation.


![Activestabilityproject-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/2710f90b-f7c4-48b1-91a9-ad20caa7248f)
Figure 1: Active Disturbance Rejection: Real-time stabilization of a $9.0^{\circ}$ initial perturbation. The controller utilizes non-linear gain scheduling to achieve asymptotic stability within 5 seconds while maintaining strict lateral track constraints.

### A Study in Disturbance Rejection and Asymmetric Control

## Project Overview
This project implements a physics simulation of the classic Inverted Pendulum (Cart-Pole) problem. The goal was to design a robust controller capable of stabilizing the pole from a 9 to 10-degree initial tilt while maintaining the cart within a constrained 4-meter track.

### Key Engineering Challenges:
- Non-Linear Dynamics: Handling disturbances beyond small-angle approximations.
- Track Constraint Management: Balancing restorative force against physical track limits.
- Numerical Stability: Implementing higher-order integration to prevent energy gain.


## Technical Implementation

### 1. Physics Engine (RK4 Integration)
Unlike simple Euler integration, this simulation uses a 4th-order Runge-Kutta (RK4) method. This ensures that the energy of the pendulum is conserved and that the simulation remains stable even at high-frequency control updates (dt = 0.006s).

### 2. Control Strategy
The system utilizes a custom Asymmetric Proportional-Derivative (PD) control law. 
- Feather Balancing: The torque output is scaled by a non-linear power function of the sine of the angle to provide more aggressive correction.
- Asymmetric Centering Bias: To prevent long-term lateral drift, a position-dependent bias is applied. 
- Damping Schedule: Specific derivative gains were tuned to minimize settling time.

---
### 🛠 System Specifications & Engineering Constraints

| Parameter | Value | Domain Significance (Aero/Robotics) |
| :--- | :--- | :--- |
| **Integration Method** | 4th-Order Runge-Kutta (RK4) | High-fidelity dynamics; prevents energy drift |
| **Time Step ($dt$)** | 0.006s | High-frequency sensor fusion & control loop rate |
| **Initial Perturbation** | $9.0^{\circ} \text{ to } 10.0^{\circ}$ | High-alpha recovery / Disturbance rejection |
| **Stability Window** | $> 150.0\text{s}$ | Long-endurance mission profiling |
| **Lateral Envelope** | $\pm 2.0\text{m}$ | Geofencing and workspace constraint handling |
| **Control Frequency** | $\approx 166\text{Hz}$ | Real-time embedded system performance proxy |

---

## 📊 Current Performance Benchmarks
* **Long-Duration Stability:** Successfully maintained vertical equilibrium for **>150 seconds** without numerical drift.
* **Station-Keeping Envelope:** Implemented a restorative bias that constrains the airframe within a **±2m lateral boundary** from the origin.
* **Numerical Precision:** Utilized **4th-Order Runge-Kutta (RK4)** integration to ensure energy conservation, crucial for high-fidelity aerospace modeling.
<img width="1804" height="1149" alt="image" src="https://github.com/user-attachments/assets/4e4df751-0f6e-4033-a9b4-cd93d325c527" />
Figure 2: Long-Duration Stability & Constraint Handling. Telemetry at T+152.0s demonstrating zero steady-state angular error (Theta = 0°). The controller successfully maintains Station-Keeping within the operational envelope (+1.80m), proving robust recovery and stability near the physical track boundaries—a critical requirement for both autonomous robotics and UAV flight-guards.

---

## How to Run
1. Ensure you have numpy and matplotlib installed:
   pip install numpy matplotlib
2. Run the simulation

---
## 🛠 Active Research & Future Iterations
* **Stochastic Disturbance Rejection:** Currently developing a module to inject periodic 50N impulses to simulate atmospheric turbulence and "wind gusts."
* **Recovery Analysis:** Evaluating the controller's "Time-to-Recovery" following 10-degree upset conditions under active disturbance.
* **3D Proxy Mapping:** Planning a transition from 2D Cart-Pole logic to 3D Quadcopter altitude-hold dynamics.
---
