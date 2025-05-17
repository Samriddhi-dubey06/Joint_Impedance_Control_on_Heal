# Joint Impedance Control on Heal Robot

This repository implements **joint-space impedance control** with **PID gains** and **quintic trajectory generation** for the Heal Robot. The controller computes joint torques based on position, velocity, and integral errors, ensuring compliant and smooth motion. The implementation supports **effort (torque) control**, is designed for ROS, and integrates trajectory planning based on maximum time and velocity.

---

## 🧠 Core Components

### PID Control Parameters
- **Proportional gains (Kp)**: `[148, 140, 130, 120, 115, 130]`
- **Derivative gains (Kd)**: `[57, 52, 45, 50, 40, 45]`
- **Integral gains (Ki)**: `[65, 60, 60, 60, 55, 60]`
- **Target Position**: `[0, 0, 0, 0, 0, 0]` (Home)

### Trajectory Generation
- **Method**: Quintic (5th-order) polynomial trajectory generation
- **Usage**: Ensures smooth position, velocity, and acceleration profiles between start and goal states.

---

## 🧮 Mathematical Formulation

### PID Control Law

The control torque is computed as:

`τ = Kp × e_pos + Kd × e_vel + Ki × ∫ e_pos dt`

Where:
- `e_pos = q_target - q_current` (position error)
- `e_vel = dq_target - dq_current` (velocity error)
- `∫ e_pos dt` is the accumulated integral of the position error

> Note: The integral term is clamped within ±50 to prevent windup.

---

### Quintic Trajectory Generation

To generate smooth trajectories, a 5th-order polynomial is used:

`q(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵`

The coefficients `[a₀, a₁, ..., a₅]` are computed using boundary conditions.

Where:
- `p₀, v₀, a₀` are the initial position, velocity, and acceleration
- `p_f, v_f, a_f` are the final position, velocity, and acceleration
- `T` is the total trajectory duration

---

## ⚙️ Implementation Flow

1. **Initialization**
   - ROS node, publishers, and subscribers are launched
   - Parameters for control and target positions are set

2. **Trajectory Generation**
   - Generates smooth joint paths using quintic polynomials

3. **Control Loop**
   - Computes `e_pos`, `e_vel`, and updates `∫ e_pos dt`
   - Calculates torque using PID formula
   - Applies anti-windup to integral term
   - Sends effort commands to the robot

4. **Target Monitoring**
   - Stops control once position error < 0.01 for all joints

---

## 🛠 System Architecture

- **Backends Supported**: Heal Robot (Addverb) and Syncro_240 Robot
- **Control Modes**: Position, Velocity, Effort (Torque)
- **Libraries Used**:
  - Jacobian calculation
  - Grasp matrix computation
  - Quintic trajectory planner
  - Snap detection using ML-based joint velocity data

> The workspace uses a non-traditional structure with separate `build` and `devel` folders for each robot configuration.

---


