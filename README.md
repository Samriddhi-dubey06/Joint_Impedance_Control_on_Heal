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


# Robot Joint Control System (Max Velocity-Based Impedance Control)

This project implements a **joint impedance controller** for a robotic arm using a trajectory planning approach based on **maximum joint velocities** instead of a fixed duration. The controller ensures compliant, smooth, and velocity-safe motion using a combination of PID-based torque control and quintic polynomial trajectory generation.

---

## 🧠 Control Strategy

The controller uses a **PID torque control** structure enhanced with impedance behavior:

1. **Position Control**:  
   Applies a restoring torque proportional to the error between current and desired joint positions.

2. **Velocity Control**:  
   Adds damping torque proportional to the joint velocity to suppress oscillations.

3. **Integral Control**:  
   Integrates the position error over time to help overcome static friction and ensure convergence, especially for small errors.

   \[
   \tau = K_p \cdot e_{pos} + K_d \cdot e_{vel} + K_i \cdot \int e_{pos} dt
   \]

Where:
- \( \tau \): computed joint torques  
- \( e_{pos} \): joint position error  
- \( e_{vel} \): joint velocity error  
- \( K_p, K_d, K_i \): gain matrices

> Integral error is clamped to avoid windup and instability.

---

## 🧮 Max Velocity-Based Trajectory Planning

Unlike fixed-duration trajectory generation, this method computes **individual trajectory times** for each joint based on the required travel distance and the **maximum allowable joint velocity**.

### Step 1: Calculate Individual Joint Times

\[
T_i = \frac{|q_{target}^i - q_{start}^i|}{v_{max}^i}
\]

Where:
- \( q_{target}^i \): target position of joint *i*  
- \( q_{start}^i \): current position of joint *i*  
- \( v_{max}^i \): maximum allowed velocity of joint *i*  
- \( T_i \): required time for joint *i* to reach the target safely

### Step 2: Use Max of All \( T_i \) as Global Duration

\[
T = \max(T_1, T_2, ..., T_n)
\]

This ensures no joint exceeds its speed limit while all joints synchronize.

---

## 📈 Quintic Trajectory Generation

Generates smooth, synchronized joint paths using 5th-order polynomials that ensure:

- Zero initial and final velocities  
- Zero initial and final accelerations  
- Smooth acceleration/deceleration (no jerk)

\[
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
\]

Coefficients are computed using boundary conditions at \( t = 0 \) and \( t = T \) for position, velocity, and acceleration.

---

## ⚙️ Features

- 🔁 Dynamic duration computation using per-joint distance and max velocity  
- 🤖 Compatible with Heal Robot and Syncro_240 Robot  
- 🎯 Target-aware tracking with termination on close convergence  
- 💡 Control loop at **500 Hz** for high responsiveness  
- 📊 Smooth transitions using quintic polynomial curves  
- 🔒 Safety: no joint exceeds velocity limits  
- 🧠 Anti-windup strategy for integral term  

---

## 🚀 How to Run

### 1. Clone and Build

```bash
git clone https://github.com/yourusername/joint_impedance_control_onheal.git
cd joint_impedance_control_onheal
catkin build
source devel/setup.bash
