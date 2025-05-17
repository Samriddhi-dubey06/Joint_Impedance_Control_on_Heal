import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# PID Gains
Kp = np.array([148, 140, 130, 120, 115, 130])  # Proportional control gains
Kd = np.array([57, 52, 45, 50, 40, 45])       # Derivative control gains
Ki = np.array([65, 60, 60, 60, 55, 60])       # Integral control gains

# Velocity Limits (maximum allowable velocity for each joint)
vmax = np.array([0.18, 0.18, 0.18, 0.18, 0.18, 0.18])  

# Tolerance for stopping movement
tolerance = 0.01  

# Target joint positions (multiple targets)
target_positions = [
    [0, 0, 0, 0, 0, 0],
    # [-0.8, -0.2, 0, 0, 0, 0]
]

# Control loop rate (Hz)
control_loop_rate = 500

# Current state
current_joint_positions = None
current_joint_velocities = None

# Integral term storage
integral_error = None

def joint_state_callback(msg):
    """Callback to update current joint positions and velocities."""
    global current_joint_positions, current_joint_velocities
    current_joint_positions = np.array(msg.position)
    current_joint_velocities = np.array(msg.velocity)

def compute_pid_torques(target_positions, current_positions, target_velocities, current_velocities, dt):
    """Compute PID torques based on current error."""
    global integral_error
    if integral_error is None:
        integral_error = np.zeros(len(target_positions))

    position_error = target_positions - current_positions
    velocity_error = target_velocities - current_velocities

    integral_error += position_error * dt
    integral_clamp = 50.0  # Prevent integral windup
    integral_error = np.clip(integral_error, -integral_clamp, integral_clamp)

    pid_torques = (Kp * position_error) + (Kd * velocity_error) + (Ki * integral_error)
    return pid_torques

def compute_quintic_coefficients(p0, v0, a0, pf, vf, af, T):
    """Compute the coefficients of a quintic polynomial for trajectory generation."""
    A = np.array([
        [1, 0,   0,    0,     0,      0],
        [0, 1,   0,    0,     0,      0],
        [0, 0,   2,    0,     0,      0],
        [1, T,   T**2, T**3,  T**4,   T**5],
        [0, 1,   2*T,  3*T**2,4*T**3, 5*T**4],
        [0, 0,   2,    6*T,   12*T**2,20*T**3],
    ])
    b = np.array([p0, v0, a0, pf, vf, af])
    coeffs = np.linalg.solve(A, b)
    return coeffs

def generate_quintic_trajectory(start_positions, target_positions):
    """Generate a quintic trajectory with positions, velocities, and accelerations."""
    # Compute time dynamically
    duration = np.max(np.abs(target_positions - start_positions) / vmax)
    time_steps = np.linspace(0, duration, int(duration * control_loop_rate))

    positions, velocities, accelerations = [], [], []
    for i in range(len(start_positions)):
        coeffs = compute_quintic_coefficients(start_positions[i], 0, 0, target_positions[i], 0, 0, duration)
        pos = [np.polyval(coeffs[::-1], t) for t in time_steps]
        vel = [np.polyval(np.polyder(coeffs[::-1], 1), t) for t in time_steps]
        acc = [np.polyval(np.polyder(coeffs[::-1], 2), t) for t in time_steps]
        positions.append(pos)
        velocities.append(vel)
        accelerations.append(acc)

    return np.array(positions).T, np.array(velocities).T, np.array(accelerations).T, duration

def move_to_target(target, effort_publisher, rate):
    """Move from the current position to the given target position."""
    global current_joint_positions, current_joint_velocities

    trajectory_positions, trajectory_velocities, trajectory_accelerations, duration = generate_quintic_trajectory(
        current_joint_positions, target
    )
    rospy.loginfo(f"Moving to target {target} with duration {duration:.2f}s")

    total_steps = len(trajectory_positions)
    for step_idx in range(total_steps):
        if rospy.is_shutdown():
            return True, True

        target_position = trajectory_positions[step_idx]
        target_velocity = trajectory_velocities[step_idx]

        if step_idx > int(0.9 * total_steps) and np.linalg.norm(target_position - current_joint_positions) < tolerance:
            rospy.loginfo("Target reached within tolerance.")
            return True, False

        dt = 1.0 / control_loop_rate
        pid_torques = compute_pid_torques(target_position, current_joint_positions, target_velocity, current_joint_velocities, dt)
        
        effort_msg = Float64MultiArray()
        effort_msg.data = pid_torques.tolist()
        effort_publisher.publish(effort_msg)
        rate.sleep()

    return False, False

def control_loop():
    rospy.init_node('compute_torques', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    effort_publisher = rospy.Publisher('/effort_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(control_loop_rate)

    rospy.loginfo("Waiting for joint state data...")
    while current_joint_positions is None or current_joint_velocities is None:
        if rospy.is_shutdown():
            return
        rate.sleep()

    for target in target_positions:
        reached, shutdown = move_to_target(np.array(target), effort_publisher, rate)
        if shutdown:
            return

    rospy.loginfo("All targets reached. Maintaining position.")

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("PID control node terminated.")