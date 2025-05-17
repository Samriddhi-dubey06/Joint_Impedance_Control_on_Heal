import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

Kp = np.array([148, 140, 130, 120, 115, 130])  # Adjusted for proportional control
Kd = np.array([57, 52, 45, 50, 40, 45])       # Adjusted for derivative damping
Ki = np.array([65, 60, 60, 60, 55, 60])       # Adjusted for integral control

# Kp = 140
# Kd = 50
# Ki = 60
   
# Tolerance for stopping movement
tolerance = 0.01  # Smaller tolerance for precise control

# Target joint positions (multiple targets)
target_positions = [
    # [0.2, 0, 0.6, 0.2, 0.0, 0],
    # [-1.2, 0.3, -0.2, 0.5, 0.5, 0.0],
    # [-1.2, 0.3, 0.4, 0.5, 0.5, 0.0],
    # [1.5, 0.3, -0.2, -0.5, 0.0, 0.0],
    # [1.5, 0.3, 0.4, -0.5, -0.5, 0.0],
    #   [-0.8, 0.2, 0, 0, 0, 0],
  [0, 0, 0, 0, 0, 0] ,
#   [-0.8, -0.2, 0, 0, 0, 0]
#     [0, 0.2, 0, 0, 0, 0] ,
]

# Trajectory parameters
trajectory_duration = 10.0  # Total trajectory duration (seconds)

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

    # Update current positions and velocities
    current_joint_positions = np.array(msg.position)
    current_joint_velocities = np.array(msg.velocity)

def compute_pid_torques(target_positions, current_positions, target_velocities, current_velocities, dt):
    """Compute PID torques based on current error."""
    global integral_error

    # Initialize the integral error if it hasn't been initialized yet
    if integral_error is None:
        integral_error = np.zeros(len(target_positions))

    # Compute errors
    position_error = target_positions - current_positions
    velocity_error = target_velocities - current_velocities

    # Update integral error (integrate position error)
    integral_error += position_error * dt

     # Clamp the integral term to prevent windup
    integral_clamp = 50.0  # Adjust based on your system's characteristics
    integral_error = np.clip(integral_error, -integral_clamp, integral_clamp)

    # Compute PID torques
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

def generate_quintic_trajectory(start_positions, target_positions, duration):
    """Generate a quintic trajectory with positions, velocities, and accelerations."""
    time_steps = np.linspace(0, duration, int(duration * control_loop_rate))
    positions = []
    velocities = []
    accelerations = []

    for i in range(len(start_positions)):
        coeffs = compute_quintic_coefficients(
            start_positions[i], 0, 0, target_positions[i], 0, 0, duration
        )
        pos = [np.polyval(coeffs[::-1], t) for t in time_steps]
        vel = [np.polyval(np.polyder(coeffs[::-1], 1), t) for t in time_steps]
        acc = [np.polyval(np.polyder(coeffs[::-1], 2), t) for t in time_steps]

        positions.append(pos)
        velocities.append(vel)
        accelerations.append(acc)

    # Return arrays of shape [time_steps, joints]
    return np.array(positions).T, np.array(velocities).T, np.array(accelerations).T

def publish_joint_accelerations(accelerations):
    """Publish the computed joint accelerations."""
    acceleration_msg = Float64MultiArray()
    acceleration_msg.data = accelerations.tolist()
    acceleration_publisher.publish(acceleration_msg)

def move_to_target(target, effort_publisher, rate):
    """Move from the current position to the given target position."""
    global current_joint_positions, current_joint_velocities

    # Generate quintic trajectory for this target
    trajectory_positions, trajectory_velocities, trajectory_accelerations = generate_quintic_trajectory(
        current_joint_positions, target, trajectory_duration
    )

    rospy.loginfo(f"Moving to target: {target}")
    total_steps = len(trajectory_positions)
    target_reached = False

    for step_idx in range(total_steps):
        if rospy.is_shutdown():
            return True, True  # Shutdown requested

        target_position = trajectory_positions[step_idx]
        target_velocity = trajectory_velocities[step_idx]
        target_acceleration = trajectory_accelerations[step_idx]

        # Compute position error
        position_error = np.linalg.norm(target_position - current_joint_positions)

        # Check if target is reached after 90% of the trajectory
        if step_idx > int(0.9 * total_steps):
            if position_error < tolerance:
                target_reached = True
                rospy.loginfo(f"Target reached within tolerance: {position_error}")
                break

        # Compute time step (dt)
        dt = 1.0 / control_loop_rate

        # Compute PID torques
        pid_torques = compute_pid_torques(target_position, current_joint_positions, target_velocity, current_joint_velocities, dt)

        # Publish torques
        effort_msg = Float64MultiArray()
        effort_msg.data = pid_torques.tolist()
        effort_publisher.publish(effort_msg)

        # Publish accelerations from the trajectory
        publish_joint_accelerations(target_acceleration)

        rate.sleep()

    rospy.loginfo("Stopping movement, maintaining final position as spring-damper.")
     # Allow the control loop to move to the next target
    return target_reached, False
    # # Maintain position as a spring-damper
    # while not rospy.is_shutdown():
    #     pid_torques = compute_pid_torques(target, current_joint_positions, np.zeros_like(current_joint_velocities), current_joint_velocities, dt)
    #     effort_msg = Float64MultiArray()
    #     effort_msg.data = pid_torques.tolist()
    #     effort_publisher.publish(effort_msg)
    #     rate.sleep()

def control_loop():
    # Initialize ROS node
    rospy.init_node('compute_torques', anonymous=True)

    # Subscribers and publishers
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    global acceleration_publisher
    acceleration_publisher = rospy.Publisher('/joint_accelerations', Float64MultiArray, queue_size=10)
    effort_publisher = rospy.Publisher('/effort_controller/command', Float64MultiArray, queue_size=10)

    # Wait for joint state data to arrive
    rospy.loginfo("Waiting for joint state data...")
    rate = rospy.Rate(control_loop_rate)
    while current_joint_positions is None or current_joint_velocities is None:
        if rospy.is_shutdown():
            return
        rate.sleep()

    # Move through each target in sequence
    for idx, target in enumerate(target_positions):
        reached, shutdown = move_to_target(np.array(target), effort_publisher, rate)
        if shutdown:
            return

    rospy.loginfo("All targets processed. Maintaining spring-damper behavior at the final position.")

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("PID control node terminated.")
