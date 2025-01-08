import rclpy
import time
import threading

import numpy as np
from youbot_kinematics.youbotKineBase import YoubotKinematicBase
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS


class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        # Set the offset for theta
        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]

        # Apply joint offsets to dh parameters
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        # Joint reading polarity signs
        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        assert isinstance(self.dh_params, dict)
        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        assert up_to_joint >= 0
        assert up_to_joint <= len(self.dh_params['a'])

        T = np.identity(4)

        # Apply offset and polarity to joint readings (found in URDF file)
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)
            
        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        print("Forward: ", T)
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        T = np.eye(4)  # Base frame transformation
        joint_positions = []
        z_axes = []

        # Compute forward kinematics to get positions and z-axes for each joint
        for i in range(5):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joint[i])
            T = T @ A
            joint_positions.append(T[:3, 3])  # Extract position
            z_axes.append(T[:3, 2])  # Extract Z-axis

        # End-effector position
        end_effector_pos = joint_positions[-1]

        # Initialize Jacobian matrix (6x5)
        jacobian = np.zeros((6, 5))

        for i in range(5):
            # Linear velocity component (cross product of Z-axis and vector to end-effector)
            jacobian[:3, i] = np.cross(z_axes[i], end_effector_pos - joint_positions[i])
            # Angular velocity component (Z-axis for revolute joints)
            jacobian[3:, i] = z_axes[i]

        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # Compute Jacobian matrix
        jacobian = self.get_jacobian(joint)

        # Check if the determinant of the Jacobian's top 3x3 matrix (linear velocity) is near zero
        linear_jacobian = jacobian[:3, :]
        rank = np.linalg.matrix_rank(linear_jacobian)

        singularity = rank < 3  # Singular if rank is less than 3

        assert isinstance(singularity, bool)
        return singularity

def inverse_kinematics(kinematics, initial_joints, target_pose, max_iters=1000, tol=1e-6, damping=0.01):
    """
    Perform inverse kinematics using the Jacobian with joint limits.

    Args:
        kinematics: Instance of YoubotKinematicStudent for FK and Jacobian.
        initial_joints: Initial guess for the joint angles (list of 5).
        target_pose: Desired 4x4 transformation matrix (np.ndarray).
        max_iters: Maximum number of iterations (int).
        tol: Tolerance for stopping condition (float).
        damping: Damping factor for DLS (float).

    Returns:
        final_joints: Joint angles that achieve the target pose (list of 5).
    """
    # Joint limits in radians (you may need to adjust these for your robot)
    JOINT_LIMITS = [
        (0, 5.90),  # Joint 1
        (0, 1.76),  # Joint 2
        (-5.18, 0),  # Joint 3
        (0, 3.58),  # Joint 4
        (0, 5.85)   # Joint 5
    ]

    def enforce_joint_limits(joints):
        """Clamp joint angles to their valid limits."""
        return [np.clip(joint, limit[0], limit[1]) for joint, limit in zip(joints, JOINT_LIMITS)]

    def is_target_reachable(target_pose, max_reach):
        """Check if the target pose is within the robot's workspace."""
        target_position = target_pose[:3, 3]
        distance = np.linalg.norm(target_position)
        return distance <= max_reach

    max_reach = 1.0  # Example maximum reach of the robot (adjust as necessary)

    # Check if the target pose is reachable
    if not is_target_reachable(target_pose, max_reach):
        print("Target pose is outside the workspace!")
        return None

    joints = np.array(initial_joints)

    for i in range(max_iters):
        # Convert `joints` to a list for `forward_kinematics`
        current_pose = kinematics.forward_kinematics(list(joints))
        current_position = current_pose[:3, 3]
        current_orientation = current_pose[:3, :3]  # Rotation matrix

        # Compute error in position
        target_position = target_pose[:3, 3]
        position_error = target_position - current_position

        # Compute error in orientation (convert rotation matrices to axis-angle)
        target_orientation = target_pose[:3, :3]
        orientation_error_matrix = np.dot(target_orientation, current_orientation.T)
        orientation_error = np.array([
            orientation_error_matrix[2, 1] - orientation_error_matrix[1, 2],
            orientation_error_matrix[0, 2] - orientation_error_matrix[2, 0],
            orientation_error_matrix[1, 0] - orientation_error_matrix[0, 1]
        ]) / 2.0

        # Combine position and orientation error
        error = np.hstack((position_error, orientation_error))

        # Check stopping condition
        if np.linalg.norm(error) < tol:
            print(f"Converged in {i} iterations.")
            return joints.tolist()

        # Compute Jacobian
        jacobian = kinematics.get_jacobian(list(joints))

        # Compute damped least squares solution
        J_damped = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + damping**2 * np.eye(6))
        delta_theta = J_damped @ error

        # Scale step size to prevent large updates
        max_delta = 0.1  # Max radians per step
        if np.linalg.norm(delta_theta) > max_delta:
            delta_theta = delta_theta * (max_delta / np.linalg.norm(delta_theta))

        # Update joint angles
        joints += delta_theta

        # Enforce joint limits
        joints = enforce_joint_limits(joints)

    print("Failed to converge within max iterations.")
    return list(joints)






def main(args=None):
    rclpy.init(args=args)

    kinematic_student = YoubotKinematicStudent()
    initial_joints = [2.95, 1.35, -2.59, 0.0, 0.0]  # Initial guess
    target_pose = np.eye(4)  # Example: Desired pose (identity matrix)
    target_pose[:3, 3] = [-0.0477, -0.1385, 0.4826]  # Desired position

    for i in range(TARGET_JOINT_POSITIONS.shape[0]):
        target_joint_angles = TARGET_JOINT_POSITIONS[i]
        target_joint_angles = target_joint_angles.tolist()
        pose = kinematic_student.forward_kinematics(target_joint_angles)
        # we would probably compute the jacobian at our current joint angles, not the target
        # but this is just to check your work
        jacobian = kinematic_student.get_jacobian(target_joint_angles)
        print("target joint angles")
        print(target_joint_angles)
        print("pose")
        print(pose)
        print("jacobian")
        print(jacobian)

    rclpy.spin(kinematic_student)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kinematic_student.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()