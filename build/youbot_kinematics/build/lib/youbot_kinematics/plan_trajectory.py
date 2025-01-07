import rclpy
from rclpy.node import Node
from scipy.linalg import expm
from scipy.linalg import logm
from itertools import permutations
import time
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

import numpy as np
from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS


class YoubotTrajectoryPlanning(Node):
    def __init__(self):
        # Initialize node
        super().__init__('youbot_trajectory_planner')

        # Save question number for check in main run method
        self.kdl_youbot = YoubotKinematicStudent()

        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        self.traj_pub = self.create_publisher(JointTrajectory, '/EffortJointInterface_trajectory_controller/command',
                                        50)
        self.checkpoint_pub = self.create_publisher(Marker, "checkpoint_positions", 500)

    def run(self):
        """This function is the main run function of the class. When called, it runs question 6 by calling the q6()
        function to get the trajectory. Then, the message is filled out and published to the /command topic.
        """
        print("run q6a")
        self.get_logger().info("Waiting 5 seconds for everything to load up.")
        time.sleep(5.0)
        traj = self.q6()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        print("Trajectory: ", traj)
        self.traj_pub.publish(traj)

    def q6(self):
        # Load target positions and convert to Cartesian space
        target_tfs, target_joints = self.load_targets()
        
        # Find shortest path through waypoints
        sorted_order, min_dist = self.get_shortest_path(target_tfs)
        self.get_logger().info(f'Shortest path order: {sorted_order}, distance: {min_dist}')
        
        # Generate intermediate waypoints (10 points between each checkpoint)
        full_tfs = self.intermediate_tfs(sorted_order, target_tfs, 3)
        
        # Publish trajectory for visualization
        self.publish_traj_tfs(full_tfs)
        
        # Convert Cartesian trajectory to joint space
        joint_trajectory = self.full_checkpoints_to_joints(full_tfs, self.kdl_youbot.current_joint_position)
        
        # Create trajectory message
        traj = JointTrajectory()
        
        # Add points to trajectory
        time_per_point = 0.5  # seconds
        for i in range(joint_trajectory.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = joint_trajectory[:,i].tolist()
            point.time_from_start.sec = int(i * time_per_point)
            point.time_from_start.nanosec = int((i * time_per_point % 1) * 1e9)
            traj.points.append(point)
        
        return traj



    def load_targets(self):
        num_target_positions = len(TARGET_JOINT_POSITIONS)
        target_joint_positions = np.zeros((5, num_target_positions + 1))
        target_cart_tf = np.repeat(np.identity(4), num_target_positions + 1, axis=1).reshape((4, 4, num_target_positions + 1))
        
        # Set initial position
        target_joint_positions[:, 0] = self.kdl_youbot.current_joint_position
        target_cart_tf[:, :, 0] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, 0].tolist())
        
        # Load remaining target positions
        for i in range(num_target_positions):
            target_joint_positions[:, i+1] = TARGET_JOINT_POSITIONS[i]
            target_cart_tf[:, :, i+1] = self.kdl_youbot.forward_kinematics(TARGET_JOINT_POSITIONS[i].tolist())
        
        return target_cart_tf, target_joint_positions
        

    def get_shortest_path(self, checkpoints_tf):
        num_checkpoints = checkpoints_tf.shape[2]
        # Start from current position (index 0)
        checkpoint_indices = list(range(1, num_checkpoints))
        
        min_dist = float('inf')
        sorted_order = None
        
        # Try all possible permutations of checkpoint orders
        for perm in permutations(checkpoint_indices):
            # Add start position to beginning
            full_path = [0] + list(perm)
            total_dist = 0
            
            # Calculate total distance for this permutation
            for i in range(len(full_path)-1):
                pos1 = checkpoints_tf[0:3, 3, full_path[i]]
                pos2 = checkpoints_tf[0:3, 3, full_path[i+1]]
                total_dist += np.linalg.norm(pos2 - pos1)
                
            if total_dist < min_dist:
                min_dist = total_dist
                sorted_order = np.array(full_path)
        
        return sorted_order, min_dist


    def publish_traj_tfs(self, tfs):
        """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        Cartesian path of the robot end-effector.
        Args:
            tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
        """
        id = 0
        for i in range(0, tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0 + id * 0.05
            marker.color.b = 1.0 - id * 0.05
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = tfs[0, -1, i]
            marker.pose.position.y = tfs[1, -1, i]
            marker.pose.position.z = tfs[2, -1, i]
            self.checkpoint_pub.publish(marker)

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        num_segments = len(sorted_checkpoint_idx) - 1
        total_points = num_segments * num_points + len(sorted_checkpoint_idx)
        full_checkpoint_tfs = np.zeros((4, 4, total_points))
        
        point_idx = 0
        # For each pair of consecutive checkpoints
        for i in range(num_segments):
            start_idx = sorted_checkpoint_idx[i]
            end_idx = sorted_checkpoint_idx[i+1]
            
            # Get intermediate transforms
            segment_tfs = self.decoupled_rot_and_trans(
                target_checkpoint_tfs[:,:,start_idx],
                target_checkpoint_tfs[:,:,end_idx],
                num_points
            )
            
            # Add checkpoint and intermediate points
            full_checkpoint_tfs[:,:,point_idx] = target_checkpoint_tfs[:,:,start_idx]
            point_idx += 1
            
            for j in range(num_points):
                full_checkpoint_tfs[:,:,point_idx] = segment_tfs[:,:,j]
                point_idx += 1
        
        # Add final checkpoint
        full_checkpoint_tfs[:,:,point_idx] = target_checkpoint_tfs[:,:,sorted_checkpoint_idx[-1]]
        
        return full_checkpoint_tfs


    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        tfs = np.zeros((4, 4, num_points))
        
        # Extract positions
        pos_a = checkpoint_a_tf[0:3, 3]
        pos_b = checkpoint_b_tf[0:3, 3]
        
        # Extract rotation matrices
        rot_a = checkpoint_a_tf[0:3, 0:3]
        rot_b = checkpoint_b_tf[0:3, 0:3]
        
        # Compute rotation matrix logarithm
        rot_a_log = logm(rot_a)
        rot_b_log = logm(rot_b)
        
        # Generate intermediate points
        for i in range(num_points):
            s = (i + 1) / (num_points + 1)
            
            # Linear interpolation of position
            pos_i = (1 - s) * pos_a + s * pos_b
            
            # SLERP for rotation
            rot_i_log = (1 - s) * rot_a_log + s * rot_b_log
            rot_i = expm(rot_i_log)
            
            # Combine into transformation matrix
            tfs[:3, :3, i] = rot_i
            tfs[:3, 3, i] = pos_i
            tfs[3, 3, i] = 1.0
        
        return tfs


    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        num_points = full_checkpoint_tfs.shape[2]
        q_checkpoints = np.zeros((5, num_points))
        
        # Use previous solution as initial guess for next point
        q_prev = init_joint_position
        
        for i in range(num_points):
            q, error = self.ik_position_only(full_checkpoint_tfs[:,:,i], q_prev)
            q_checkpoints[:,i] = q
            q_prev = q
            
            if error > 0.01:  # 1cm threshold
                self.get_logger().warning(f'High IK error at point {i}: {error}m')
        
        return q_checkpoints



    def ik_position_only(self, pose, q0, lam=0.25, num=5000):
        q = q0.copy()
        
        for i in range(num):
            # Get current end-effector position
            current_tf = self.kdl_youbot.forward_kinematics(q.tolist())
            current_pos = current_tf[0:3, 3]
            
            # Compute position error
            target_pos = pose[0:3, 3]
            error_pos = target_pos - current_pos
            
            # Early exit if error is small enough
            error_magnitude = np.linalg.norm(error_pos)
            if error_magnitude < 1e-4:
                return q, error_magnitude
                
            # Get Jacobian and extract position components
            J = self.kdl_youbot.get_jacobian(q.tolist())
            J_pos = J[0:3, :]  # Position part only
            
            # Compute joint updates using damped least squares
            JJt = J_pos @ J_pos.T
            damping = lam * np.eye(3)
            delta_q = J_pos.T @ np.linalg.inv(JJt + damping) @ error_pos
            
            # Update joint positions
            q = q + delta_q
        
        # Return final joint positions and error
        final_tf = self.kdl_youbot.forward_kinematics(q.tolist())
        final_error = np.linalg.norm(pose[0:3, 3] - final_tf[0:3, 3])
        return q, final_error




def main(args=None):
    rclpy.init(args=args)

    youbot_planner = YoubotTrajectoryPlanning()

    youbot_planner.run()

    rclpy.spin(youbot_planner)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    youbot_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()