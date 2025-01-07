#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent

import numpy as np

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        print("Creating Subscriber")
        # Create subscriber for the trajectory
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/EffortJointInterface_trajectory_controller/command',
            self.trajectory_callback,
            50
        )
        
        # Create publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'trajectory_visualization',
            10
        )
        
        # Initialize YouBot kinematics
        self.kdl_youbot = YoubotKinematicStudent()
        
        self.get_logger().info('Trajectory visualizer node started')
        
    def trajectory_callback(self, msg):
        self.get_logger().info('Received trajectory message')
        
        # Create marker array message
        marker_array = MarkerArray()
        
        # Create line strip marker for the trajectory
        line_marker = Marker()
        line_marker.header.frame_id = "base_link"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "trajectory_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # Set the line properties
        line_marker.scale.x = 0.02  # line width
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        # Create point markers for each trajectory point
        points = []
        for i, point in enumerate(msg.points):
            # Convert joint positions to end effector position
            print("Positions: ", point.positions)
            tf_matrix = self.kdl_youbot.forward_kinematics(list(point.positions))
            ee_position = tf_matrix[0:3, 3]
            
            # Add point to line strip
            p = Point()
            p.x = float(ee_position[0])
            p.y = float(ee_position[1])
            p.z = float(ee_position[2])
            line_marker.points.append(p)
            
            # Create sphere marker for each point
            point_marker = Marker()
            point_marker.header.frame_id = "base_link"
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = "trajectory_points"
            point_marker.id = i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            
            # Set the sphere properties
            point_marker.pose.position.x = float(ee_position[0])
            point_marker.pose.position.y = float(ee_position[1])
            point_marker.pose.position.z = float(ee_position[2])
            point_marker.pose.orientation.w = 1.0
            
            point_marker.scale.x = 0.03
            point_marker.scale.y = 0.03
            point_marker.scale.z = 0.03
            
            # Color points based on time (gradient from blue to red)
            t = float(i) / len(msg.points)
            point_marker.color.r = t
            point_marker.color.b = 1.0 - t
            point_marker.color.a = 1.0
            
            marker_array.markers.append(point_marker)
        
        # Add line marker to array
        marker_array.markers.append(line_marker)
        
        # Publish the marker array
        print("Marker_array: ", (len(marker_array.markers)))
        self.marker_pub.publish(marker_array)
        self.get_logger().info('Published visualization markers')

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = TrajectoryVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()