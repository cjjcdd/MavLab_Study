import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import sys
sys.path.append('/home/jj/Desktop/MavLab_Study/ros2_ws/')

from src.student.tut_03 import module_control as con
from src.student.tut_03.module_kinematics import quat_to_eul
from src.student.tut_03.read_input import read_input
import numpy as np

class MAVController(Node):
    def __init__(self, control_mode, control_params):
        super().__init__('mav_controller')
        
        # Store control parameters and initialize control module
        self.control_params = control_params
        con.initialize(control_params)
        
        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'mav_odom',
            self.odom_callback,
            10)
        
        # Create publisher for rudder command
        self.rudder_pub = self.create_publisher(
            Float64,
            'mav_rudder_command',
            10)
        
        # Initialize time
        self.start_time = self.get_clock().now()
        
        # Select control mode
        self.control_mode = control_mode
        
        self.get_logger().info('MAV Controller initialized')

    def odom_callback(self, msg):
        # Get current time in seconds
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9

        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        eul = quat_to_eul(quat, order='ZYX')
        
        # Create state vector from odometry
        # Following the state vector ordering: [x, y, z, φ, θ, ψ, u, v, w, p, q, r, δ]
        state = np.array([
            msg.pose.pose.position.x,      # x
            msg.pose.pose.position.y,      # y
            msg.pose.pose.position.z,      # z
            eul[0],                        # φ (phi)
            eul[1],                        # θ (theta)
            eul[2],                        # ψ (psi)
            msg.twist.twist.linear.x,      # u
            msg.twist.twist.linear.y,      # v
            msg.twist.twist.linear.z,      # w
            msg.twist.twist.angular.x,     # p
            msg.twist.twist.angular.y,     # q
            msg.twist.twist.angular.z,     # r
            0.0                            # δ (rudder angle - will be filled by control)
        ])
        
        # Calculate control command
        rudder_cmd = self.control_mode(t, state)
        
        # Publish command (rudder_cmd is in radians, convert to degrees for consistency)
        cmd_msg = Float64()
        cmd_msg.data = float(rudder_cmd * 180.0 / np.pi)  # Convert to degrees
        self.rudder_pub.publish(cmd_msg)
        
        # Log info
        self.get_logger().info(
            f'\nTime: {t:.2f}s\n'
            f'Rudder Command: {rudder_cmd*180/np.pi:.2f} deg\n'
        )

def main(args=None):
    controller = None
    
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        
        # Path to the input YAML file
        input_path = '/home/jj/Desktop/MavLab_Study/ros2_ws/src/student/tut_03/input.yml'
        
        # Read parameters including control parameters
        vessel_params, hydrodynamic_data, control_params = read_input(input_path)
        
        # Get control type from control parameters
        control_type = str(control_params.get('type', 'fixed')).lower()
        
        # Select appropriate control function
        if control_type == 'switching':
            control_mode = con.switching_rudder
        elif control_type == 'fixed':
            control_mode = con.fixed_rudder
        else:
            raise ValueError(f"Invalid control type: {control_type}")
        
        # Create controller
        controller = MAVController(control_mode, control_params)
        
        # Spin the node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean up resources
        if controller is not None:
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()