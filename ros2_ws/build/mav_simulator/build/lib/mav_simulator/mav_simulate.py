import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

import sys
sys.path.append('/home/jj/Desktop/MavLab_Study/ros2_ws/')

from src.student.tut_03.class_vessel import Vessel
from src.student.tut_03.module_kinematics import eul_to_quat
from src.student.tut_03.read_input import read_input
import numpy as np

class MavNode(Node):
    def __init__(self, vessel_params, hydrodynamic_data, control_params):
        # Initialize the node
        super().__init__('mav_node')

        # Initialize the vessel with all required parameters
        self.vessel = Vessel(vessel_params, hydrodynamic_data, control_params)
        
        # Get the time step from simulation parameters
        self.dt = vessel_params.get('sim', {}).get('dt', 0.1)  # Default to 0.1 seconds if not found
        
        # Initialize vessel simulation state
        self.vessel.current_state = self.vessel.initial_state.copy()
        self.vessel.t = 0.0  # Initialize time
        
        # Create a publisher to publish the vessel odometry
        self.publisher = self.create_publisher(Odometry, 'mav_odom', 10)

        # Create a publisher to publish the rudder angle
        self.rudder_publisher = self.create_publisher(Float64, 'mav_rudder', 10)

        # Create a subscriber to receive the rudder angle
        self.rudder_subscriber = self.create_subscription(Float64, 'mav_rudder_command', self.rudder_callback, 10)

        # Create a timer to call the timer_callback function at the vessel time step
        self.timer = self.create_timer(self.dt, self.timer_callback)        

    def timer_callback(self):
        
        # Integrate the vessel forward in time using Euler integration
        # Get the current state derivative
        state_dot = self.vessel.vessel_ode(self.vessel.t, self.vessel.current_state)
        
        # Update the state using Euler integration
        self.vessel.current_state = self.vessel.current_state + self.dt * state_dot
        
        # Update the time
        self.vessel.t += self.dt
        
        # Create an odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.vessel.current_state[0]  # x position
        odom_msg.pose.pose.position.y = self.vessel.current_state[1]  # y position
        odom_msg.pose.pose.position.z = self.vessel.current_state[2]  # z position
        
        # Set orientation from Euler angles
        q = eul_to_quat(self.vessel.current_state[3:6], order='ZYX', deg=False)
        odom_msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        
        # Set linear velocity
        odom_msg.twist.twist.linear.x = self.vessel.current_state[6]   # u
        odom_msg.twist.twist.linear.y = self.vessel.current_state[7]   # v
        odom_msg.twist.twist.linear.z = self.vessel.current_state[8]   # w
        
        # Set angular velocity
        odom_msg.twist.twist.angular.x = self.vessel.current_state[9]  # p
        odom_msg.twist.twist.angular.y = self.vessel.current_state[10] # q
        odom_msg.twist.twist.angular.z = self.vessel.current_state[11] # r
        
        # Publish the odometry message
        self.publisher.publish(odom_msg)

        # Publish rudder angle
        rudder_msg = Float64()
        rudder_msg.data = self.vessel.current_state[12] * 180 / np.pi  # Convert to degrees
        self.rudder_publisher.publish(rudder_msg)

        # Log info
        self.get_logger().info(
            f'\nTime: {self.vessel.t:.2f}s\n'
            f'Position (x,y,z): [{self.vessel.current_state[0]:.2f}, {self.vessel.current_state[1]:.2f}, {self.vessel.current_state[2]:.2f}]\n'
            f'Euler angles (phi,theta,psi): [{self.vessel.current_state[3]*180/np.pi:.2f}, {self.vessel.current_state[4]*180/np.pi:.2f}, {self.vessel.current_state[5]*180/np.pi:.2f}]\n'
            f'Velocity (u,v,w): [{self.vessel.current_state[6]:.2f}, {self.vessel.current_state[7]:.2f}, {self.vessel.current_state[8]:.2f}]\n'
            f'Angular velocity (p,q,r): [{self.vessel.current_state[9]:.2f}, {self.vessel.current_state[10]:.2f}, {self.vessel.current_state[11]:.2f}]\n'
            f'Rudder Command: {self.vessel.current_state[12] * 180 / np.pi:.2f} deg\n'
        )

    def rudder_callback(self, msg):
        # Set the commanded rudder angle by updating the control parameters
        # This assumes the control system can accept dynamic amplitude changes
        # If the control type is 'fixed', update the amplitude
        if hasattr(self.vessel, 'control_params'):
            self.vessel.control_params['amplitude'] = np.radians(msg.data)  # Convert from degrees to radians

def main(args=None):
    vessel_node = None  # Initialize variable to avoid UnboundLocalError
    
    try:
        # Path to the input YAML file
        input_path = '/home/jj/Desktop/MavLab_Study/ros2_ws/src/student/tut_03/input.yml'

        # Initialize the ROS 2 node
        rclpy.init(args=args)

        # Read the vessel parameters and hydrodynamic data from the YAML file
        vessel_params, hydrodynamic_data, control_params = read_input(input_path)

        # Create the vessel node
        vessel_node = MavNode(vessel_params, hydrodynamic_data, control_params)

        # Spin the node (keep it running)
        rclpy.spin(vessel_node)

    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean up resources
        if vessel_node is not None:
            vessel_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()