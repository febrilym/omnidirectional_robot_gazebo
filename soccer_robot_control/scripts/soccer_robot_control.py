#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class SoccerRobotControl:
    def __init__(self):
        rospy.init_node('soccer_robot_control')
        
        # get parameters
        self.robot_name = rospy.get_param('~robot_name', 'robot_1')
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)
        self.wheel_distance = rospy.get_param('~wheel_distance', 0.2404)  # sqrt(0.17^2 + 0.17^2)
        
        rospy.loginfo(f"Starting Soccer Robot Control for {self.robot_name} . . .")
        rospy.loginfo(f"Wheel radius: {self.wheel_radius}, Wheel distance: {self.wheel_distance:.4f}")
        
        # create publishers for individual wheel velocity controllers
        self.wheel_pubs = []
        wheel_topics = [
            f"/{self.robot_name}/first_wheel_velocity_controller/command",
            f"/{self.robot_name}/second_wheel_velocity_controller/command", 
            f"/{self.robot_name}/third_wheel_velocity_controller/command",
            f"/{self.robot_name}/forth_wheel_velocity_controller/command"
        ]
        
        for topic in wheel_topics:
            pub = rospy.Publisher(topic, Float64, queue_size=10)
            self.wheel_pubs.append(pub)
            rospy.loginfo(f"Created publisher: {topic}")
        
        # send initial stop command
        self.stop_robot()
        
        # subscribe to cmd_vel
        cmd_vel_topic = f'/{self.robot_name}/cmd_vel'
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
        rospy.loginfo(f"Subscribed to: {cmd_vel_topic}")
        
        rospy.loginfo("Soccer Robot Control Initialized")
        
    def stop_robot(self):
        """send zero velocity to all wheels"""
        for pub in self.wheel_pubs:
            msg = Float64()
            msg.data = 0.0
            pub.publish(msg)
        rospy.loginfo_once("Initialized with zero velocity")
    
    def calculate_wheel_velocities_transformed(self, vx, vy, wz):
        """
        kinematics with proper transformation
        """
        L = self.wheel_distance
        r = self.wheel_radius
        
        # transform the inputs:
        transformed_vx = -vy    # swap y to x and reverse
        transformed_vy = -vx    # swap x to y and reverse
        
        # standard omnidirectional kinematics with transformed inputs
        sqrt2_2 = math.sqrt(2) / 2
        
        # wheel orientations from urdf:
        # first: 225° (5π/4) = front-left
        # second: 315° (7π/4) = front-right
        # third: 45° (π/4) = back-right
        # forth: 135° (3π/4) = back-left
        
        w1 = (transformed_vx * (-sqrt2_2) + transformed_vy * (-sqrt2_2) + L * wz) / r
        w2 = (transformed_vx * (-sqrt2_2) + transformed_vy * (sqrt2_2) + L * wz) / r
        w3 = (transformed_vx * (sqrt2_2) + transformed_vy * (sqrt2_2) + L * wz) / r
        w4 = (transformed_vx * (sqrt2_2) + transformed_vy * (-sqrt2_2) + L * wz) / r
        
        return [w1, w2, w3, w4]
    
    def cmd_vel_callback(self, msg):
        """
        teleop keyboard commands properly:
        - without shift: differential mode
        - with shift: holonomic mode
        """
        try:
            # log raw input
            rospy.logdebug(f"Raw cmd_vel: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")
            
            # mode detection and input transformation
            is_holonomic_mode = abs(msg.linear.y) > 0.001
            
            if not is_holonomic_mode:
                vx = msg.linear.x
                vy = 0.0
                wz = msg.angular.z
                mode_str = "NON-SHIFT (differential)"
            else:
                vx = msg.linear.x
                vy = msg.linear.y
                wz = msg.angular.z
                mode_str = "SHIFT (holonomic)"
            
            rospy.loginfo(f"Mode: {mode_str}, Raw: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
            
            # stop robot if all commands are zero
            if abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001:
                self.stop_robot()
                return
            
            # calculate wheel velocities with proper transformation
            wheel_velocities = self.calculate_wheel_velocities_transformed(vx, vy, wz)
            
            # publish to individual wheel controllers
            for i, vel in enumerate(wheel_velocities):
                wheel_msg = Float64()
                wheel_msg.data = vel
                self.wheel_pubs[i].publish(wheel_msg)
            
            # log for debugging
            if rospy.get_time() % 1.0 < 0.05:
                rospy.loginfo(f"Wheel velocities: [{wheel_velocities[0]:.2f}, {wheel_velocities[1]:.2f}, "
                             f"{wheel_velocities[2]:.2f}, {wheel_velocities[3]:.2f}]")
                
        except Exception as e:
            rospy.logerr(f"Error in cmd_vel_callback: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def run(self):
        """main control loop"""
        rospy.loginfo("Waiting for cmd_vel commands . . . ")
        rospy.loginfo("Use teleop keyboard:")
        rospy.loginfo("  WITHOUT Shift: I=forward, ,=backward, J=rotate left, L=rotate right")
        rospy.loginfo("  WITH Shift: L=forward, J=backward, I=right, <=left")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = SoccerRobotControl()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller shutdown requested")
    except Exception as e:
        rospy.logerr(f"Fatal error in controller: {str(e)}")
        import traceback
        rospy.logerr(traceback.format_exc())