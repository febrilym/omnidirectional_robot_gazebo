#!/usr/bin/env python3
"""
Soccer Robot Controller for ROS 1 - FIXED VERSION
"""

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
        self.wheel_distance = rospy.get_param('~wheel_distance', 0.4)
        
        rospy.loginfo(f"Starting Soccer Robot Control for {self.robot_name}")
        rospy.loginfo(f"Wheel radius: {self.wheel_radius}, Wheel distance: {self.wheel_distance}")
        
        # publishers for individual velocity controllers
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
            rospy.loginfo(f"Created publisher for: {topic}")
        
        # init: send null command for certainly robot does'nt move
        self.stop_robot()
        
        # subscribe to cmd_vel
        rospy.Subscriber(f'/{self.robot_name}/cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.loginfo("Soccer Robot Control initialized successfully!")
        
    def stop_robot(self):
        """Mengirimkan command nol ke semua roda"""
        for pub in self.wheel_pubs:
            msg = Float64()
            msg.data = 0.0
            pub.publish(msg)
        rospy.loginfo("Initial stop command sent")
        
    def calculate_wheel_velocities(self, vx, vy, wz):
        """
        Calculate wheel velocities based on URDF wheel orientations.
        Wheel orientations (from URDF):
          first: 225° (5π/4)
          second: 315° (7π/4)
          third: 45° (π/4)
          forth: 135° (3π/4)
        """
        L = self.wheel_distance  # distance from center to wheel (m)
        r = self.wheel_radius    # wheel radius (m)
        
        sqrt2_2 = math.sqrt(2) / 2
        
        # first wheel (225°)
        w1 = (vx * (-sqrt2_2) + vy * (-sqrt2_2) + L * wz) / r
        # second wheel (315°)
        w2 = (vx * (-sqrt2_2) + vy * (sqrt2_2) + L * wz) / r
        # third wheel (45°)
        w3 = (vx * (sqrt2_2) + vy * (sqrt2_2) + L * wz) / r
        # fourth wheel (135°)
        w4 = (vx * (sqrt2_2) + vy * (-sqrt2_2) + L * wz) / r
        
        return np.array([w1, w2, w3, w4], dtype=float)
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages"""
        try:
            vx = msg.linear.x
            vy = msg.linear.y
            wz = msg.angular.z
            
            rospy.loginfo(f"Received cmd_vel: vx={vx}, vy={vy}, wz={wz}")
            
            # if all value is null, stop the robot
            if vx == 0.0 and vy == 0.0 and wz == 0.0:
                self.stop_robot()
                return
            
            # calculate wheel velocities
            wheel_velocities = self.calculate_wheel_velocities(vx, vy, wz)
            
            # publish to individuals controllers
            for i, vel in enumerate(wheel_velocities):
                wheel_msg = Float64()
                wheel_msg.data = vel
                self.wheel_pubs[i].publish(wheel_msg)
            
            # debug logging
            rospy.loginfo(f"Wheel velocities: [{wheel_velocities[0]:.2f}, {wheel_velocities[1]:.2f}, {wheel_velocities[2]:.2f}, {wheel_velocities[3]:.2f}]")
                
        except Exception as e:
            rospy.logerr(f"Error in cmd_vel_callback: {str(e)}")
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Waiting for cmd_vel commands...")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = SoccerRobotControl()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller shutdown requested")
    except Exception as e:
        rospy.logerr(f"Controller error: {str(e)}")