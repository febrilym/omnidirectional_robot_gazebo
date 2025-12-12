#!/usr/bin/env python3
"""
Soccer Robot Controller for ROS 1
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class SoccerRobotControl:
    def __init__(self):
        rospy.init_node('soccer_robot_control')
        
        # Get parameters
        self.robot_name = rospy.get_param('~robot_name', 'robot_1')
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)
        self.wheel_distance = rospy.get_param('~wheel_distance', 0.4)
        
        rospy.loginfo(f"Starting Soccer Robot Control for {self.robot_name}")
        rospy.loginfo(f"Wheel radius: {self.wheel_radius}, Wheel distance: {self.wheel_distance}")
        
        # Publishers untuk INDIVIDUAL velocity controllers
        self.wheel_pubs = []
        wheel_topics = [
            f"/first_wheel_velocity_controller/command",
            f"/second_wheel_velocity_controller/command", 
            f"/third_wheel_velocity_controller/command",
            f"/forth_wheel_velocity_controller/command"
        ]
        
        for topic in wheel_topics:
            pub = rospy.Publisher(topic, Float64, queue_size=10)
            self.wheel_pubs.append(pub)
            rospy.loginfo(f"Created publisher for: {topic}")
        
        # Subscribe to cmd_vel
        rospy.Subscriber(f'/{self.robot_name}/cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.loginfo("Soccer Robot Control initialized successfully!")
        
    def calculate_wheel_velocities(self, vx, vy, wz):
        """
        Calculate wheel velocities - SAME AS ROS 2 REFERENCE
        """
        # Convert angular velocity (match ROS 2 calculation)
        angular_vel = wz * self.wheel_radius
        
        # Wheel 1: 45 degrees
        w1 = (vx * math.sin(math.pi/4) + 
              vy * math.cos(math.pi/4) + 
              self.wheel_distance * angular_vel) / self.wheel_radius
        
        # Wheel 2: 135 degrees (45 + 90)
        w2 = (vx * math.sin(math.pi/4 + math.pi/2) + 
              vy * math.cos(math.pi/4 + math.pi/2) + 
              self.wheel_distance * angular_vel) / self.wheel_radius
        
        # Wheel 3: 225 degrees (45 + 180)
        w3 = (vx * math.sin(math.pi/4 - math.pi) + 
              vy * math.cos(math.pi/4 - math.pi) + 
              self.wheel_distance * angular_vel) / self.wheel_radius
        
        # Wheel 4: 315 degrees (45 - 90)
        w4 = (vx * math.sin(math.pi/4 - math.pi/2) + 
              vy * math.cos(math.pi/4 - math.pi/2) + 
              self.wheel_distance * angular_vel) / self.wheel_radius
        
        return np.array([w1, w2, w3, w4], dtype=float)
    
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages"""
        try:
            vx = msg.linear.x
            vy = msg.linear.y
            wz = msg.angular.z
            
            # Calculate wheel velocities
            wheel_velocities = self.calculate_wheel_velocities(vx, vy, wz)
            
            # Publish ke INDIVIDUAL controllers
            for i, vel in enumerate(wheel_velocities):
                wheel_msg = Float64()
                wheel_msg.data = vel
                self.wheel_pubs[i].publish(wheel_msg)
            
            # Debug logging
            if rospy.get_time() % 1.0 < 0.05:  # Log every ~1 second
                rospy.loginfo(f"Cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
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