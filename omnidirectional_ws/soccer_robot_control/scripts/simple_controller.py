#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller')
        
        self.robot_name = rospy.get_param('~robot_name', 'robot_1')
        
        # publisher for cmd_vel
        self.cmd_pub = rospy.Publisher(f'/{self.robot_name}/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo(f"Simple Controller started for {self.robot_name}")
        
        # test: send forward command periodicly
        self.timer = rospy.Timer(rospy.Duration(0.1), self.send_cmd)
        
    def send_cmd(self, event):
        # test with forward command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.1
        cmd_msg.angular.z = 0.0
        
        self.cmd_pub.publish(cmd_msg)
        
        rospy.loginfo_throttle(2, "Sending forward command...")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = SimpleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass