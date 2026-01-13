#!/usr/bin/env python3

import rospy
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry

class OdometryLogger:
    def __init__(self):
        rospy.init_node('simple_odometry_logger')
        
        # parameters
        self.robot_name = rospy.get_param('~robot_name', 'robot_1')
        self.topic = rospy.get_param('~topic', f'/{self.robot_name}/odom')
        
        # setup output
        output_dir = os.path.expanduser('~/bratasena_ws/src/soccer_robot_logs')
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(output_dir, f"odom_{self.robot_name}_{timestamp}.csv")
        
        # setup csv
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])
        
        # subscriber
        rospy.Subscriber(self.topic, Odometry, self.callback)
        
        rospy.loginfo(f"Odometry Logger started for {self.robot_name}")
        rospy.loginfo(f"Topic: {self.topic}")
        rospy.loginfo(f"File: {self.filename}")
        
        rospy.on_shutdown(self.shutdown)
    
    def callback(self, msg):
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.to_sec(),
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
    
    def shutdown(self):
        rospy.loginfo(f"Odometry data saved to {self.filename}")

if __name__ == '__main__':
    try:
        logger = OdometryLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass