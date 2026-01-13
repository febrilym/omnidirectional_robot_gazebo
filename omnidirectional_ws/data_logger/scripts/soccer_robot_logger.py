#!/usr/bin/env python3

import rospy
import csv
import os
import time
from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import math

class SoccerRobotLogger:
    def __init__(self):
        rospy.init_node('soccer_robot_logger')
        
        # parameters
        self.robot_name = rospy.get_param('~robot_name', 'robot_1')
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/bratasena_ws/src/soccer_robot_logs'))
        self.sampling_rate = rospy.get_param('~sampling_rate', 20)
        
        # setup directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # file names
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.odom_file = os.path.join(self.output_dir, f"odom_{self.robot_name}_{timestamp}.csv")
        self.joints_file = os.path.join(self.output_dir, f"joints_{self.robot_name}_{timestamp}.csv")
        self.cmd_file = os.path.join(self.output_dir, f"cmd_{self.robot_name}_{timestamp}.csv")
        
        # setup csv files
        self.setup_csv_files()
        
        # wheel joints
        self.wheel_joints = [
            f"{self.robot_name}_first_wheel_joint",
            f"{self.robot_name}_second_wheel_joint",
            f"{self.robot_name}_third_wheel_joint",
            f"{self.robot_name}_forth_wheel_joint"
        ]
        
        # subscribers
        rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber(f"/{self.robot_name}/cmd_vel", Twist, self.cmd_callback)
        
        # timing control
        self.last_odom_time = 0
        self.last_joint_time = 0
        self.last_cmd_time = 0
        self.interval = 1.0 / self.sampling_rate
        
        # counters
        self.odom_count = 0
        self.joint_count = 0
        self.cmd_count = 0
        
        # print info
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"SOCCER ROBOT LOGGER - {self.robot_name}")
        rospy.loginfo(f"Output: {self.output_dir}")
        rospy.loginfo(f"Rate: {self.sampling_rate} Hz")
        rospy.loginfo("=" * 50)
        
        rospy.on_shutdown(self.shutdown)
    
    def setup_csv_files(self):
        # odometry csv
        with open(self.odom_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'robot', 'x', 'y', 'z', 
                'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'
            ])
        
        # joints csv
        with open(self.joints_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'robot', 'wheel1_vel', 'wheel2_vel', 'wheel3_vel', 'wheel4_vel',
                'wheel1_pos', 'wheel2_pos', 'wheel3_pos', 'wheel4_pos'
            ])
        
        # commands csv
        with open(self.cmd_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'robot', 'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_wx', 'cmd_wy', 'cmd_wz'
            ])
    
    def odom_callback(self, msg):
        current_time = time.time()
        
        if current_time - self.last_odom_time < self.interval:
            return
        
        self.last_odom_time = current_time
        self.odom_count += 1
        
        with open(self.odom_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.to_sec(),
                self.robot_name,
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
        
        # log every 100 data
        if self.odom_count % 100 == 0:
            rospy.loginfo(f"Odometry logged: {self.odom_count} samples")
    
    def joint_callback(self, msg):
        current_time = time.time()
        
        if current_time - self.last_joint_time < self.interval:
            return
        
        self.last_joint_time = current_time
        self.joint_count += 1
        
        # find joints wheel
        wheel_vel = [0.0] * 4
        wheel_pos = [0.0] * 4
        
        for i, joint_name in enumerate(self.wheel_joints):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                if idx < len(msg.velocity):
                    wheel_vel[i] = msg.velocity[idx]
                if idx < len(msg.position):
                    wheel_pos[i] = msg.position[idx]
        
        with open(self.joints_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.header.stamp.to_sec(),
                self.robot_name,
                wheel_vel[0], wheel_vel[1], wheel_vel[2], wheel_vel[3],
                wheel_pos[0], wheel_pos[1], wheel_pos[2], wheel_pos[3]
            ])
    
    def cmd_callback(self, msg):
        current_time = time.time()
        
        if current_time - self.last_cmd_time < self.interval:
            return
        
        self.last_cmd_time = current_time
        self.cmd_count += 1
        
        with open(self.cmd_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                rospy.get_time(),
                self.robot_name,
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z
            ])
    
    def shutdown(self):
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"SHUTDOWN - {self.robot_name}")
        rospy.loginfo(f"Odometry: {self.odom_count} samples")
        rospy.loginfo(f"Joints: {self.joint_count} samples")
        rospy.loginfo(f"Commands: {self.cmd_count} samples")
        rospy.loginfo(f"Data saved to: {self.output_dir}")
        rospy.loginfo("=" * 50)

def main():
    try:
        logger = SoccerRobotLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Logger stopped by user")
    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    main()