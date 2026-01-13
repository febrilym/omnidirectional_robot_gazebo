#!/usr/bin/env python3

import rosbag
import csv
import argparse
import sys
import os
from tqdm import tqdm

def convert_bag_to_csv(bag_file, topic, output_file):
    try:
        # open bag file
        bag = rosbag.Bag(bag_file, 'r')
        
        # counting messages total for progress bar
        message_count = bag.get_message_count(topic_filters=[topic])
        
        if message_count == 0:
            print(f"ERROR: No messages found for topic '{topic}' in bag file")
            bag.close()
            return False
        
        print(f"Converting {message_count} messages from {topic}")
        print(f"Input: {bag_file}")
        print(f"Output: {output_file}")
        
        # open csv file for write
        with open(output_file, 'w', newline='') as csvfile:
            writer = None

            for topic, msg, t in tqdm(bag.read_messages(topics=[topic]), 
                                     total=message_count, 
                                     desc="Converting",
                                     unit="msg"):
                
                # header
                if writer is None:
                    fieldnames = ['timestamp']
                    
                    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                        fieldnames.extend([
                            'pos_x', 'pos_y', 'pos_z',
                            'ori_x', 'ori_y', 'ori_z', 'ori_w',
                            'vel_x', 'vel_y', 'vel_z',
                            'ang_vel_x', 'ang_vel_y', 'ang_vel_z'
                        ])
                    
                    elif hasattr(msg, 'name') and hasattr(msg, 'position'):
                        fieldnames.extend(['joint_names', 'positions', 'velocities', 'efforts'])

                    elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                        fieldnames.extend([
                            'lin_x', 'lin_y', 'lin_z',
                            'ang_x', 'ang_y', 'ang_z'
                        ])

                    else:
                        for attr in dir(msg):
                            if not attr.startswith('_') and not callable(getattr(msg, attr)):
                                fieldnames.append(attr)
                    
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                
                # data row
                row_data = {'timestamp': t.to_sec()}
                
                # odometry message
                if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                    row_data.update({
                        'pos_x': msg.pose.pose.position.x,
                        'pos_y': msg.pose.pose.position.y,
                        'pos_z': msg.pose.pose.position.z,
                        'ori_x': msg.pose.pose.orientation.x,
                        'ori_y': msg.pose.pose.orientation.y,
                        'ori_z': msg.pose.pose.orientation.z,
                        'ori_w': msg.pose.pose.orientation.w,
                        'vel_x': msg.twist.twist.linear.x,
                        'vel_y': msg.twist.twist.linear.y,
                        'vel_z': msg.twist.twist.linear.z,
                        'ang_vel_x': msg.twist.twist.angular.x,
                        'ang_vel_y': msg.twist.twist.angular.y,
                        'ang_vel_z': msg.twist.twist.angular.z
                    })
                
                # jointstate message
                elif hasattr(msg, 'name') and hasattr(msg, 'position'):
                    row_data.update({
                        'joint_names': ';'.join(msg.name),
                        'positions': ';'.join(str(p) for p in msg.position),
                        'velocities': ';'.join(str(v) for v in msg.velocity) if msg.velocity else '',
                        'efforts': ';'.join(str(e) for e in msg.effort) if msg.effort else ''
                    })
                
                # twist message
                elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                    row_data.update({
                        'lin_x': msg.linear.x,
                        'lin_y': msg.linear.y,
                        'lin_z': msg.linear.z,
                        'ang_x': msg.angular.x,
                        'ang_y': msg.angular.y,
                        'ang_z': msg.angular.z
                    })
                
                # generic handling
                else:
                    for field in writer.fieldnames[1:]:
                        if hasattr(msg, field):
                            value = getattr(msg, field)
                            if not isinstance(value, (int, float, str, bool)):
                                value = str(value)
                            row_data[field] = value
                
                writer.writerow(row_data)
        
        bag.close()
        print(f"\nConversion complete!")
        print(f"Output saved to: {output_file}")
        
        # file info display
        file_size = os.path.getsize(output_file)
        print(f"File size: {file_size / 1024:.2f} KB")
        
        return True
        
    except Exception as e:
        print(f"ERROR: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Convert ROS bag files to CSV format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -b robot_data.bag -t /odom -o odometry.csv
  %(prog)s -b data.bag -t /joint_states -o joints.csv
  %(prog)s -b commands.bag -t /cmd_vel -o commands.csv
        """
    )
    
    parser.add_argument('-b', '--bag', required=True, 
                       help='Input ROS bag file')
    parser.add_argument('-t', '--topic', required=True,
                       help='Topic to extract from bag file')
    parser.add_argument('-o', '--output', required=True,
                       help='Output CSV file path')
    parser.add_argument('--list-topics', action='store_true',
                       help='List all topics in the bag file and exit')
    
    args = parser.parse_args()

    if args.list_topics:
        try:
            bag = rosbag.Bag(args.bag, 'r')
            print(f"\nTopics in {args.bag}:")
            print("-" * 50)
            for topic, msg_type in bag.get_type_and_topic_info().topics.items():
                print(f"{topic} [{msg_type.msg_type}] - {msg_type.message_count} messages")
            bag.close()
            return
        except Exception as e:
            print(f"Error reading bag file: {e}")
            return
    
    # convert bag to csv
    success = convert_bag_to_csv(args.bag, args.topic, args.output)
    
    if not success:
        sys.exit(1)

if __name__ == '__main__':
    main()