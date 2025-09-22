# extracts csv from .db3 bag

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import csv
from pathlib import Path

def read_rosbag(bag_path, output_csv="output.csv"):
    rclpy.init()
    reader = SequentialReader()

    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # prep csv
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            'timestamp', 'topic', 'linear_x', 'linear_y', 'linear_z',
            'angular_x', 'angular_y', 'angular_z'
        ])

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic in ['/cmd_vel', '/odom']:
                msg_type = Twist if type_map[topic].endswith('Twist') else Odometry
                msg = deserialize_message(data, msg_type)

                if isinstance(msg, Odometry):
                    twist = msg.twist.twist
                else:
                    twist = msg

                writer.writerow([
                    t, topic,
                    twist.linear.x, twist.linear.y, twist.linear.z,
                    twist.angular.x, twist.angular.y, twist.angular.z
                ])

    print(f"[✓] Data written to {output_csv}")
    rclpy.shutdown()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 bag_to_csv.py <bag_folder_path>")
        exit(1)

    bag_path = Path(sys.argv[1])
    read_rosbag(bag_path)
