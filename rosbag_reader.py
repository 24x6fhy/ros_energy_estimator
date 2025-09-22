# Author: Yair Jacob
# helper: read bag -> structured data

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def load_twist_messages(bag_path, topics=('/cmd_vel', '/odom')):
    reader = SequentialReader()

    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    all_data = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic in topics:
            msg_type = Twist if type_map[topic].endswith('Twist') else Odometry
            msg = deserialize_message(data, msg_type)

            # if odom check twist
            if isinstance(msg, Odometry):
                twist = msg.twist.twist
            else:
                twist = msg

            all_data.append({
                'timestamp': timestamp,
                'topic': topic,
                'linear': (
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z
                ),
                'angular': (
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z
                )
            })

    return all_data
