# Author: Yair Jacob
# use: extracts csv from .db3 bag

import csv
import sys
from pathlib import Path
from rosbag_reader import load_twist_messages

def write_csv(data, output_file):
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'topic',
            'linear_x', 'linear_y', 'linear_z',
            'angular_x', 'angular_y', 'angular_z'
        ])

        for d in data:
            writer.writerow([
                d['timestamp'],
                d['topic'],
                *d['linear'],
                *d['angular']
            ])

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 bag_to_csv.py <path_to_bag>")
        sys.exit(1)

    bag_path = Path(sys.argv[1])

    print(f"Reading bag from: {bag_path}")
    messages = load_twist_messages(bag_path)

    if not messages:
        print("No messages found.")
    else:
        write_csv(messages, "output.csv")
        print(f"Saved {len(messages)} messages to output.csv")
