# Author: Yair Jacob
# use: CLI tool

import csv
import argparse

def read_csv(file_path):
    data = []
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            row['timestamp'] = int(row['timestamp'])
            row['linear_x'] = float(row['linear_x'])
            row['angular_z'] = float(row['angular_z'])
            data.append(row)
        return data

def est_distance(data):
    total_distance = 0.0
    for i in range(1, len(data)):
        v = data[i]['linear_x']
        dt = (data[i]['timestamp'] - data[i - 1]['timestamp']) / 1e9 # ns to s
        total_distance += abs(v) * dt
    return total_distance
    
def est_avg_speed(data):
    total_distance = est_distance(data)
    total_time = (data[-1]['timestamp'] - data[0]['timestamp']) / 1e9 # ns to s
    avg_speed = total_distance / total_time if total_time > 0 else 0
    return avg_speed

def est_energy(data):
    a, b, c = 20, 10, 5
    total_energy = 0.0 # joules
    for i in range(1, len(data)):
        v = abs(data[i]['linear_x'])
        w = abs(data[i]['angular_z'])
        dt = (data[i]['timestamp'] - data[i - 1]['timestamp']) / 1e9 # ns to s
  
        # est power
        power = a * v +b * w + c
        total_energy += power * dt
    wh = total_energy / 3600 # J to Wh
    return wh

def est_idle_time(data):
    idle_time = 0.0
    active_time = 0.0
    for i in range(1, len(data)):
        v = abs(data[i]['linear_x'])
        w = abs(data[i]['angular_z'])
        dt = abs(data[i]['timestamp'] -data[i - 1]['timestamp']) / 1e9
        if v < 0.01 and w < 0.01:
            idle_time += dt
        else:
            active_time += dt
    return idle_time, active_time
    
def main():
    parser = argparse.ArgumentParser(description="ROS bag energy & motion estimator")
    parser.add_argument('--csv', required=True, help='Path to ouutput CSV')
    parser.add_argument('--distance', action='store_true')
    parser.add_argument('--speed', action='store_true')
    parser.add_argument('--energy', action='store_true')
    parser.add_argument('--idle', action='store_true')

    args = parser.parse_args()
    data = read_csv(args.csv)

    if args.distance:
        print(f"Total distance traveled: {est_distance(data):.2f} meters")
    if args.speed:
        print(f"Average linear speed: {est_avg_speed(data):.2f} m/s")
    if args.energy:
        print(f"Estimated energy usage: {est_energy(data):.2f} Wh")
    if args.idle:
        it, at = est_idle_time(data)    
        print(f"Idle time: {it:.1f} s | Active time: {at:.1f} s")



if __name__ == "__main__":
    main()