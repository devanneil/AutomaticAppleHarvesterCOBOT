#!/usr/bin/env python3

import yaml

with open("tmp.dat", "r") as f:
    traj = yaml.safe_load(f)

points = traj["points"]

num_joints = len(points[0]["velocities"])

max_velocities = [0.0] * num_joints
max_indices = [0] * num_joints

for point_idx, point in enumerate(points):
    for joint_idx, velocity in enumerate(point["velocities"]):
        if abs(velocity) > max_velocities[joint_idx]:
            max_velocities[joint_idx] = abs(velocity)
            max_indices[joint_idx] = point_idx

print("Maximum Joint Velocities")
print("------------------------")

for i, (vel, idx) in enumerate(zip(max_velocities, max_indices)):
    print(f"Joint {i+1}: {vel:.4f} rad/s (point {idx})")

print("Time Total")
end_time_from_start = points[-1]["time_from_start"]["sec"]
print(end_time_from_start)