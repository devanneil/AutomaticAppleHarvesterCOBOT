#!/usr/bin/env python3

import yaml

with open("tmp.dat", "r") as f:
    messages = yaml.safe_load_all(f)

    msg = next(messages, None)

    joint_max = dict(zip(msg['name'], msg['position']))
    joint_min = dict(zip(msg['name'], msg['position']))

    for msg in messages:
        if msg is None:
            continue

        for name, position in zip(msg['name'], msg['position']):
            if joint_max[name] < position:
                joint_max[name] = position
            if joint_min[name] > position:
                joint_min[name] = position

print("Max joint values:")
for name, position in joint_max.items():
    print(f"{name}: {position}")

print("Min joint values:")
for name, position in joint_min.items():
    print(f"{name}: {position}")