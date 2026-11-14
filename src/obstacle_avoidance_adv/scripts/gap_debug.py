"""Explain the gap search on one live LiDAR scan.

Run in the container while the rover launch is running (the controller need not be):
    python3 /home/ws/ugv_ws/src/obstacle_avoidance_adv/scripts/gap_debug.py
It uses only the LiDAR, like the controller does when camera depth is missing.
"""
from dataclasses import fields
import math
import os
import sys

import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from obstacle_avoidance_adv.planner import Scan, Settings, View, rotation


def main():
    rclpy.init()
    node = rclpy.create_node('gap_debug')
    buffer = Buffer()
    TransformListener(buffer, node)
    scans = []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    while len(scans) < 3 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.)
    tf = None
    for _ in range(30):
        try:
            tf = buffer.lookup_transform('base_footprint', scans[-1].header.frame_id, Time())
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=.2)
    if tf is None:
        sys.exit('No TF from base_footprint to the LiDAR frame')
    # Same settings as the controller: its config file, falling back to the defaults.
    config = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'controller.yaml')
    with open(config) as f:
        values = yaml.safe_load(f)['obstacle_avoidance_adv']['ros__parameters']
    cfg = Settings(**{f.name: float(values[f.name]) for f in fields(Settings) if f.name in values})
    print(f'settings from {os.path.normpath(config)}: half_width {cfg.half_width}, '
          f'gap_margin {cfg.gap_margin}, smallest gap {2 * (cfg.half_width + cfg.gap_margin):.2f} m\n')
    previous = Scan(scans[-2], tf.transform, np.zeros(3), 0., None, cfg.half_length, cfg.half_width)
    scan = Scan(scans[-1], tf.transform, np.zeros(3), .1, previous, cfg.half_length, cfg.half_width)
    view = View([previous, scan], np.zeros(3), np.empty((0, 2)), cfg, 0.)
    p = view.points

    print('LiDAR points ahead (x forward, y left, metres), grouped:')
    ahead = p[(p[:, 0] > 0) & (p[:, 0] < 3.) & (abs(p[:, 1]) < 1.5)]
    ahead = ahead[np.argsort(np.arctan2(ahead[:, 1], ahead[:, 0]))]
    groups, current = [], [ahead[0]] if len(ahead) else []
    for a, b in zip(ahead[:-1], ahead[1:]):
        if np.linalg.norm(a - b) > .10:
            groups.append(np.array(current)); current = []
        current.append(b)
    if len(current):
        groups.append(np.array(current))
    for g in groups:
        if len(g) >= 3:
            print(f'  {len(g):3d} pts  x {g[:,0].min():.2f}..{g[:,0].max():.2f}  y {g[:,1].min():+.2f}..{g[:,1].max():+.2f}')

    front = view.clearance(0.) + cfg.half_length
    print(f'\nclearance {view.clearance(0.):.2f} m (front face {front:.2f} m from centre); '
          f'lookahead {cfg.gap_lookahead} m')
    corridor = cfg.half_width + cfg.gap_margin
    print(f'corridor half-width {corridor:.2f} m')
    print('\n   y    result')
    # Same checks, in the same order, as View.gap_offset in the controller.
    for y in np.arange(-cfg.gap_max_offset, cfg.gap_max_offset + 1e-9, .04):
        d = p[:, 1] - y
        near_row = (p[:, 0] > front - cfg.obstacle_length) & (p[:, 0] < front + cfg.obstacle_length + .05)
        left = near_row & (d >= corridor) & (d < corridor + cfg.gap_side_window)
        right = near_row & (d <= -corridor) & (d > -corridor - cfg.gap_side_window)
        if abs(y) < corridor:
            why = 'on the route'
        elif not (left.any() and right.any()):
            why = f'not between two obstacles (left {left.any()}, right {right.any()})'
        else:
            near = min(front, float(p[left | right, 0].min()))
            entry = near - cfg.half_length - cfg.stop_clearance - cfg.gap_runup
            through = front + cfg.obstacle_length + cfg.gap_exit - entry
            channel = (p[:, 0] > entry - cfg.half_length) & (p[:, 0] < entry + through + cfg.half_length)
            if entry <= 0:
                why = f'obstacles too close (nearest beside the gap {near:.2f} m); no room to line up'
            elif abs(math.atan2(y, entry)) > cfg.gap_max_angle:
                why = f'entry angle {math.degrees(math.atan2(y, entry)):.0f} deg > {math.degrees(cfg.gap_max_angle):.0f}'
            elif np.any(channel & (abs(d) < corridor)):
                hit = p[channel & (abs(d) < corridor)]
                why = f'corridor blocked: {len(hit)} pts, nearest x={hit[:,0].min():.2f} y={hit[np.argmin(hit[:,0]),1]:+.2f}'
            elif not view.gap_reachable(0., entry, through, y):
                why = 'drive to entry blocked, or far side of the gap not observed'
            else:
                why = f'GAP OK (entry {entry:.2f} m ahead)'
        print(f'{y:+.2f}   {why}')
    print('\ncontroller result:', view.gap_plan(0.))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
