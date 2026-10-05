#!/usr/bin/env python3
"""Patrol between waypoints on the map with Nav2.

Before running: localization and Nav2 are up, the initial pose is set in
RViz, cmd_vel_node is running, and the robot is in Classic Walk mode.
"""

import math
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Waypoints in the map frame: (x [m], y [m], yaw [degrees]).
# Replace these with points from YOUR map (see Step 2 of the lesson).
WAYPOINTS = [
    (1.0, 0.0, 0),
    (0.0, 0.0, 180),
]
LAPS = 3          # how many times to visit all the waypoints
PAUSE_S = 3.0     # wait at each waypoint, in seconds


def make_pose(navigator, x, y, yaw_deg):
    """Build a goal pose in the map frame."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def go_to(navigator, pose, name):
    """Send one goal and block until it finishes. Returns a TaskResult."""
    if not navigator.goToPose(pose):
        return TaskResult.FAILED  # the goal was rejected

    ticks = 0
    while not navigator.isTaskComplete():  # waits up to 0.1 s per call
        ticks += 1
        feedback = navigator.getFeedback()
        if feedback and ticks % 10 == 0:   # print about once a second
            print(f'  {name}: {feedback.distance_remaining:.2f} m to go, '
                  f'{feedback.number_of_recoveries} recoveries')
    return navigator.getResult()


def main():
    # Turn off rclpy's own Ctrl+C handling, so that Ctrl+C raises a plain
    # KeyboardInterrupt and we can still cancel the goal before exiting.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    navigator = BasicNavigator()

    # Wait for bt_navigator only. With the default (localizer='amcl'),
    # BasicNavigator publishes an initial pose of (0, 0, 0) and would
    # overwrite the pose you set in RViz.
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    try:
        for lap in range(1, LAPS + 1):
            for i, (x, y, yaw) in enumerate(WAYPOINTS, start=1):
                name = f'lap {lap}, point {i}'
                print(f'{name}: going to ({x}, {y}, {yaw} deg)')
                result = go_to(navigator, make_pose(navigator, x, y, yaw), name)
                if result != TaskResult.SUCCEEDED:
                    print(f'{name}: {result.name}, stopping the patrol')
                    return
                print(f'{name}: reached')
                time.sleep(PAUSE_S)
        print('patrol finished')
    except KeyboardInterrupt:
        # Without this, Nav2 keeps driving to the last goal after the script exits
        print('Ctrl+C: cancelling the current goal')
        navigator.cancelTask()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
