# Lesson 6: Scripted goals: a patrol in Python

In Lesson 5 you sent goals by clicking in RViz, one at a time. A real application sends them from a program: deliver to a list of rooms, inspect a set of points every hour, or patrol a corridor.

In this lesson you will write a Python script that makes the robot **patrol** between points on your map, using **nav2_simple_commander**, the Python API that ships with Nav2.

In this lesson you will:

1. Read the example patrol script on the robot.
2. Pick waypoints on your map.
3. Write and run your own patrol script.
4. Handle failures and Ctrl+C safely.

**Before you start:** complete Lesson 5. Your robot must reach goals you send from RViz. You need your saved map, a Linux laptop for RViz, and the Unitree handheld remote.

> **⚠ Safety: the robot moves on its own for minutes at a time.**
> - **One person holds the handheld remote** for the whole patrol, ready to take over.
> - Keep the patrol area clear of people and bags. Choose waypoints in open floor.
> - Test each waypoint once with **Nav2 Goal** in RViz before you put it in the script.

---

## Background

### Actions

In Lesson 5 you sent a **NavigateToPose action** to `bt_navigator`. An action is a long-running request with three parts:

| Part | Direction | NavigateToPose example |
|------|-----------|------------------------|
| **Goal** | client → server | The pose to drive to |
| **Feedback** | server → client, while it runs | `distance_remaining`, `navigation_time`, `number_of_recoveries` |
| **Result** | server → client, once at the end | `SUCCEEDED`, `ABORTED` (failed) or `CANCELED` |

The client can also **cancel** a goal while it runs. Your script is an action client, exactly like the **Nav2 Goal** button in RViz.

### nav2_simple_commander

Writing an action client by hand takes a lot of code. `nav2_simple_commander` wraps it in one class, `BasicNavigator`:

| Method | What it does |
|--------|--------------|
| `waitUntilNav2Active()` | Blocks until Nav2 is ready to accept goals |
| `goToPose(pose)` | Sends a NavigateToPose goal and returns immediately. `False` if the goal was rejected. |
| `isTaskComplete()` | Waits up to 0.1 s for the goal to finish. `True` once it has finished, whatever the result. |
| `getFeedback()` | The latest feedback message |
| `getResult()` | `TaskResult.SUCCEEDED`, `FAILED` or `CANCELED` |
| `cancelTask()` | Cancels the running goal |
| `followWaypoints(poses)` | Sends a whole list of poses to `waypoint_follower` (see the optional part) |

The library's own examples are in `/opt/ros/humble/lib/python3.10/site-packages/nav2_simple_commander/`. Start with `example_nav_to_pose.py`.

---

## Step 1: Read the example script

The robot has a short example patrol:

```bash
cat ~/go2_patrol/patrol_two_points.py
```

Read it and find:

1. **`make_pose()`**: builds a `PoseStamped` in the `map` frame. The heading is given directly as a quaternion (`qz`, `qw`), as in Lesson 5, Step 6.
2. **`go_to()`**: sends a goal with `goToPose()`, then loops on `isTaskComplete()` until it finishes, and checks `getResult()`.
3. **`main()`**: waits for Nav2, then visits `point_1`, waits 5 s, and visits `point_2`.

> **Don't run it.** Its two points belong to a different map (`test_search_map_edited_v1`). On your map they could be inside a wall, or outside the room.

Your script in Step 4 does the same thing, plus:

- waypoints as `(x, y, yaw in degrees)` instead of quaternions,
- several laps,
- progress printed from the feedback,
- **Ctrl+C cancels the goal**, so the robot stops.

## Step 2: Start the navigation stack

Start everything from Lesson 5, Steps 1–4. Each command goes in its own terminal (`ssh -X unitree@192.168.123.18`):

```bash
# T1  localization
ros2 launch go2_slam_nav go2_a3_localization.launch.py \
  map:=$HOME/go2_maps/<group_name>_map.yaml serial_port:=$(ls /dev/serial/by-id/*CP2102*)

# T2  RViz, then 2D Pose Estimate (BEFORE starting Nav2)
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

# T3  Nav2
ros2 launch go2_slam_nav go2_a3_nav2.launch.py

# T4  velocity bridge (the robot stands up!), then remote: → + START (Classic Walk)
ros2 run mengram_pub cmd_vel_node
```

Check that a **Nav2 Goal** from RViz works before you go on. Use T5 for the rest of the lesson.

## Step 3: Pick your waypoints

You need the map coordinates `(x, y)` and heading `yaw` of each point. There are two ways to find them.

**From RViz.** Click **Publish Point** in the toolbar, then click on the map. The point is published on `/clicked_point`:

```bash
ros2 topic echo /clicked_point --field point     # T5, before you click
```

Choose the heading yourself: 0° faces the map's +x axis (red axis in RViz), 90° faces +y (green), 180° faces −x.

**By driving there.** Walk the robot to the spot with the remote, facing the way you want it to face, then read AMCL's pose:

```bash
ros2 topic echo /amcl_pose --once --field pose.pose
```

AMCL gives the heading as a quaternion. Convert it to degrees with `yaw = 2 · atan2(qz, qw)`:

```bash
python3 -c "import math; print(math.degrees(2 * math.atan2(QZ, QW)))"   # replace QZ and QW
```

Pick **two or three** points:

- in **open floor**, at least 0.5 m from walls and furniture (the robot's footprint plus inflation),
- 2–4 m apart,
- **tested**: send each one once with **Nav2 Goal** in RViz.

## Step 4: Write the script

Make a folder for your group:

```bash
mkdir -p ~/<group_name>_patrol
```

The script is in the course repository as [`patrol.py`](patrol.py). Copy it to the robot from your laptop:

```bash
# on your laptop, in the lesson_6 folder
scp patrol.py unitree@192.168.123.18:~/<group_name>_patrol/
```

Or create it on the robot with `nano ~/<group_name>_patrol/patrol.py` and paste it in.

Then edit the top of the file with your waypoints:

```python
WAYPOINTS = [
    (1.0, 0.0, 0),      # (x [m], y [m], yaw [degrees]) in the map frame
    (0.0, 0.0, 180),
]
LAPS = 3          # how many times to visit all the waypoints
PAUSE_S = 3.0     # wait at each waypoint, in seconds
```

### What the script does

Read the whole file. The important parts:

**Heading in degrees.** `make_pose()` converts `yaw` to a quaternion with the formula from Lesson 5: `qz = sin(yaw/2)`, `qw = cos(yaw/2)`.

**The goal loop.** `go_to()` sends the goal, then calls `isTaskComplete()` in a loop. Each call waits up to 0.1 s, so the loop runs about 10 times a second. Every 10th time it prints the feedback. When the goal finishes, it returns `getResult()`.

**Stop on failure.** If a goal ends with anything other than `SUCCEEDED`, the patrol stops. It doesn't skip to the next point: if the robot couldn't reach one waypoint, something is wrong, and a person should look.

**Ctrl+C cancels the goal.** This is a safety feature. The goal runs inside Nav2, not inside your script. **If the script just exits, Nav2 keeps driving the robot to the last goal.** So the script catches `KeyboardInterrupt` and calls `cancelTask()` before exiting. For this to work, `rclpy.init()` gets `signal_handler_options=SignalHandlerOptions.NO`. Otherwise rclpy handles Ctrl+C itself and shuts down before the script can send the cancel.

**Waiting for Nav2.** The script calls `waitUntilNav2Active(localizer='bt_navigator')`. With the default, `localizer='amcl'`, `BasicNavigator` also *publishes an initial pose* of (0, 0, 0), which would overwrite the pose you set in RViz. Setting the initial pose is your job, in RViz, before the script starts.

## Step 5: Run the patrol (T5)

Remote in hand, area clear:

```bash
python3 ~/<group_name>_patrol/patrol.py
```

The output looks like this:

```
[INFO] [basic_navigator]: Nav2 is ready for use!
lap 1, point 1: going to (1.0, 0.0, 0 deg)
[INFO] [basic_navigator]: Navigating to goal: 1.0 0.0...
  lap 1, point 1: 0.84 m to go, 0 recoveries
  lap 1, point 1: 0.42 m to go, 0 recoveries
lap 1, point 1: reached
lap 1, point 2: going to (0.0, 0.0, 180 deg)
...
patrol finished
```

Watch in RViz: each new goal draws a new **Path**, and the robot follows it, as in Lesson 5.

### Experiment: Ctrl+C

1. Start the patrol.
2. While the robot is walking towards a point, press **Ctrl+C** in T5.
3. The script prints `Ctrl+C: cancelling the current goal`. The robot stops, and the T3 log says `Goal canceled`.

Now try to predict what would happen without the `except KeyboardInterrupt` block. (Don't try it: the robot would keep walking with nothing left to stop it but the remote.)

### Experiment: an unreachable waypoint

1. Add a waypoint **inside a wall** to `WAYPOINTS`.
2. Run the patrol. What result does the script print when it gets there, and how long does Nav2 try first? Watch the recovery behaviors in the T3 log.

### Experiment: an obstacle on the route

While the robot patrols, place a box on its path, as in Lesson 5. Does the patrol still complete? Does the robot take the same route on the next lap once you remove the box?

### Exercise

Change the script so that it:

1. patrols **forever** until Ctrl+C, instead of a fixed number of laps,
2. **skips** a waypoint that fails and carries on with the next one, but stops after two failures in a row,
3. prints how long each lap took.

---

## Optional: one action for the whole patrol

Nav2 also has a `waypoint_follower` server: you give it the whole list, and it visits the points in order. It waits 1 s at each one (`waypoint_pause_duration: 1000` ms in `go2_a3_nav2.yaml`).

```python
poses = [make_pose(navigator, x, y, yaw) for x, y, yaw in WAYPOINTS]
navigator.followWaypoints(poses)
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    if feedback:
        print(f'heading to waypoint {feedback.current_waypoint}')
result = navigator.getResult()
```

The result message lists `missed_waypoints`: points Nav2 couldn't reach. It doesn't stop at the first failure, because `stop_on_failure` is `false` in our config.

Compare the two approaches. Which is simpler? Which gives your script more control, for example to do something at each waypoint?

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| The script never prints `Nav2 is ready for use!` | Nav2 isn't running, or not active yet. Check T3 for `Managed nodes are active`. |
| `ModuleNotFoundError: No module named 'nav2_simple_commander'` (or `rclpy`) | The terminal has no ROS environment. Open a new SSH session, or run `source ~/demo_ws/install/setup.bash`. |
| The first goal ends straight away with `FAILED` | The waypoint is in an obstacle or outside the map. Test it with **Nav2 Goal** in RViz. |
| The script prints progress, but the robot doesn't move | `cmd_vel_node` isn't running, or the robot isn't in Classic Walk mode (→ + START). See Lesson 5. |
| The robot goes to the wrong places | Localization is wrong. Check in RViz that the scan sits on the walls. Set the initial pose again. Also check that the waypoints come from **this** map. |
| After Ctrl+C the robot keeps walking | The goal wasn't cancelled. Stop it with the remote, or press **Cancel** in the Navigation 2 panel. Check that your script has the `except KeyboardInterrupt` block and `SignalHandlerOptions.NO`. |

## Summary

```python
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
navigator = BasicNavigator()
navigator.waitUntilNav2Active(localizer='bt_navigator')   # don't reset the initial pose

navigator.goToPose(make_pose(navigator, x, y, yaw))        # send a goal
while not navigator.isTaskComplete():                      # wait, reading feedback
    feedback = navigator.getFeedback()
result = navigator.getResult()                             # SUCCEEDED / FAILED / CANCELED

navigator.cancelTask()                                     # on Ctrl+C: stop the robot
```

```bash
# with Lesson 5's stack running (T1–T4) and the initial pose set:
python3 ~/<group_name>_patrol/patrol.py
```

**This is the last lesson.** You have gone from raw laser scans to a robot that maps a room, knows where it is, and patrols it on its own.
