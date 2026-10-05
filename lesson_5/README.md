# Lesson 5: Autonomous navigation with Nav2

In Lesson 4 the robot learned **where it is** on the map. You still had to drive it with the remote.

In this lesson the robot drives itself. You click a goal on the map, and **Nav2** (the ROS 2 Navigation Stack) plans a path around the walls, follows it, avoids obstacles that weren't on the map, and stops at the goal.

In this lesson you will:

1. Start localization (Lesson 4) and Nav2 on top of it.
2. Look at the **costmaps** Nav2 uses to decide where the robot can go.
3. Start the bridge that lets ROS drive the robot.
4. Send navigation goals from RViz and from the command line.
5. See how Nav2 reacts to obstacles that are not on the map.

**Before you start:** complete Lesson 4. You need your saved map, a Linux laptop for RViz, and the Unitree handheld remote.

> **⚠ Safety: this is the first lesson where software moves the robot.**
> - **One person holds the handheld remote at all times** and is ready to take over. The remote overrides ROS.
> - Clear the area. Keep people at least 1 m from the robot's path.
> - Start with goals **1–2 m away**, in open space.
> - Only **one** person in the group runs `cmd_vel_node` and sends goals.

---

## Background

### From a goal to the robot's legs

```
  you click a goal
        │
        ▼
  bt_navigator ──► planner_server ──► a path across the map (global costmap)
        │
        ▼
  controller_server ──► velocity commands, 10 times a second (local costmap)
        │
        ▼  /cmd_vel_nav
  velocity_smoother ──► limits acceleration
        │
        ▼  /cmd_vel  (geometry_msgs/Twist, 20 Hz)
  cmd_vel_node ──► /api/sport/request  (Unitree Move, api_id 1008)
        │
        ▼
     the robot walks
```

| Node | Job |
|------|-----|
| `bt_navigator` | The "boss". Runs a **behavior tree**: plan a path, follow it, and if something fails, try a recovery and plan again. |
| `planner_server` | Finds a path from the robot to the goal on the **global costmap**. We use **NavFn**, a grid search (Dijkstra). |
| `controller_server` | Follows the path. Every 0.1 s it tries hundreds of possible velocities, simulates each for 1.5 s, and picks the best one. We use **DWB** (Dynamic Window Approach). |
| `smoother_server` | Smooths the planner's path so it has fewer sharp corners. |
| `behavior_server` | Recovery behaviors: **spin** on the spot, **back up**, **wait**. Used when the robot is stuck. |
| `waypoint_follower` | Visits a list of goals in order. You will use it in Lesson 6. |
| `velocity_smoother` | Takes the controller's commands from `/cmd_vel_nav`, limits accelerations so they are gentle (0.5 m/s², 1.0 rad/s²), and republishes them on `/cmd_vel` at 20 Hz. |
| `lifecycle_manager_navigation` | Configures and activates all of the above, as in Lesson 4. |

> **Topic remapping.** Nav2 nodes publish on a topic *named* `cmd_vel` by default. The launch file **remaps** the names to build the chain above. `controller_server`'s `cmd_vel` becomes `/cmd_vel_nav`. On `velocity_smoother`, `cmd_vel` becomes `/cmd_vel_nav` and `cmd_vel_smoothed` becomes `/cmd_vel`. Look for `remappings=` in `go2_a3_nav2.launch.py`, and check the result with `ros2 topic info -v /cmd_vel_nav`. Recovery behaviors from `behavior_server` publish straight to `/cmd_vel`. They are short, slow motions that don't need smoothing.

Nav2 only produces **velocity commands**. It knows nothing about Unitree robots. `cmd_vel_node`, the bridge from the optional part of Lesson 2, turns them into Unitree Move requests. **Without `cmd_vel_node`, Nav2 plans and "drives", but the robot doesn't move.**

### Costmaps

The map from Lesson 3 only says *free*, *occupied* or *unknown*. Nav2 needs more: **how dangerous** is each cell for a robot of this size? This is a **costmap**, built in layers:

| Layer | Source | What it adds |
|-------|--------|--------------|
| **Static** | `/map` from map_server | The walls of your saved map |
| **Obstacle** | `/scan`, live | Anything the lidar sees now: people, bags, a chair that moved. Cleared again when the laser sees through the space. |
| **Inflation** | computed | A "danger zone" around every obstacle, fading out over 0.40 m. The planner prefers paths away from walls. |

Nav2 keeps **two** costmaps:

| | Global costmap | Local costmap |
|-|----------------|---------------|
| Used by | planner | controller |
| Size | the whole map | 3 m × 3 m around the robot, moving with it |
| Frame | `map` | `odom` |
| Updates | 1 per second | 5 per second |

The robot is described by its **footprint**, a rectangle of 0.84 m × 0.44 m around `base_link`. A cell is "lethal" for the robot if the footprint would touch an obstacle there.

---

You need five terminals on the robot (`ssh -X unitree@192.168.123.18`):

| Terminal | Runs |
|----------|------|
| T1 | localization (map_server, AMCL, lidar, odometry) |
| T2 | RViz |
| T3 | Nav2 |
| T4 | velocity bridge `cmd_vel_node` |
| T5 | inspection commands and command-line goals |

## Step 1: Start localization (T1)

Exactly as in Lesson 4:

```bash
ros2 launch go2_slam_nav go2_a3_localization.launch.py \
  map:=$HOME/go2_maps/<group_name>_map.yaml \
  serial_port:=$(ls /dev/serial/by-id/*CP2102*)
```

## Step 2: Open RViz and set the initial pose (T2)

Nav2 ships a ready-made RViz config with all the displays you need for this lesson:

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

It shows the map, the laser scan and the AMCL particles, and has the **Navigation 2** panel on the left. The costmap and path displays stay empty until Nav2 is running.

This config was written for a different, wheeled robot. **RobotModel**, **Bumper Hit** and the **Realsense** group show errors or stay empty, because our robot has no such topics. Untick them.

Now set the initial pose with **2D Pose Estimate**, as in Lesson 4, Step 4: click where the robot stands, and drag the way it faces.

> **⚠ Do this before you start Nav2.** Until AMCL has an initial pose, there is no `map` frame, and Nav2 can't start: its log fills with `Timed out waiting for transform from base_link to map`.

Check that the pose is set (T5):

```bash
ros2 run tf2_ros tf2_echo map odom    # must print a Translation, not "frame does not exist"
```

Then check in RViz that the scan sits on the walls. Nav2 trusts AMCL completely. If the robot thinks it is 1 m from where it really is, it will walk into things.

## Step 3: Start Nav2 (T3)

```bash
ros2 launch go2_slam_nav go2_a3_nav2.launch.py
```

The log is long. Wait for:

```
[lifecycle_manager_navigation]: Managed nodes are active
```

Check (T5):

```bash
ros2 lifecycle get /controller_server   # active [3]
ros2 lifecycle get /bt_navigator        # active [3]
ros2 topic list | grep -E "costmap|plan"
```

The robot **doesn't move** yet: there is no goal, and `cmd_vel_node` isn't running.

The costmaps now appear in RViz. Find these in the **Displays** list and toggle them on and off to see what each one is:

- **Global Planner → Global Costmap**: the whole map with walls inflated. Blue → red → magenta means low cost → high cost → lethal.
- **Global Planner → Path**: the planned path. It appears after you send a goal.
- **Controller → Local Costmap**: the 3 m × 3 m square around the robot.
- **Controller → Polygon**: the robot's footprint rectangle.
- **Amcl Particle Swarm**: the particles from Lesson 4.

> **Experiment: the obstacle layer.** With Nav2 running but no goal, ask someone to walk slowly past the robot, about 1 m away. Watch them appear in the local costmap, and disappear again once they've gone.

### Experiment: Nav2 without the bridge

Before you let Nav2 move the robot, watch what it *would* do. `cmd_vel_node` isn't running, so nothing reaches the robot.

1. In T5, run `ros2 topic echo /cmd_vel`.
2. In RViz, click **Nav2 Goal** in the toolbar, then click 1 m in front of the robot and drag the way it should face.
3. The path appears in RViz, and `/cmd_vel` fills with velocity commands, 20 per second. The robot doesn't move.
4. After 10 s, the T3 log says `[controller_server]: Failed to make progress`. Nav2 thinks the robot is stuck: it clears the costmaps and tries a recovery behavior. Cancel the goal with **Cancel** in the Navigation 2 panel.

This is exactly what you'll see later if `cmd_vel_node` crashes or the robot is lying down.

## Step 4: Start the velocity bridge (T4)

> **⚠ The robot stands up as soon as this starts** (it sends `api_id 1004`, StandUp). Remote in hand, area clear.

```bash
ros2 run mengram_pub cmd_vel_node
```

You should see:

```
Sent sport mode command (api_id=1004).
Bridge running: /cmd_vel -> /api/sport/request (max_x=0.5, max_y=0.3, max_z=1.0, deadman=0.5s)
```

## Step 5: Send a goal from RViz

1. In the toolbar, click **Nav2 Goal**.
2. Click on the map 1–2 m in front of the robot, in free space. Drag in the direction the robot should face when it arrives. Release.

Watch:

- The **global path** appears, from the robot to the goal.
- The robot **turns towards the path** and walks along it.
- The **local costmap** moves with the robot.
- At the goal, the robot **turns to the requested heading** and stops. The T3 log says `Reached the goal!` and `Goal succeeded`.

**To stop the robot during a goal**, use any of these:

- Press **Cancel** in the Navigation 2 panel.
- Take over with the **handheld remote**.
- Press **Ctrl+C** in T4. `cmd_vel_node` sends a stop as it exits.

### Experiment: an obstacle that is not on the map

1. Send a goal 3–4 m away, across open floor.
2. While the robot walks, have someone place a box (or stand still) on the path, **at least 1.5 m ahead of the robot**.
3. Watch the obstacle appear in the local costmap. Does the controller steer around it? Does the planner make a new path?

### Experiment: an impossible goal

1. Click a goal **inside a wall**, or in a grey (unknown) area outside the map.
2. What does the planner do? Look at the T3 log and the Navigation 2 panel.

The planner accepts goals up to `tolerance: 0.5` m from a free cell. Further than that, the goal fails. Notice `allow_unknown: true`: the planner is allowed to plan through grey, unexplored cells.

### Experiment: recovery behaviors

Put the robot in a tight spot: facing a wall, about 40 cm from it. Send a goal behind it. If the controller can't make progress for 10 s (`movement_time_allowance`), bt_navigator runs recovery behaviors: clearing the costmaps, spinning, backing up and waiting. Watch the T3 log to see which ones run.

## Step 6: Send a goal from the command line

The RViz button sends a **NavigateToPose action** to bt_navigator. You can send the same action yourself:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {header: {frame_id: map},
         pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

The feedback prints `distance_remaining`, `navigation_time` and `number_of_recoveries` while the robot walks. The final result is `SUCCEEDED`, `ABORTED` or `CANCELED`.

Ctrl+C cancels the goal. On Humble the command then often hangs at `Canceling goal...`, even though the goal **was** cancelled: the T3 log says `Goal canceled`. Press Ctrl+C a second time to get your prompt back.

Coordinates are in the `map` frame, in metres. To find the coordinates of a spot, hover over it in RViz: the status bar at the bottom shows the map position. Or read them from `/amcl_pose` when the robot stands there.

The heading is a **quaternion**. For a rotation `yaw` about the vertical axis:

```
qz = sin(yaw / 2)
qw = cos(yaw / 2)
```

| Facing | yaw | qz | qw |
|--------|-----|----|----|
| +x | 0 | 0.0 | 1.0 |
| +y (left) | π/2 | 0.707 | 0.707 |
| −x | π | 1.0 | 0.0 |
| −y (right) | −π/2 | −0.707 | 0.707 |

Lesson 6 sends goals like this from a Python script.

---

## Shortcut: everything in one launch

`robot_nav_stack.launch.py` starts localization **and** Nav2 together, and publishes the initial pose for you:

```bash
ros2 launch mengram_pub robot_nav_stack.launch.py \
  map:=$HOME/go2_maps/<group_name>_map.yaml \
  serial_port:=$(ls /dev/serial/by-id/*CP2102*) \
  initial_x:=0.0 initial_y:=0.0 initial_qz:=0.0 initial_qw:=1.0
```

The initial pose is sent once, 5 s after launch. `(0, 0)` facing `+x` is where the robot stood when you **started mapping**. **Check in RViz that the scan matches the walls.** If it doesn't, set the pose again with **2D Pose Estimate**.

You still need `cmd_vel_node` in a separate terminal.

---

## The settings

Parameters are in `~/demo_ws/install/go2_slam_nav/share/go2_slam_nav/config/go2_a3_nav2.yaml`.

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `FollowPath.max_vel_x` / `min_vel_x` | `0.4` / `-0.4` m/s | Top speed forward and backward |
| `FollowPath.max_vel_y` | `0.0` | **No sideways walking.** Nav2 drives the dog like a wheeled robot, even though it can step sideways. |
| `FollowPath.max_vel_theta` | `0.8` rad/s | Top turning speed |
| `FollowPath.sim_time` | `1.5` s | How far ahead DWB simulates each candidate velocity |
| `general_goal_checker.xy_goal_tolerance` | `0.15` m | "Arrived" when within 15 cm… |
| `general_goal_checker.yaw_goal_tolerance` | `0.3` rad | …and within about 17° of the goal heading |
| `progress_checker.required_movement_radius` / `movement_time_allowance` | `0.2` m / `10` s | If the robot moves less than 20 cm in 10 s, it is "stuck" and recovery starts |
| `footprint` / `footprint_padding` | 0.84 m × 0.44 m / `0.1` m | The robot's outline, around `base_link`, plus 10 cm of padding on every side |
| `inflation_radius` | `0.40` m | How far the danger zone extends from obstacles |
| `obstacle_max_range` | `6.0` m | Laser points further than this don't mark obstacles |
| `GridBased.allow_unknown` | `true` | Plan through unexplored (grey) cells |

The speed caps in `cmd_vel_node` (0.5 m/s, 0.3 m/s, 1.0 rad/s) are a second safety limit on top of these.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| The path appears in RViz but the robot doesn't move, and after 10 s the log says `Failed to make progress` | `cmd_vel_node` is not running (T4). Check with `ros2 node list`. Also check that the robot is standing. |
| `Control loop missed its desired rate of 10.0000Hz` | The Jetson is busy, for example when RViz is running on it. Harmless if it happens now and then. Close RViz if it is constant. |
| `No goal checker was specified in parameter 'current_goal_checker'` | Harmless. There is only one goal checker, and Nav2 uses it. |
| `ros2 action send_goal` hangs at `Canceling goal...` | Known issue on Humble. The goal was cancelled anyway. Press Ctrl+C again. |
| T3 repeats `Timed out waiting for transform from base_link to map` and `StaticLayer: "map" passed to lookupTransform argument target_frame does not exist` | AMCL has no initial pose yet, so the `map` frame doesn't exist. Set it with **2D Pose Estimate** (Lesson 4, Step 4). The messages stop and Nav2 finishes starting up. |
| `Managed nodes are active` never appears | A node failed to start. Scroll up in T3 for the first `ERROR`. |
| The goal is rejected or aborted straight away | The goal is inside an obstacle, or more than 0.5 m outside free space. Try a goal in open floor. |
| The robot walks into a wall, or misses the goal by a lot | Localization is wrong. Stop. Check in RViz that the scan sits on the walls, and set the initial pose again. |
| The robot keeps spinning or backing up | Recovery behaviors: it thinks it is stuck. Often the inflated walls block a narrow doorway (0.84 m × 0.44 m robot plus 0.40 m inflation). Cancel and try a goal in open space. |
| The robot stops for a moment, then continues | Probably `cmd_vel_node`'s 0.5 s stop rule. Check whether `/cmd_vel` stops briefly: `ros2 topic hz /cmd_vel`. |
| Costmaps don't show in RViz | Fixed Frame must be `map`. Check that the topics exist with `ros2 topic list \| grep costmap`. |

## Summary

```bash
# T1  localization (Lesson 4)
ros2 launch go2_slam_nav go2_a3_localization.launch.py \
  map:=$HOME/go2_maps/<group_name>_map.yaml serial_port:=$(ls /dev/serial/by-id/*CP2102*)

# T2  view, then 2D Pose Estimate → click and drag (BEFORE starting Nav2)
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

# T3  Nav2
ros2 launch go2_slam_nav go2_a3_nav2.launch.py

# T4  velocity bridge (the robot stands up!)
ros2 run mengram_pub cmd_vel_node

# RViz: Nav2 Goal → click and drag.   Stop: Cancel / remote / Ctrl+C in T4
```

**Next lesson:** send goals from a Python script, and make the robot patrol between two points.
