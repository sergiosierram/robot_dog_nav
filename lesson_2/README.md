# Lesson 2: Frames, odometry, and driving the robot

In Lesson 1 the laser scan existed only in its own frame, `laser`. RViz could draw it, but nothing knew where the robot was or how the lidar sits on it.

Before we can build a map (Lesson 3), ROS needs to know two things at every moment:

1. **Where the robot is** relative to where it started. This is **odometry**.
2. **Where the lidar is mounted** on the robot. This is a **static transform**.

In this lesson you will start each piece by hand, so you can see what the mapping launch file in Lesson 3 does for you.

**Before you start:** complete Lesson 1. You need SSH access with `ssh -X` from a Linux laptop (for RViz), and the lidar running.

---

## Background: TF and the frame tree

ROS describes every sensor and body part with its own **coordinate frame**. The **TF** system publishes how each frame relates to the others, as a tree:

```
odom ──► base_link ──► laser
 │          │            │
 │          │            └─ the lidar's own frame: the scan is measured here
 │          └─ the robot's body: x forward, y left, z up
 └─ fixed in the world, set where the robot was when odometry started
```

| Transform | Who publishes it | Changes over time? |
|-----------|------------------|--------------------|
| `odom → base_link` | `sportstate_to_odom` (from the robot's own state estimate) | Yes, as the robot walks |
| `base_link → laser` | `static_transform_publisher` | No, the lidar is bolted on |

Once both exist, ROS can express every laser point in the `odom` frame. This is what SLAM needs.

> **Convention (REP 105):** `odom` is smooth and continuous, but it **drifts** over time. Lesson 3 adds a `map` frame on top that corrects the drift: `map → odom → base_link → laser`.

---

## Step 1: Open four terminals on the robot

Each of the four terminals runs one process. Open each from your laptop:

```bash
ssh -X unitree@192.168.123.18
```

| Terminal | Runs |
|----------|------|
| T1 | lidar driver |
| T2 | odometry |
| T3 | lidar static transform |
| T4 | inspection commands, then RViz |

> In a group, **only one person** starts T1–T3. If several people start the same nodes, you get duplicate transforms and a jumpy display. Everyone else can run T4 commands and RViz.

## Step 2: Start the lidar (T1)

Same as Lesson 1:

```bash
ros2 launch go2_slam_nav rplidar_a3.launch.py
```

## Step 3: Start odometry (T2)

The Go2's main board estimates its own position and heading, and publishes them on `/lf/sportmodestate` (message type `unitree_go/msg/SportModeState`). First, look at the raw data:

```bash
ros2 topic hz /lf/sportmodestate                     # about 20 Hz
ros2 topic echo /lf/sportmodestate --once --field position
ros2 topic echo /lf/sportmodestate --once --field imu_state.rpy
```

`position` holds x, y, z in metres. `imu_state.rpy` holds roll, pitch and yaw in radians.

This message is Unitree-specific. Standard ROS tools such as SLAM and Nav2 expect a `nav_msgs/Odometry` message and an `odom → base_link` transform. A small bridge node converts between them:

```bash
ros2 run go2_state_bridge sportstate_to_odom
```

The source is `~/demo_ws/src/go2_state_bridge/go2_state_bridge/sportstate_to_odom.py`. It is less than 100 lines. Read it: it does three things.

1. It saves the **first** pose it receives as the origin, so `odom` starts at (0, 0, 0) wherever the robot is standing. **Restarting this node resets odometry to zero.**
2. It rotates later poses into that starting frame, keeping only x, y and yaw. The robot is treated as moving on flat ground.
3. It publishes `/odom` and broadcasts the `odom → base_link` transform.

> When you stop it with Ctrl+C, it prints a Python `KeyboardInterrupt` traceback. That is normal and harmless.

## Step 4: Tell ROS where the lidar is mounted (T3)

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.30 \
  --roll 0 --pitch 0 --yaw 3.141592653589793 \
  --frame-id base_link --child-frame-id laser
```

This says the lidar sits directly above the body centre (`x = 0`, `y = 0`), 30 cm up, and **rotated 180°**. The mapping (Lesson 3) and localization (Lesson 4) launch files use the same values.

The height `z` doesn't matter much for us: SLAM and Nav2 work in 2D and ignore it. It only changes how high the scan is drawn in RViz. The `x` and `y` values matter a lot, as the experiment in Step 7 shows.

Why 180°? The RPLidar ROS driver points the scan's +x axis **towards the cable**. On our robot the cable faces **backwards**, so the lidar's +x points backwards, the opposite of `base_link`'s +x (forward). A 180° yaw lines the two up. The driver's frame diagram is in `~/demo_ws/src/rplidar_ros/rplidar_A2.png`. The A3 uses the same convention.

> **Exercise:** look at the robot from above and from the side. Check that the lidar's centre really is midway between the front and back legs, and midway between left and right. If the lidar were mounted 15 cm further forward, which number in the command would change, and to what?

## Step 5: Inspect the frame tree (T4)

```bash
ros2 topic hz /odom                              # about 20 Hz
ros2 topic echo /odom --once --field pose.pose   # near zero right after starting
ros2 run tf2_ros tf2_echo odom laser             # the full chain, odom to laser
```

`tf2_echo odom laser` prints `Translation: [0.000, 0.000, 0.300]` and a 180° yaw while the robot stands at its start point. The first line may say `frame does not exist`. That is normal: wait a second for the first transform to arrive.

Draw the tree as a PDF:

```bash
cd ~ && ros2 run tf2_tools view_frames
```

This creates `frames_<date>.pdf`. Copy it to your laptop and open it:

```bash
# on your laptop
scp unitree@192.168.123.18:~/frames_*.pdf .
```

You should see `odom → base_link → laser`, with `base_link` updating at about 20 Hz and `laser` marked as static.

## Step 6: View it in RViz (T4)

```bash
rviz2
```

1. Set **Global Options → Fixed Frame** to `odom`. This is a change from Lesson 1: RViz now draws everything in the world frame.
2. **Add → By display type → TF**. Under the TF display, turn on **Show Names**. You will see three sets of axes: `odom`, `base_link` and `laser`. Check that `laser` points the opposite way to `base_link`.
3. **Add → By topic → /scan → LaserScan**. Set **Size** to `0.03` and **Decay Time** to `5`.
4. **Add → By topic → /odom → Odometry**. Under **Covariance**, untick it. Set **Keep** to `50`. The display draws an arrow at each recent pose, which shows the robot's path.
5. Save the config with **File → Save Config As** → `~/<your_name>_lesson2.rviz`.

## Step 7: Move the robot and watch

Use the **Unitree handheld remote** to walk the robot slowly. The remote is the safest option, and it is also how you will drive while mapping in Lesson 3.

Watch:

- `base_link` and `laser` moving away from `odom`.
- The trail of odometry arrows.
- The laser points. With **Decay Time = 5**, scans from the last five seconds overlap. **If the TF is correct, walls stay sharp as the robot moves.**

### Experiment: what a wrong transform looks like

1. Turn the robot slowly in place. The walls should stay still in RViz.
2. Stop T3 with Ctrl+C. Restart it with `--yaw 0` instead of `--yaw 3.14159…`.
3. Turn the robot again. The scan is now flipped front-to-back. While the robot turns, the walls smear in arcs instead of staying put.
4. Restart T3 with the correct yaw.

This is exactly the error SLAM would bake into your map. A wrong lidar transform is the most common reason a map comes out blurry.

### Experiment: odometry drift

1. Mark the robot's starting position on the floor with tape.
2. Walk the robot around the room, about a 5 m loop, and stop it exactly on the tape.
3. Run `ros2 topic echo /odom --once --field pose.pose.position`. It should print (0, 0). How far off is it?

That error is **drift**, and it gets worse the further the robot walks. The SLAM node in Lesson 3 fixes it by matching laser scans against the map.

---

## Optional: drive from the keyboard

Instead of the remote, you can drive with ROS velocity commands. Nav2 drives the robot this way in Lesson 5.

`teleop_twist_keyboard` publishes `geometry_msgs/Twist` messages on `/cmd_vel`. A bridge node, `cmd_vel_node`, turns each message into a Unitree **Move** request (`api_id 1008`) on `/api/sport/request`.

> **⚠ Safety: read before you run this.**
> - Starting `cmd_vel_node` immediately sends Unitree `api_id 1004` (**StandUp**). The robot stands up as soon as the node starts. Clear the area first.
> - Keep the handheld remote in someone's hand. It can override ROS at any time.
> - Only **one** person runs `cmd_vel_node` and teleop at a time.
> - The bridge caps speed at 0.5 m/s forward, 0.3 m/s sideways and 1.0 rad/s turning. If no command arrives for 0.5 s, it sends a stop.

```bash
# T5: velocity bridge
ros2 run mengram_pub cmd_vel_node

# T6: keyboard (start slow)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
```

Once the robot is standing, switch it to **Classic Walk** mode: on the handheld remote, press **→ (right arrow) + START**. In any other mode the robot ignores the Move commands and doesn't walk.

The teleop terminal must be the active window to read your keys. `i` moves forward, `,` moves back, `j`/`l` turn, and `k` stops. Hold Shift (`I`, `J`, `L`…) to step sideways. Because of the 0.5 s stop rule, the robot stops shortly after you release a key.

Check the command flow:

```bash
ros2 topic echo /cmd_vel              # what teleop sends
ros2 topic echo /api/sport/request    # what the bridge forwards to the robot
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `/odom` doesn't appear, or `ros2 topic hz /odom` prints nothing | Check that `/lf/sportmodestate` is publishing (Step 3). If it isn't, the robot's main board is not running sport mode. Restart the robot. |
| RViz: `Frame [odom] does not exist` | `sportstate_to_odom` is not running (T2). |
| RViz: `Could not transform from [laser] to [odom]` | The static transform is not running (T3). |
| Odometry jumps back to (0, 0) | Someone restarted `sportstate_to_odom`. It resets the origin each time it starts. |
| RViz flickers, or the robot jumps between two poses | Two people are running the same node. Run `ros2 node list` and look for duplicates. |
| Teleop: the robot doesn't move | Is `cmd_vel_node` running? Is the robot standing? Is it in Classic Walk mode (→ + START on the remote)? Is the teleop terminal the active window? |

## Summary

```bash
# T1  lidar
ros2 launch go2_slam_nav rplidar_a3.launch.py
# T2  odometry  (odom -> base_link)
ros2 run go2_state_bridge sportstate_to_odom
# T3  lidar mount (base_link -> laser)
ros2 run tf2_ros static_transform_publisher --x 0.0 --z 0.30 --yaw 3.141592653589793 --frame-id base_link --child-frame-id laser
# T4  inspect + view
ros2 run tf2_ros tf2_echo odom laser
rviz2          # Fixed Frame = odom; add TF, /scan, /odom
```

**Next lesson:** `go2_a3_mapping.launch.py` starts T1–T3 for you, adds **slam_toolbox**, and you will build and save a map of the room.
