# Lesson 1: Connect to the Go2, start the lidar, view it in RViz

In this lesson you will:

1. Connect your laptop to the robot over Ethernet.
2. Log in to the robot's onboard computer with SSH, with X11 forwarding turned on so that its graphical programs open on your screen.
3. Start the RPLidar A3 driver.
4. Check the laser data from the command line, then view it in RViz.

## Background

The Go2 has two computers on its internal network, `192.168.123.0/24`:

| Address           | Machine                                | Can you SSH in?        |
|-------------------|----------------------------------------|------------------------|
| `192.168.123.161` | Main control board (locomotion)        | No, Unitree locks it   |
| `192.168.123.18`  | Expansion dock (Jetson, Ubuntu 22.04)  | Yes, we work here      |

The robot's built-in lidar is not used. Instead an **RPLidar A3** is mounted on the robot and plugged into the Jetson by USB, where it shows up as `/dev/ttyUSB0`. The Jetson runs **ROS 2 Humble**. Everything is already installed in `~/demo_ws`.

---

## Step 1: Network setup on your laptop

Plug an Ethernet cable from your laptop into the robot. Then give your laptop a fixed address on the robot's network. Use any free address from `.2` to `.254`, except `.18` and `.161`. In a group, each person needs a different address, for example `.100`, `.101`, `.102`…

**macOS**

```bash
networksetup -listallhardwareports              # find the adapter name for your Ethernet port
networksetup -setmanual "USB 10/100 LAN" 192.168.123.100 255.255.255.0
```

**Ubuntu**

```bash
ip link                                          # find the interface name, e.g. enp3s0
sudo ip addr add 192.168.123.100/24 dev enp3s0
```

Check that the robot answers:

```bash
ping -c 3 192.168.123.18
```

> When you are done with the robot, switch back to automatic addressing.
> macOS: `networksetup -setdhcp "USB 10/100 LAN"`. Ubuntu: `sudo ip addr del 192.168.123.100/24 dev enp3s0`.

## Step 2: Install an X server (needed to see RViz)

RViz runs on the robot, but its window is drawn on your laptop. That only works if your laptop runs an X server.

| Your OS | What to do |
|---------|------------|
| Ubuntu / Linux desktop | Nothing to install, it already has one. **This is the supported setup for this lesson.** |
| macOS   | **RViz does not work over `ssh -X` from a Mac.** XQuartz only supports OpenGL 1.4 for forwarded windows, and RViz2 needs at least 1.5. It fails with `OpenGL 1.5 is not supported in GLRenderSystem::initialiseContext`. Mac users: do Steps 1–5 on your Mac (they need no graphics), and pair with someone on a Linux laptop for Step 6. |
| Windows | Not tested. [MobaXterm](https://mobaxterm.mobatek.net/) has an X server, but RViz may hit the same OpenGL limit as on macOS. Pair with a Linux user if it fails. |

## Step 3: SSH into the robot with X11 forwarding

```bash
ssh -X unitree@192.168.123.18
```

- `-X` turns on X11 forwarding. If you get X authorization errors, use `-Y` instead, which is "trusted" forwarding.
- Your instructor will give you the password.

Once logged in, check that forwarding is working:

```bash
echo $DISPLAY        # should print something like localhost:10.0
```

If it prints an empty line, X forwarding is not active. See [Troubleshooting](#troubleshooting).

When you log in, the robot's `~/.bashrc` automatically loads the ROS 2 environment and the workspace (you will see `Setup unitree ros2 environment`). If you ever need to load it yourself, run:

```bash
source ~/unitree_ros2/setup.sh
source ~/demo_ws/install/setup.bash
```

You will need **two terminals logged in to the robot**. Open a second terminal on your laptop and run the same `ssh -X` command again.

## Step 4: Start the lidar (terminal 1)

```bash
ros2 launch go2_slam_nav rplidar_a3.launch.py
```

You should see the lidar spin up, and the log should contain lines like these:

```
[sllidar_node-1] [INFO] [1787935959.467529912] [sllidar_node]: SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:1.0.1, SLLIDAR SDK Version:2.1.0
[sllidar_node-1] [INFO] [1787935959.487394808] [sllidar_node]: SLLidar S/N: D3C5ED93C0EA98C6C2E29EF5211F4061
[sllidar_node-1] [INFO] [1787935959.487565400] [sllidar_node]: Firmware Ver: 1.32
[sllidar_node-1] [INFO] [1787935959.487600344] [sllidar_node]: Hardware Rev: 6
[sllidar_node-1] [INFO] [1787935959.488382648] [sllidar_node]: SLLidar health status : 0
[sllidar_node-1] [INFO] [1787935959.488512792] [sllidar_node]: SLLidar health status : OK.
[sllidar_node-1] [INFO] [1787935959.679347000] [sllidar_node]: current scan mode: Sensitivity, sample rate: 16 Khz, max_distance: 25.0 m, scan frequency:10.0 Hz,
```

The launch file is at `~/demo_ws/src/unitree-go2-slam-nav2/go2_slam_nav/launch/rplidar_a3.launch.py`. It starts the `sllidar_node` driver with these settings:

| Parameter         | Value          | Meaning                                    |
|-------------------|----------------|--------------------------------------------|
| `serial_port`     | `/dev/ttyUSB0` | USB serial device the lidar is plugged into |
| `serial_baudrate` | `256000`       | Baud rate required by the A3               |
| `frame_id`        | `laser`        | TF frame the scan is published in          |
| `scan_mode`       | `Sensitivity`  | A3 scan mode (16 kHz sample rate)          |

You can override any launch argument, for example: `ros2 launch go2_slam_nav rplidar_a3.launch.py serial_port:=/dev/ttyUSB1`.

> **Only one lidar driver can run at a time.** The serial port can be opened by only one program. If someone else in your group has already started the lidar, do not start it again. Go straight to Step 5. Every terminal logged in to the robot sees the same topics.

Leave this terminal running. Press **Ctrl+C** to stop the lidar.

## Step 5: Check the data from the command line (terminal 2)

```bash
ros2 topic list                       # /scan should be in the list
ros2 topic hz /scan                   # should be about 10–11 Hz (Ctrl+C to stop)
ros2 topic info /scan                 # message type: sensor_msgs/msg/LaserScan
ros2 topic echo /scan --once --field header
```

A `LaserScan` message holds one full 360° sweep. Its main fields are:

- `angle_min`, `angle_max`, `angle_increment`: the angle in radians of each reading.
- `ranges[]`: one distance in metres per angle. `inf` means nothing was hit within range.
- `range_min`, `range_max`: the distances the sensor can measure (about 0.15 m to 25 m for the A3).

## Step 6: View the scan in RViz (terminal 2)

### Option A: use the ready-made config

```bash
rviz2 -d $(ros2 pkg prefix sllidar_ros2)/share/sllidar_ros2/rviz/sllidar_ros2.rviz
```

### Option B: build the view yourself (recommended once, so you learn how RViz works)

```bash
rviz2
```

1. In the **Displays** panel on the left, open **Global Options** and set **Fixed Frame** to `laser`.
   - The fixed frame is the coordinate frame RViz draws everything in. So far the only frame that exists is `laser`. Nothing else publishes TF yet, so choosing a frame like `map` or `odom` would give a "Frame does not exist" error.
2. Click **Add** (bottom left), open the **By topic** tab, and pick `/scan` → **LaserScan**. Click OK.
3. Expand the new **LaserScan** display:
   - **Size (m)**: raise it to `0.03`–`0.05` so the points are easier to see.
   - **Color Transformer**: `Intensity` colours points by how strong the return was. `FlatColor` gives every point one colour.
   - **Decay Time**: set it to `1`–`2` seconds to keep old scans on screen as a trail.
4. Click **Add** → **By display type** → **Axes** to show where the sensor sits and which way it points. Red = x (forward), green = y (left), blue = z (up).
5. Move around the view:
   - Left-drag to rotate.
   - Middle-drag (or Shift + left-drag) to pan.
   - Scroll to zoom.
   - Under **Views** on the right, set **Type** to `TopDownOrtho` for a 2D bird's-eye view.

**Try this:** walk around the robot, or hold a box near it, and watch the points move. Find the lidar's forward direction (+x on the axes) and compare it to the robot's actual front. In a later lesson the robot will define its own body frame, `base_link`, and you'll see the lidar is mounted rotated 180° relative to it.

To keep your setup for next time, use **File → Save Config As** and save it to a file such as `~/<your_name>_lidar.rviz`.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ping` gets no answer | Check the cable. Check that your laptop's address is in `192.168.123.x` with netmask `255.255.255.0`. Make sure the robot is switched on (the Jetson takes about 1 minute to boot). |
| `echo $DISPLAY` is empty | You connected without `-X`/`-Y`, or no X server is running on your laptop (on macOS, start XQuartz first). |
| `rviz2` fails with `OpenGL 1.5 is not supported` / `libGL error: No matching fbConfigs` | You are connecting from macOS (or another X server without OpenGL support). This cannot be fixed with settings. Use a Linux laptop for Step 6 (see Step 2). |
| `rviz2` fails with other GLX / OpenGL errors on Linux | Try `ssh -Y`. If that doesn't help, try `LIBGL_ALWAYS_SOFTWARE=1 rviz2`. |
| RViz is very slow | This is normal: X11 sends every frame over the cable. Keep the window small and the point size low. |
| `SL_RESULT_OPERATION_TIMEOUT` or `Error, cannot bind to the specified serial port` | Another lidar driver is already running (see Step 4), or the USB cable is loose. Check with `ls -l /dev/ttyUSB*`. |
| `/scan` appears in the list but RViz shows nothing | Fixed Frame must be `laser`, and the LaserScan display must be subscribed to `/scan`. Look for red error text in the Displays panel. |
| `ros2: command not found` | The environment did not load. Run the two `source` lines from Step 3. |

## Summary

```bash
# laptop
ssh -X unitree@192.168.123.18                          # terminal 1
ssh -X unitree@192.168.123.18                          # terminal 2

# robot, terminal 1
ros2 launch go2_slam_nav rplidar_a3.launch.py

# robot, terminal 2
ros2 topic hz /scan
rviz2          # Fixed Frame = laser, Add → /scan → LaserScan
```

**Next lesson:** connecting the lidar to the robot's body (`base_link`) and odometry using TF, then building a map with SLAM.
