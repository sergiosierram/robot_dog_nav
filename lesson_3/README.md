# Lesson 3: Building a map with SLAM

In Lesson 2 you saw that odometry **drifts**: walk a loop, come back to the start, and `/odom` no longer reads (0, 0).

**SLAM** (Simultaneous Localization And Mapping) fixes this. It matches each new laser scan against the map built so far. It uses that match to correct the robot's pose, and the corrected pose to extend the map.

In this lesson you will:

1. Start the full mapping stack with a single launch file.
2. Walk the robot around the room and watch the map grow in RViz.
3. Watch SLAM correct the odometry drift.
4. Save the map to a file. Lesson 4 loads it.

**Before you start:** complete Lessons 1 and 2. You need a Linux laptop for RViz. Have the Unitree handheld remote ready to drive.

---

## Background

### The `map` frame

SLAM adds one frame to the tree from Lesson 2:

```
map ──► odom ──► base_link ──► laser
 │        │
 │        └─ from sportstate_to_odom: smooth, but drifts
 └─ from slam_toolbox: fixed to the room
```

slam_toolbox publishes `map → odom`. This transform is the **drift correction**. The robot's true pose is `map → base_link`, which is SLAM's correction combined with odometry. If odometry were perfect, `map → odom` would stay at zero forever.

### Occupancy grid

The map is a `nav_msgs/OccupancyGrid`: a 2D image where each pixel is a 5 cm × 5 cm cell.

| Colour (in the saved image) | Value | Meaning |
|-----------------------------|-------|---------|
| White | 0 | Free: the laser passed through it |
| Black | 100 | Occupied: the laser hit something here |
| Grey  | −1 | Unknown: the laser hasn't seen this cell yet |

### slam_toolbox

We use [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) in **online asynchronous** mode. It builds the map live while the robot moves. When it falls behind, it skips scans instead of queuing them.

Behind the scenes it keeps a **pose graph**: one node per scan, linked by the robot's estimated motion between them. When the robot comes back to a place it has already seen, slam_toolbox detects a **loop closure**. It adds a constraint between the old and new nodes and re-optimises the whole graph, which removes the drift that built up along the loop.

---

## Step 1: Start mapping (T1)

Log in (`ssh -X unitree@192.168.123.18`) and run:

```bash
ros2 launch go2_slam_nav go2_a3_mapping.launch.py use_sim_time:=false
```

> **⚠ Don't forget `use_sim_time:=false`.** This launch file includes slam_toolbox's own `online_async_launch.py`, which defaults to `use_sim_time:=true`. With that setting, slam_toolbox waits for a simulator clock on `/clock`. A real robot has none, so slam_toolbox sits there with **no error message**: no map, and no `map` frame. You can check the setting with `ros2 param get /slam_toolbox use_sim_time`. It must print `False`.

> **Only one person in the group runs this.** It starts the lidar driver, and only one program can use the serial port at a time.

This one launch file starts everything you started by hand in Lesson 2, plus SLAM:

| Node | Same as Lesson 2… | Publishes |
|------|-------------------|-----------|
| `sllidar_node` | T1 | `/scan` |
| `sportstate_to_odom` | T2 | `/odom`, `odom → base_link` |
| `static_transform_publisher` | T3 (x=0.15, z=0.30, yaw=π) | `base_link → laser` |
| `slam_toolbox` | **new** | `/map`, `map → odom` |

You should see a line like `Using solver plugin solver_plugins::CeresSolver`. You may also see this warning, which is harmless:

```
minimum laser range setting (0.0 m) exceeds the capabilities of the used Lidar (0.1 m)
```

### If the lidar fails to start

The lidar does not always appear as `/dev/ttyUSB0`. If you unplug and re-plug it, it can come back as `/dev/ttyUSB1`. You then get `Error, unexpected error, code: 80008004`. Use the lidar's stable name instead, which never changes:

```bash
ls /dev/serial/by-id/
# usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_..._if00-port0

ros2 launch go2_slam_nav go2_a3_mapping.launch.py use_sim_time:=false \
  serial_port:=$(ls /dev/serial/by-id/*CP2102*)
```

### The settings

Parameters are in `~/demo_ws/src/unitree-go2-slam-nav2/go2_slam_nav/config/slam_toolbox_go2_a3.yaml`:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `mode` | `mapping` | Build a new map. The other option is `localization`, against a saved map. |
| `resolution` | `0.05` | Size of one map cell in metres |
| `max_laser_range` | `8.0` | Ignore laser points beyond 8 m, which are noisy. The A3 can reach 25 m. |
| `minimum_travel_distance` / `_heading` | `0.02` m / `0.02` rad | Add a new scan to the graph after this much motion. Small values give a detailed map but use more CPU. |
| `map_update_interval` | `0.2` s | How often `/map` is republished |
| `use_scan_matching` | `true` | Correct odometry by matching scans. Set it to `false` and see what happens. |

## Step 2: Check it's running (T2)

```bash
ros2 param get /slam_toolbox use_sim_time      # Boolean value is: False
ros2 topic echo /map --once --field info       # resolution 0.05, width and height in cells
ros2 run tf2_ros tf2_echo map odom             # the drift correction, about 0 at the start
```

## Step 3: View the map in RViz (T2)

```bash
rviz2
```

1. Set **Fixed Frame** to `map`.
2. **Add → By topic → /map → Map**. Under the Map display, expand **Topic** and set **Durability Policy** to `Transient Local`. RViz then receives the latest map as soon as it connects, instead of waiting for the next update.
3. Add the displays from Lesson 2: **TF**, **/scan → LaserScan** (Size `0.03`) and **/odom → Odometry**.
4. In **Views**, set **Type** to `TopDownOrtho`.

Even before the robot moves, a first patch of map should appear: the area the lidar can see from where the robot stands.

Save the config with **File → Save Config As** → `~/<your_name>_mapping.rviz`.

## Step 4: Walk the robot and build the map

Drive with the **handheld remote**. How you drive decides how good the map is:

- **Go slowly**, especially when turning. The lidar only scans 10 times per second, and fast turns blur the map. Turning is worse than driving straight.
- **Follow the walls** and visit every part of the room you will want to navigate to later.
- **Close the loop.** End where you started, then drive part of the loop a second time. This is what triggers loop closure.
- **Keep people out of the scan.** People in the room become black blobs on the map. Ask the class to stand still at the edges, or behind the robot.
- **Watch out for glass, mirrors and very dark surfaces.** The laser passes through glass or bounces off it, which creates false free space or ghost walls.

While you drive, watch:

- **Grey turning into white and black** as the robot explores.
- **The `map` and `odom` axes in the TF display.** At first they sit on top of each other. As drift builds up, they slowly separate.
- **Loop closures.** When the robot comes back to the start, the map may visibly "snap": doubled walls merge into one, and the `odom` axes jump. That is slam_toolbox correcting the drift you measured in Lesson 2.

### Experiment: measure the drift correction

Run this in T2 while you drive:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Translation starts near `[0, 0, 0]`. After a long walk it no longer reads zero: that is how far odometry has drifted. Compare it with your drift measurement from Lesson 2.

## Step 5: Save the map

When the map looks complete, **keep the mapping launch running** and save it from T2:

```bash
mkdir -p ~/go2_maps
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/<group_name>_map
```

The log should say `Map saved successfully`. You now have two files:

- `<group_name>_map.pgm`: the image (white, black and grey, as above).
- `<group_name>_map.yaml`: the metadata, for example:

  ```yaml
  image: <group_name>_map.pgm
  mode: trinary
  resolution: 0.05            # metres per pixel
  origin: [-7.03, -5.42, 0]   # world position (x, y, yaw) of the image's bottom-left pixel
  negate: 0
  occupied_thresh: 0.65       # cells above 65% are treated as occupied
  free_thresh: 0.25           # cells below 25% are treated as free
  ```

> **Use your own group name.** `~/go2_maps/` is shared by everyone who uses the robot. A file with the same name overwrites someone else's map.

### Optional: save the pose graph too

The `.pgm` file is only a picture. To **continue mapping later**, or to re-optimise the map, save slam_toolbox's internal pose graph:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /home/unitree/go2_maps/<group_name>_posegraph}"
```

`result=0` means success. This writes `<group_name>_posegraph.data` and `<group_name>_posegraph.posegraph`.

### Look at your map

Copy it to your laptop and open the `.pgm` file in any image viewer:

```bash
# on your laptop
scp unitree@192.168.123.18:~/go2_maps/<group_name>_map.* .
```

Stray dots, people-shaped blobs, or walls that "leak" through glass can be cleaned up in an image editor such as GIMP. Paint unwanted obstacles white and close gaps in black. Save under a new name, for example `<group_name>_map_edited.pgm`, and update the `image:` line in a copy of the `.yaml` file. This is how the `*_edited_v1` maps in `~/go2_maps/` were made.

Then stop mapping in T1 with **Ctrl+C**.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| No `/map`, and RViz says `Frame [map] does not exist` | You forgot `use_sim_time:=false`. Check with `ros2 param get /slam_toolbox use_sim_time`. |
| `Error, unexpected error, code: 80008004` from `sllidar_node` | The lidar is on a different port, or another lidar driver is already running. See "If the lidar fails to start" in Step 1. Check for duplicates with `ps aux \| grep sllidar`. |
| The map is blurry, or walls appear twice | You drove or turned too fast, or the `base_link → laser` transform is wrong (Lesson 2 experiment). Stop, start the launch again, and map more slowly. |
| The map rotates or jumps wildly while the robot turns in place | Scan matching failed. This is common in long empty corridors or wide open spaces with nothing in range. Stay within about 8 m of walls. |
| Map display in RViz is empty, but `/map` exists | Fixed Frame must be `map`. Set Durability Policy to `Transient Local`. |
| `map_saver_cli` fails or times out | Mapping must still be running when you save. Check that `/map` is publishing. |
| Your map overwrote someone else's | Always use your group name in the filename. |

## Summary

```bash
# T1  everything: lidar + odom + lidar TF + slam_toolbox
ros2 launch go2_slam_nav go2_a3_mapping.launch.py use_sim_time:=false

# T2  view (Fixed Frame = map; add Map, TF, LaserScan, Odometry)
rviz2

# drive slowly with the remote, close the loop, then save:
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/<group_name>_map
```

**Next lesson:** turn SLAM off and **localize** the robot on the saved map with AMCL.
