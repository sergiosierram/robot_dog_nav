# Lesson 4: Localization on a saved map with AMCL

In Lesson 3 you built a map with slam_toolbox and saved it to `~/go2_maps/`. SLAM was building the map and locating the robot on it **at the same time**.

Once the map exists, we don't need to keep building it. We only need to answer one question: **where is the robot on this map right now?** This is **localization**, and it is what Nav2 relies on in Lesson 5 to plan paths.

In this lesson you will:

1. Load your saved map with **map_server**.
2. Start **AMCL**, a particle filter that tracks the robot's pose on the map.
3. Tell AMCL where the robot starts, using RViz's **2D Pose Estimate** tool.
4. Walk the robot and watch the particle cloud converge.
5. Break localization on purpose and watch AMCL recover.

**Before you start:** complete Lesson 3 and have a saved map, for example `~/go2_maps/<group_name>_map.yaml`. You need a Linux laptop for RViz. Have the Unitree handheld remote ready to drive.

---

## Background

### Same frame tree, different source for `map → odom`

The tree is the same as in Lesson 3. Only the node that publishes the top link changes:

```
map ──► odom ──► base_link ──► laser
 │        │
 │        └─ from sportstate_to_odom: smooth, but drifts
 └─ from AMCL (Lesson 3: from slam_toolbox)
```

AMCL never moves `odom` or `base_link` itself. It publishes `map → odom`, which is the correction that makes `map → base_link` match the real position in the room. That is exactly the role slam_toolbox played in Lesson 3.

### map_server

`map_server` reads your `.yaml` and `.pgm` files and publishes the grid on `/map`. The map is fixed: nothing is added to it while the robot walks. People who walk past, or a chair that has moved, do **not** appear on the map. AMCL has to cope with them.

### AMCL: a particle filter

**AMCL** (Adaptive Monte Carlo Localization) keeps hundreds or thousands of guesses of the robot's pose. Each guess is called a **particle**: one (x, y, yaw) on the map. Every time the robot moves, AMCL repeats three steps:

1. **Predict.** Move every particle by the odometry change, plus some random noise, because odometry is not perfect.
2. **Weigh.** For each particle, project the current laser scan onto the map from that pose. If the laser points land on walls, the particle gets a high weight. If they land in free space, it gets a low weight.
3. **Resample.** Draw a new set of particles, keeping more copies of the heavy ones. Bad guesses die out, good ones multiply.

At first the particles are spread out, because AMCL isn't sure where the robot is. As the robot walks and sees more of the room, they **converge** into a tight cluster. The "Adaptive" part means AMCL uses many particles while it's uncertain and fewer once they converge, between 1000 and 5000 in our config.

> **AMCL needs a starting guess.** Our config sets `set_initial_pose: false`, so AMCL publishes **nothing** until you give it an initial pose. Until then there is no `map` frame.

### Lifecycle nodes

`map_server` and `amcl` are Nav2 **lifecycle nodes**. They start in an *unconfigured* state and do nothing until someone tells them to *configure* and then *activate*. That someone is `lifecycle_manager_localization`. With `autostart: true`, it brings both nodes up as soon as they start. You will see this in the log.

---

## Step 1: Start localization (T1)

Log in (`ssh -X unitree@192.168.123.18`) and run:

```bash
ros2 launch go2_slam_nav go2_a3_localization.launch.py \
  map:=$HOME/go2_maps/<group_name>_map.yaml \
  serial_port:=$(ls /dev/serial/by-id/*CP2102*)
```

> **Use `$HOME`, not `~`, in `map:=`.** The shell only expands `~` at the start of a word, so `map:=~/go2_maps/...` reaches the launch file as a literal `~` and map_server can't find the file.

> **Only one person in the group runs this.** It starts the lidar driver, odometry and the lidar transform, the same as the mapping launch. **Stop the mapping launch from Lesson 3 first.** Two lidar drivers can't share the port.

Without `map:=`, the launch file loads `test_search_map_edited_v1.yaml`, a map of a different room. AMCL would try to match your room against it and fail.

This launch file starts:

| Node | Same as… | Publishes |
|------|----------|-----------|
| `sllidar_node` | Lesson 2, T1 | `/scan` |
| `sportstate_to_odom` | Lesson 2, T2 | `/odom`, `odom → base_link` |
| `base_link_to_laser_tf` | Lesson 2, T3, **but with `x = 0.0`** | `base_link → laser` |
| `map_server` | **new** | `/map` |
| `amcl` | **new** | `/amcl_pose`, `/particle_cloud`, `map → odom` |
| `lifecycle_manager_localization` | **new** | starts `map_server` and `amcl` |

You don't need `use_sim_time:=false` here, unlike Lesson 3. This launch file sets it to `false` by default.

The log should end with:

```
[map_io]: Read map /home/unitree/go2_maps/<group_name>_map.pgm: <width> X <height> map @ 0.05 m/cell
[lifecycle_manager_localization]: Managed nodes are active
[amcl]: AMCL cannot publish a pose or update the transform. Please set the initial pose...
```

The last warning repeats every two seconds until you complete Step 4. That is expected.

> **Remember the lidar exercise from Lesson 2?** This launch file puts the lidar at `x = 0.0`. The mapping launch file put it at `x = 0.15`. Your map was built with 0.15. With 0.0, every scan is drawn 15 cm too far back. AMCL partly absorbs the error by shifting the robot's estimated pose, but the estimate is then off by up to 15 cm, and the error grows when the robot turns. Compare with your ruler measurement. Your instructor will tell you which value the class uses.

## Step 2: Check it's running (T2)

```bash
ros2 lifecycle get /map_server                 # active [3]
ros2 lifecycle get /amcl                       # active [3]
ros2 topic echo /map --once --field info       # your map's size and resolution
ros2 run tf2_ros tf2_echo map odom             # "frame does not exist" for now
```

`tf2_echo` keeps reporting `Invalid frame ID "map"`. That is correct: AMCL doesn't publish `map → odom` until it has an initial pose. Leave it running. It will start printing a transform in Step 4.

## Step 3: Set up RViz (T3)

```bash
rviz2
```

1. Set **Fixed Frame** to `map`.
2. **Add → By topic → /map → Map**. Set **Durability Policy** to `Transient Local`, as in Lesson 3.
3. **Add → By topic → /scan → LaserScan**. Set **Size** to `0.05` and pick a bright **Color**, for example red. Set **Color Transformer** to `FlatColor`.
4. **Add → By display type → TF**.
5. **Add → By display type → nav2_rviz_plugins → ParticleCloud**. Set **Topic** to `/particle_cloud`, and under it set **Reliability Policy** to `Best Effort`. AMCL publishes the particles as "best effort", so the default "reliable" setting receives nothing.
6. **Add → By topic → /amcl_pose → PoseWithCovariance**. This shows AMCL's best estimate as an arrow, and its uncertainty as an ellipse.
7. In **Views**, set **Type** to `TopDownOrtho`.

Only the map appears for now. The scan, particles and robot frames can't be drawn in `map` until AMCL knows where the robot is.

Save the config with **File → Save Config As** → `~/<your_name>_localization.rviz`.

## Step 4: Give AMCL the starting pose

### In RViz: 2D Pose Estimate

1. Find the robot's real position on the map. Look at where it stands in the room, and find the same spot among the walls on the map.
2. Click **2D Pose Estimate** in the toolbar at the top.
3. **Click** on the map where the robot stands. Keep the mouse button held down and **drag** in the direction the robot is facing. A green arrow follows the mouse. Release.

RViz publishes your guess on `/initialpose`. AMCL scatters particles around it: about ±0.5 m in position and ±15° in heading. Immediately:

- `tf2_echo map odom` in T2 starts printing a transform.
- The red laser scan appears on top of the map.
- A cloud of green arrows appears around the robot: the particles.

**Look at the scan.** If your guess was good, the red points sit on the black walls of the map. If they are shifted or rotated, try again with **2D Pose Estimate**. You can repeat it as often as you like.

### From the command line

`(0, 0, 0)` on the map is the spot where the robot stood when you **started mapping** in Lesson 3. If the robot is back on that spot, facing the same way, you can set the pose without RViz:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: map},
  pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}},
         covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                      0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```

The `covariance` is the uncertainty of your guess: a variance of 0.25 m² is a standard deviation of 0.5 m in x and y, and 0.0685 rad² is about 15° in yaw. This is what RViz sends too. This is how the combined launch file in Lesson 5 sets the start pose automatically.

## Step 5: Walk the robot and watch it converge

Check AMCL's estimate:

```bash
ros2 topic echo /amcl_pose --field pose.pose.position
```

Now drive slowly with the **handheld remote**: a metre forward, a turn on the spot, a few steps sideways.

Watch in RViz:

- **The particle cloud shrinks.** Each new scan rules out the particles whose view doesn't match the map.
- **The ellipse around `/amcl_pose` shrinks** with it.
- **The scan stays on the walls** while the robot moves.

> **Particles only update when the robot moves.** AMCL runs a new update after 2 cm or about 1° of motion (`update_min_d`, `update_min_a`). While the robot stands still, the cloud doesn't change. To force an update without moving, run:
>
> ```bash
> ros2 service call /request_nomotion_update std_srvs/srv/Empty
> ```

### Experiment: a bad initial guess

1. Use **2D Pose Estimate** to put the robot about **1 m** away from its true position, facing the right way.
2. The scan no longer lines up with the walls.
3. Walk the robot slowly. Does AMCL find the true pose again? How far do you need to walk?
4. Repeat with the heading **90°** wrong. Does it recover?

AMCL can correct a guess that is roughly right, because some particles land near the true pose and get high weights. If no particle is close to the truth, there is nothing for it to keep.

### Experiment: the kidnapped robot

The hardest case: AMCL has no idea where the robot is. Spread the particles over the **whole** map:

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

The particle cloud now covers every free cell of the map. Walk the robot around the room, and turn on the spot from time to time. Watch the particles gather into a few clusters at places that look alike, and eventually into one.

This takes longer than a good initial guess, and in a room with symmetric features it may pick the wrong cluster. That is why we normally give AMCL a starting pose.

### Experiment: AMCL vs. odometry

Run this in T2 while you walk a loop:

```bash
ros2 run tf2_ros tf2_echo map odom
```

As in Lesson 3, this transform grows as odometry drifts. The difference: slam_toolbox corrected the drift by adding to the map. AMCL corrects it by matching against a map that never changes.

---

## The settings

Parameters are in `~/demo_ws/src/unitree-go2-slam-nav2/go2_slam_nav/config/go2_a3_amcl.yaml`:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `robot_model_type` | `OmniMotionModel` | The robot can walk sideways, not only forward and turn. A wheeled robot would use `DifferentialMotionModel`. |
| `min_particles` / `max_particles` | `1000` / `5000` | Fewer particles once AMCL is sure, more while it is unsure |
| `alpha1`–`alpha5` | `0.5` | How much noise to add to the odometry in the predict step. High values: AMCL trusts odometry less, so the cloud spreads more as the robot walks. |
| `laser_model_type` | `likelihood_field` | How a scan is scored against the map: by the distance from each laser point to the nearest wall |
| `laser_max_range` | `8.0` | Same as mapping. Points further away are ignored. |
| `max_beams` | `120` | Only 120 of the ~1,600 points in each scan are used for scoring, to save CPU |
| `update_min_d` / `update_min_a` | `0.02` m / `0.02` rad | Motion needed before a new update |
| `transform_tolerance` | `2.0` s | How far into the future the `map → odom` transform is stamped. Other nodes can use it without waiting for the next update. |
| `set_initial_pose` | `false` | Wait for `/initialpose` instead of assuming a starting pose |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `AMCL cannot publish a pose or update the transform. Please set the initial pose...` keeps repeating | Normal until you set the initial pose (Step 4). |
| `Error, unexpected error, code: 80008004` from `sllidar_node` | Another lidar driver is running, probably the mapping launch from Lesson 3. Stop it. Check with `ps aux \| grep sllidar`. Always pass `serial_port:=$(ls /dev/serial/by-id/*CP2102*)`. |
| `[map_server]: Original error: Failed to load map yaml file` | The `map:=` path is wrong, and the localization stack doesn't start. Use `$HOME`, not `~`, and the full path to the `.yaml` file. Check with `ls $HOME/go2_maps/`. |
| The map shows the wrong room | You forgot `map:=`, so the default map was loaded. |
| The ParticleCloud display is empty | Set **Reliability Policy** to `Best Effort`. Make sure you set an initial pose. The particles only change when the robot moves. |
| Map display empty | Set **Durability Policy** to `Transient Local`. Fixed Frame must be `map`. |
| The scan never lines up with the walls, even after walking | The initial pose is too far off. Set it again with **2D Pose Estimate**. Also check that you loaded the map of **this** room, and that the furniture hasn't moved much since mapping. |
| The pose jumps between two places | AMCL is torn between two similar-looking spots. Walk to a place with distinctive features and turn on the spot. |
| Everything jumps back to the start | Someone restarted the launch file. `sportstate_to_odom` resets odometry, and AMCL needs a new initial pose. |

## Summary

```bash
# T1  everything: lidar + odom + lidar TF + map_server + AMCL
ros2 launch go2_slam_nav go2_a3_localization.launch.py \
  map:=$HOME/go2_maps/<group_name>_map.yaml \
  serial_port:=$(ls /dev/serial/by-id/*CP2102*)

# T2  check
ros2 lifecycle get /amcl                       # active [3]
ros2 run tf2_ros tf2_echo map odom             # appears after the initial pose

# T3  view (Fixed Frame = map; add Map, LaserScan, TF, ParticleCloud [Best Effort], /amcl_pose)
rviz2
# then: 2D Pose Estimate → click where the robot is, drag the way it faces
```

**Next lesson:** keep localization running, add **Nav2**, and let the robot plan and walk to a goal you click in RViz.
