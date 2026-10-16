# ego_ground_sim

Gazebo Classic (11) simulation for the ROS 2 port of the ground-robot
ego-planner. It closes the loop *robot <-> planner* without depending on the
(still ROS 1) CMU simulation environment.

## V550_mec + 2D LiDAR

Use `ros2 launch ego_ground_sim sim_v550_mec_2d.launch.py` for the vendor V550
visual model, a single-plane LiDAR and the CMU indoor scene. This dedicated
profile uses ideal planar motion and `/v550_sim/` command topics, with no
Patchwork or hardware drivers. The differential-drive / 3D LiDAR descriptions
below apply to the older simulation profiles.
See [V550 仿真步骤](../V550仿真.md) for the full startup and verification steps.
Use `sim_v550_custom.launch.py` for the empty arena. The V550 profiles default to
an overhead camera and show the sampled EGO spline in green and odometry history
in orange. See [自定义地图与轨迹显示](../V550自定义地图.md) for placing obstacles,
saving/reloading a `.world`, and the measured detour check.
Use `sim_v550_rooms.launch.py` for the 24 x 18 m six-room building, open doors,
30 furniture/obstacle models and visible laser rays. The V550 profiles enable
the scan collision guard, run at up to 0.22 m/s and reset planning to measured
odometry when tracking error exceeds 0.12 m. See
[大型室内场地与验证](../V550室内仿真.md) for startup, limitations and geometry-based checks.
V550 simulation is forward-only and uses a measured-obstacle repulsion direction
to exit soft inflation overlap before replanning the original goal. See
[膨胀区脱困](../V550膨胀区脱困.md) for the hard body envelope and recovery limits.

## What it provides

| topic | type | published by |
| --- | --- | --- |
| `/state_estimation` | `nav_msgs/Odometry` | differential drive plugin |
| `/scan_3d` | `sensor_msgs/PointCloud2` | Gazebo 3D LiDAR (sensor frame) |
| `/registered_scan` | `sensor_msgs/PointCloud2` | `point_cloud_transformer` (map frame, points really transformed) |
| `/benchmark/{cloud,P,N}` | `sensor_msgs/PointCloud2` | patchwork `ground_segmentation` |
| `/odom`, `/cmd_vel_safe` | odometry / twist | chassis (plugin + watchdog) |
| TF `map -> odom -> base_footprint -> base_link -> {wheels, lidar_link}` | tf2 | static publisher, diff drive plugin, robot_state_publisher |

Consumed: `/cmd_vel` (planner) -> `cmd_vel_safety` -> `/cmd_vel_safe` -> wheels.

## Chassis model

Forward + steering (unicycle) model, i.e. exactly what the MPC of this project
assumes: `linear.x` = forward speed, `angular.z` = yaw rate. Implemented with
`libgazebo_ros_diff_drive.so` (two driven wheels, wheel separation 0.37 m,
wheel diameter 0.12 m) - the plugin integrates a proper physics simulation, it
does *not* teleport the model.

## Safety / failure behaviour

`cmd_vel_safety` publishes zero velocity (and logs a warning) when

* no `/cmd_vel` arrives for more than `cmd_timeout` (default 0.5 s) - planner
  stopped or crashed,
* no `/state_estimation` arrives for more than `odom_timeout` (default 1.0 s) -
  state estimate lost,
* the command contains NaN/Inf.

Commands are clamped to `max_linear` / `max_angular`.

## Known pitfall: gzclient exits right after start up

Symptom: RViz2 works, Gazebo opens and immediately disappears (闪退); gzserver
keeps running and `/clock`, `/state_estimation`, `/scan_3d` are all published.

Reason: `GAZEBO_RESOURCE_PATH` did not contain Gazebo's own media directory. The
`gazebo_ros` launch files only export the paths of *ROS packages* that declare
`<gazebo_ros>` exports and append whatever the shell had - if the shell never
sourced `/usr/share/gazebo/setup.sh`, Gazebo's `media` directory is missing. A
*set but incomplete* `GAZEBO_RESOURCE_PATH` replaces Gazebo's built-in defaults,
so OGRE cannot find `gazebo.material`:

```
[Err] [RenderEngine.cc:197] Failed to initialize scene      # gzclient log
OGRE EXCEPTION: Cannot locate material called 'Gazebo/shadow_caster'   # ~/.gazebo/ogre.log
gzclient: boost::shared_ptr<gazebo::rendering::Camera>::operator->(): Assertion `px != 0' failed.
```

This launch file therefore starts `gzserver`/`gzclient` itself with a complete
environment (`GAZEBO_RESOURCE_PATH`, `GAZEBO_MODEL_PATH`, `GAZEBO_PLUGIN_PATH`,
`OGRE_RESOURCE_PATH`) taken from `/usr/share/gazebo-11`, so it works even without
sourcing Gazebo's setup script. `gazebo_resource_path:=...` overrides it. If you
start Gazebo by hand, run `source /usr/share/gazebo/setup.sh` first.

## The robot is not visible in the Gazebo window

The robot *is* spawned - check it without ROS:

```bash
gz model -m ego_ground_robot -p        # prints the pose, e.g. -7.99 -5.99 0.0 ...
cat ~/.gazebo/server-11345/default.log # "Loading world file ...", "applying Gaussian noise model"
```

It is just not inside the initial camera view: gzclient's default user camera is centred on
the origin and covers roughly the middle of the map, while the robot starts in the corner of
the 30 x 30 m arena (default spawn `-8, -6`) and its body is only 0.36 m across (about 1 % of
the window width). RViz2 shows the same robot in its RobotModel display, which is why the
planner obviously works.

How to see it:

* in gzclient switch the left panel to the **World** tab, expand the model tree and
  **double-click `ego_ground_robot`** (or right-click it -> *Move To* / *Follow*) - the camera
  jumps to the robot;
* or zoom out with the mouse wheel until the whole arena is visible;
* or start the robot near the middle of the arena, e.g.
  `ros2 launch ego_ground_sim sim_ego_planner.launch.py spawn_x:=-2.0 spawn_y:=2.0`.

If the *World* tree does not list `ego_ground_robot` either, the spawn really failed - usually
because an old gzserver is still running (`pgrep -a gzserver`, then kill it) or because the
spawn_entity output in the launch log reports an error.

## Usage

```bash
ros2 launch ego_ground_sim sim_ego_planner.launch.py              # gui + rviz
ros2 launch ego_ground_sim sim_ego_planner.launch.py gui:=false rviz:=false   # headless
ros2 launch ego_ground_sim ego_ground_sim.launch.py               # simulation only
```

Send a goal with the RViz2 *2D Goal Pose* tool (`/goal_pose`).

Useful launch arguments of `sim_ego_planner.launch.py`: `gui`, `rviz`,
`use_sim_time`, `odom_topic`, `pointcloud_topic`, `goal_topic` and
`mapping_cloud_topic` (default `/benchmark/N`, see below).
`ego_ground_sim.launch.py` additionally accepts `world`, `spawn_x`, `spawn_y`,
`spawn_z` and `spawn_yaw`.

### Why the mapper gets the segmented cloud

`libgazebo_ros_ray_sensor` returns the ground as well as the obstacles. The grid
mapper marks every point it receives as occupied and inflates it, so feeding the
raw scan (`/registered_scan`) into `/grid_map/cloud` marks the ground *and the
robot itself* as being inside an obstacle: every plan then aborts with
"First 3 control points in obstacles". The launch therefore feeds the mapping with
the ground-segmented cloud (`/benchmark/N`) by default; both the 2D safety map and
the 3D collision map are then built from real obstacles only. Set
`mapping_cloud_topic:=/registered_scan` if the scanner cannot see the ground.

### Simulation time

Every node runs with `use_sim_time:=true`; the planner/MPC/watchdog timers follow
the simulated clock. Note that a *stand-alone* `rclcpp::Clock` does not subscribe
to `/clock`, so timestamps must always come from the owning node
(`node->get_clock()`); otherwise the planner mixes wall clock and simulated time
(this was a real bug and is fixed in `EGOPlannerManager`).

## Verification

```bash
ros2 launch ego_ground_sim sim_ego_planner.launch.py gui:=false rviz:=false &
python3 install/ego_ground_sim/share/ego_ground_sim/scripts/closed_loop_check.py 6.0 4.5
```

Measured (2026-10-11): goal error 0.61 m after driving 22.41 m (17.20 m straight,
i.e. a detour around the obstacles), 34 successful plans / 0 failures, 21
published B-splines, max speed 1.13 m/s, minimum clearance to the five obstacles
0.67 m and no obstacle hit, then full stop at the goal (`/cmd_vel_safe` = 0,
position drift 0.000 m). The watchdog stops the robot 0.5 s after the command
source disappears, and a NaN command is turned into zero velocity.

## CMU indoor corridor scene

The repository already ships the CMU indoor scene (the one shown in the original
README animation): `.../vehicle_simulator/mesh/indoor` (mesh + textures + a
preview point cloud, ~129 x 99 x 3.9 m). `ego_ground_sim` installs that mesh as a
Gazebo model, so it can be used from ROS 2 without touching the (ROS 1) simulation
environment:

```bash
ros2 launch ego_ground_sim sim_cmu_indoor.launch.py                     # GUI + RViz2
ros2 launch ego_ground_sim sim_cmu_indoor.launch.py gui:=false rviz:=false
```

This uses `worlds/cmu_indoor.world` and `config/ego_planner_indoor.yaml`.

### Frame handling (mandatory for this scene)

patchwork's concentric zone model is centred on the **origin of the cloud**, so it
has to segment in the *sensor* frame. The planner, however, needs the segmented
cloud in the **map** frame. The node therefore transforms its input into
`sensor_frame` and its outputs into `output_frame`:

```yaml
ground_segmentation:
  ros__parameters:
    sensor_frame: "lidar_link"   # segment here ("" = use the input frame)
    output_frame: "map"          # publish here ("" = keep the input frame)
```

Without this, a map frame cloud is segmented around the *map origin*: in this
scene the robot is 66 m away from it, every point falls outside `max_r` and
`/benchmark/{P,N}` stay empty - the planner then sees no obstacle at all.

### Mesh shift and map size

`worlds/cmu_indoor.world` shifts the mesh by `(-62.8, -20.4, 0)` so that it is
centred on the origin, because the planner's occupancy buffer is centred on the
origin as well. The CMU original coordinate `(0, 0)` is therefore
`(-62.8, -20.4)` in this world. The default spawn of `sim_cmu_indoor.launch.py`
is `(-31.3, -23.4)`, near the red robot in `imag/ego_example.gif`, in the passage
above the rectangular enclosure. This corresponds to `(31.5, -3.0)` in the
original CMU frame, with about 1.3 m wall clearance in the preview cloud.
Floor support under the robot and 2 m along the passage was checked against the
collision mesh. The default yaw is `-pi/2`, facing along -Y. Both Gazebo and
RViz cameras look at the spawn area. Nearby goals on the same floor are
`(-31.3, -25.4)` and `(-31.3, -21.4)`; navigation from this spawn has not yet
been tested. The scene centre contains an atrium without floor, so a visually
empty place is not always a safe spawn.
The buffer is 140 x 110 x 2.5 m at 0.1 m resolution
(~0.6 GB); raise `grid_map/resolution` to 0.15 if that is too much.

### Picking goals

The scene consists of several rooms; a goal that looks free can still be behind a
wall. Use the helper (it reads the preview point cloud, builds an occupancy grid,
inflates it by the robot radius and only reports cells *connected* to the spawn
via observed floor; results are approximate candidates, not a navigation guarantee):

```bash
python3 install/ego_ground_sim/share/ego_ground_sim/scripts/indoor_free_space.py \
        -31.3 -23.4 8 25          # spawn_x spawn_y min_dist max_dist
```

### Verified behaviour

The indoor runs below used the earlier spawn `(-62.8, -20.4)`.

| run | result |
| --- | --- |
| 13 m hop along the corridor, goal (-50.0, -19.0) | **REACHED**, goal error 0.42 m, 0 emergency stops, stopped at the goal |
| 32 m goal into another room, goal (-32.75, -10.0) | drove 25-29 m through the scene with avoidance and no collision, then oscillated ~14 m short (see limitation) |
| arena regression after the changes | goal error 0.68 m, 21.1 m detour, no collision |
| real time factor of the big mesh | ~0.95 (16 x 360 ray sensor, 0.001 s physics step) |

Known limitation: EGO plans on a *local* map (`local_update_range` 8 m) and treats
unknown space as free, so multi-room goals can end in a local minimum (the robot
oscillates at a junction). For long-range missions either send waypoints that stay
in one corridor, or add a global planner that feeds a reference path (see the
project documentation for the "navigation" setup).

### Corridor specific parameters

`config/ego_planner_indoor.yaml` differs from the arena configuration in
`obstacles_inflation` (0.22 m instead of 0.35, the robot is 0.36 m across),
`control_points_distance` (0.5), `lambda_collision` (1.2) and
`relaxed_initial_collision: true`. The last one matters: if the robot ends up
closer to a wall than `obstacles_inflation` (tracking error in a narrow corridor),
the upstream code rejects *every* plan ("First 3 control points in obstacles"), the
FSM falls back to emergency stops and the robot can never drive out. With the flag
the first trajectory segment is tolerated (the robot is already there) while the
rest of the trajectory must still be collision free and the FSM safety check keeps
running. It defaults to `false`, so the arena configuration is unaffected.
