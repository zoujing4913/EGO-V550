# V550 unknown-area navigation

Use `ros2 launch ego_wheeltec v550_unknown_nav.launch.py` for online SLAM and RViz goals in unknown areas.
See [未知区域实车导航](V550未知区域实车导航.md) for dependencies, commissioning, enabling and limitations.

[toc]

# ROS 2 (Humble) port

The required Wheeltec base driver, serial library, LiDAR driver, messages and V550
models are now bundled in `wheeltec_vendor/`. Build and source only this workspace.
See [V550 单工作空间](V550单工作空间.md) for dependencies and launch commands.

This repository was originally a ROS 1 (melodic/noetic) project. It has been ported
to **ROS 2 Humble**; the ROS 1 sources are still in the tree but are excluded from
the ROS 2 build (see *What changed* below).

## Build

```bash
cd ~/ego_ground_ws            # the workspace that contains src/ego-planner-for-ground-robot
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Notes

* `qpOASES` does **not** have to be installed manually anymore: if no system
  installation is found, the copy inside `ego-planner/qpOASES` is compiled
  together with `mpc_controller` (built with `-fPIC` and linked statically into
  `libmpc_controller.so`). Installing it into `/usr/local` still works and is
  preferred if you already have it.
* `patchwork` needs `libboost-dev`, `libpcl-dev` and OpenCV, `plan_env` needs
  `libopencv-dev`; all of them are normally already present on a desktop install.

## Run

For the **V550_mec + 2D LiDAR simulation**, run
`ros2 launch ego_ground_sim sim_v550_mec_2d.launch.py`.
It uses the supplied WheelTec V550 meshes and the CMU indoor scene, with Gazebo
and RViz. Follow [V550 仿真步骤](V550仿真.md) for environment setup and goal commands.
For an empty arena to arrange your own obstacles, use
`ros2 launch ego_ground_sim sim_v550_custom.launch.py` and follow
[自定义地图与轨迹显示](V550自定义地图.md).
For the larger 24 x 18 m building with six rooms, open doors, furniture and visible
laser rays, use `ros2 launch ego_ground_sim sim_v550_rooms.launch.py` and follow
[大型室内场地与激光显示](V550室内仿真.md). V550 simulations now enable a scan-based
command collision guard and replan from measured odometry when tracking lags.
V550 simulation now uses forward-only commands and an inflation-exit recovery;
the room scene has 30 furniture/obstacle models. See [膨胀区脱困说明](V550膨胀区脱困.md).

For a **Wheeltec V550 real robot**, use the separate `ego_wheeltec` entry point;
follow [V550 实车部署流程](V550实车部署.md). Select the actual chassis and sensor
type explicitly. The default real-robot launch is observation only.
For the confirmed **V550_mec + 2D LiDAR** setup, run
`ros2 launch ego_wheeltec v550_mec_2d.launch.py` after starting the factory drivers.
This profile uses forward/reverse and yaw commands (`linear.y=0`).

```bash
ros2 launch ego_planner run_in_sim.launch.py \
    odom_topic:=/state_estimation pointcloud_topic:=/registered_scan
```

This starts four things: `ego_planner_node` (planning FSM + local map),
`traj_server` (MPC trajectory tracking, publishes `/cmd_vel`), the patchwork
`ground_segmentation` node (`/pointcloud_scan` -> `/benchmark/{cloud,P,N}`) and
RViz2. Useful launch arguments:

| argument | default | meaning |
| --- | --- | --- |
| `odom_topic` | `/state_estimation` | nav_msgs/Odometry used as the robot state |
| `pointcloud_topic` | `/registered_scan` | PointCloud2 used for mapping/ground segmentation |
| `goal_topic` | `/goal_pose` | PoseStamped goal ("2D Goal Pose" of RViz2) |
| `map_size_x/y/z` | `300/300/4` | size of the 3D occupancy buffer (lower it, e.g. `40/40/3`, for small robots: memory is `x*y*z/res^3` voxels) |
| `max_vel`, `max_acc` | `1.5`, `3.0` | velocity/acceleration limits used by the planner and the optimizer |
| `planning_horizon` | `10.5` | replanning horizon, ~1.5x the sensor horizon |
| `flight_type` | `1` | `1`: goal from RViz, `2`: preset waypoints from the YAML |
| `rviz` | `true` | start RViz2 |

In RViz2 select the **2D Goal Pose** tool and click a target position; the planner
then replans continuously and `traj_server` publishes velocity commands on
`/cmd_vel`. Set the goal topic with `goal_topic:=...` if you want to send goals
from your own node instead (see the *Quick Start* section below for the ROS 1
equivalent).

Other entry points:

```bash
ros2 launch patchwork ground_segmentation.launch.py pointcloud_topic:=/registered_scan
ros2 launch ego_planner rviz.launch.py
```

## What changed for ROS 2

* `catkin` -> `ament_cmake`, `package.xml` format 3, `#include <ros/ros.h>` ->
  `rclcpp`, `ros::NodeHandle`/`ros::Publisher`/`ros::Subscriber`/`ros::Timer` ->
  the `rclcpp` equivalents, `ROS_*` macros -> `RCLCPP_*`.
* Messages are generated with `rosidl`: `ego_planner/msg/Bspline.msg` and
  `ego_planner/msg/DataDisp.msg` (headers must now be written as
  `std_msgs/Header`, time fields as `builtin_interfaces/Time`). patchwork's
  `node.msg`/`ground_estimate.msg` had to be renamed to `Node.msg`/
  `GroundEstimate.msg` because ROS 2 message names must start with a capital letter.
* `tf` -> `tf2` (`tf2::Quaternion`, `tf2::Matrix3x3::getRPY`), `nav_msgs::Path`
  etc. -> `nav_msgs::msg::Path`, `pcl_conversions` is used as a header-only
  library, `message_filters` uses the ROS 2 API.
* Parameter names use dots instead of slashes (`grid_map.resolution`), the values
  live in
  [config/ego_planner.yaml](ego-planner/planner/plan_manage/config/ego_planner.yaml)
  (replacing `launch/advanced_param.xml`) and in
  [config/ground_segmentation.yaml](ego-planner/patchwork/config/ground_segmentation.yaml)
  (replacing `params_ouster128.yaml`). Numeric parameters may be written as
  `2` or `2.0` - the nodes coerce the type, so the ROS 1 style keeps working.
* Launch files are Python (`run_in_sim.launch.py`, `rviz.launch.py`,
  `ground_segmentation.launch.py`). The ROS 1 launch/param/rviz files are kept
  under `*/launch/ros1/` and `*/config/ros1/` for reference.
* The occupancy grid subscribes to point clouds with a *best effort* QoS so that
  it also works with drivers that publish best effort.
* Latent bugs that only surfaced under ROS 2 were fixed: `EGOReplanFSM::trigger_`
  and friends are now initialized (they were read uninitialized), the
  `gradient_descent_optimizer` limits no longer use a narrowing conversion, and
  all timestamps come from the same clock source (`rclcpp` throws when times of
  different clocks are subtracted).

## Not ported yet

* `autonomous_exploration_development_environment/` (the CMU vehicle simulator:
  Gazebo classic plugins, ROS 1 RViz plugins, joystick drivers) is still ROS 1 and
  is excluded from the colcon build via a `COLCON_IGNORE` file, together with
  `ego-planner/qpOASES`. Delete that file after porting the simulator if you want
  it to be built.
* Inside `patchwork`, only the ground segmentation node (`ground_segmentation`,
  from `nodes/offline_own_data.cpp`) is built. The KITTI/legoloam benchmark
  helpers (`offline_kitti.cpp`, `ros_kitti.cpp`, `pub_for_legoloam.cpp`) still use
  ROS 1 APIs and the old message names.

For a real robot you only need to provide the two inputs of the planner:
odometry (default `/state_estimation`) and a registered point cloud (default
`/registered_scan`). Everything else - ground segmentation, local map, planning,
MPC - is contained in this workspace.

# Simulation: closed loop point-to-point driving (ROS 2)

`ego_ground_sim` closes the loop *Gazebo <-> planner* without the (still ROS 1)
CMU environment: a differential drive chassis, a 3D LiDAR, odometry/TF and a
watchdog in front of the wheels.

```bash
ros2 launch ego_ground_sim sim_ego_planner.launch.py                      # Gazebo GUI + RViz2
ros2 launch ego_ground_sim sim_ego_planner.launch.py gui:=false rviz:=false  # headless
```

Then send a goal with RViz2's **2D Goal Pose** tool (`/goal_pose`). The chains are

```
goal -> ego_planner_node -> /planning/bspline -> traj_server --+
                                                               v
                                    /cmd_vel -> cmd_vel_safety -> /cmd_vel_safe -> wheels
3D LiDAR -> /scan_3d -> point_cloud_transformer -> /registered_scan -> patchwork
                                          -> /benchmark/{P,N} -> local map -> planner
odometry (diff drive plugin) -> /state_estimation -> planner + MPC + watchdog
```

Details of the simulation package are in
[ego_ground_sim/README.md](ego_ground_sim/README.md). Two points are worth
knowing up front:

* The mapper treats *every* measured point as occupied, so feeding the raw scan
  into it would put the ground - and therefore the robot - inside an obstacle and
  every plan would fail. The launch therefore feeds `/grid_map/cloud` with the
  ground-segmented cloud (`mapping_cloud_topic:=/benchmark/N`, the default).
  Use `mapping_cloud_topic:=/registered_scan` only if your scanner does not see
  the ground.
* All nodes run with `use_sim_time:=true` and their timers follow the simulated
  clock. A stand-alone `rclcpp::Clock` does **not** subscribe to `/clock`; mixing
  such a clock with the node clock made the planner's collision check loop over
  ~1e9 seconds of "time" (a real bug that is fixed in this workspace, see
  `EGOPlannerManager::clock_` and the guard in `EGOReplanFSM::checkCollisionCallback`).

## Verified behaviour (headless run of 2026-10-11)

```
ros2 launch ego_ground_sim sim_ego_planner.launch.py gui:=false rviz:=false
python3 install/ego_ground_sim/share/ego_ground_sim/scripts/closed_loop_check.py 6.0 4.5
```

| check | result |
| --- | --- |
| robot really moves (by physics, from `/cmd_vel_safe`) | travelled 22.41 m instead of 17.20 m straight -> it drove a detour |
| planner receives its inputs | 778 non-ground points, 402 `/map_2d` updates, odometry at 50 Hz |
| goal produces trajectories | 21 `/planning/bspline` messages, 34 plans, 0 failures |
| MPC drives the robot | max speed 1.13 m/s (`MPC/v_max` = 1.2) |
| obstacle avoidance | min clearance +0.94 / +1.64 / +0.67 / +1.49 / +0.90 m to the five obstacles, no footprint hit |
| arrival and stop | goal error 0.61 m, `/cmd_vel_safe` = 0, position drift 0.000 m afterwards |
| data timeout stop | command source stopped -> `/cmd_vel_safe` = 0 after 0.5 s, drift 0.000 m |
| invalid command | NaN velocity -> `/cmd_vel_safe` = 0 |

Note: when a new goal arrives, the FSM first turns the robot towards the initial
direction of the planned trajectory (`ADJUST_POSE`). That behaviour comes from the
original ROS 1 code; depending on the initial polynomial it can be a large turn.

## Simulation on the CMU indoor scene

The scene that the original README shows is part of this repository
(`autonomous_exploration_development_environment/src/vehicle_simulator/mesh/indoor`,
mesh + textures + preview point cloud, ~129 x 99 x 3.9 m). `ego_ground_sim`
installs it as a Gazebo model, so it can be used from ROS 2 directly:

```bash
ros2 launch ego_ground_sim sim_cmu_indoor.launch.py
```

The default spawn is near the red robot in `imag/ego_example.gif`, in the passage
above the rectangular enclosure: map coordinate `(-31.3, -23.4)`, equivalent to
CMU original coordinate `(31.5, -3.0)`. The robot faces -Y (`spawn_yaw=-pi/2`),
and the Gazebo/RViz cameras look at this area. Floor support was checked against
the collision mesh; navigation from this spawn has not yet been tested.
Override it with `spawn_x:=... spawn_y:=... spawn_yaw:=...` if needed.

The historical indoor measurements below used spawn `(-62.8, -20.4)`.

Verified: a 13 m goal along the corridor is reached with 0.42 m final error and no
emergency stops (robot drives 0.9 m/s, real time factor ~0.95 despite the 20 MB
mesh). Goals further away are reached as long as they stay in one corridor: EGO
plans on a local 8 m map and treats unknown space as free, so a goal in another
room can end in a local minimum - use waypoints (or add a global planner) for
multi-room missions. Details, parameters and the goal-picking helper are in
[ego_ground_sim/README.md](ego_ground_sim/README.md).

Two things that were required to make this scene work, and that also matter on a
real robot:

* patchwork must segment in the *sensor* frame - its concentric zone model is
  centred on the cloud origin. The node now transforms its input into
  `sensor_frame` and its outputs into `output_frame` (`patchwork.sensor_frame`,
  `patchwork.output_frame`; both empty by default = previous behaviour).
* `optimization.relaxed_initial_collision` (default `false`): when the robot is
  closer to a wall than `obstacles_inflation`, the upstream code rejects every plan
  and the robot can only emergency stop. With the flag the first trajectory segment
  is tolerated while the rest of the trajectory and the FSM safety check stay
  unchanged.

# Environment Configuration (ROS 1, legacy)

The sections below describe the original ROS 1 setup and are kept for reference.
For ROS 2 Humble see the *ROS 2 (Humble) port* section above.

## qpOASES

We use qpOASES to solve MPC，and MPC is used for trajectory tracking

qpOASES  solver installation :

```bash
cd qpOASES
mkdir build
cd build
cmake ..
sudo make
sudo make install
```

## patchwork

patchwork is used for ground segmentation. This planner accepts 3d point cloud information **Pointcloud2**. After the point cloud is segmented on the ground, the point cloud is projected to the 2d plane to construct a 2d grid map for planning.

Follow the official repository to download this package：[patchworl_github](https://github.com/LimHyungTae/patchwork)

## Simulation environment

The simulation environment uses the open source simulation environment of the CMU Robotics Institute. Users  need to install the relevant libraries and download the mesh file. We recommend to use the indoor environment to test the planner.

The environment have two versions for ros-melodic and ros-noetic. The package in this project is for ros-melodic.

[CMU-environment](https://www.cmu-exploration.com/)

# Quick Start (ROS 1, legacy)

First , start the simulation environment :

```bash
roslaunch vehicle_simulator system_indoor.launch
```

Second, start the planner :

```bash
roslaunch ego_planner run_in_sim.launch
```

In rviz , use  **2d nav goal** to set the goal :

![ego example](imag/ego_example.gif)

If you can't see the imag in this file, you can see it in the folder "imag".



# Explanation of Parameter Adjustments

In the simulation environment provided by CMU, the car does not have a real physical engine. So the car can move strictly according to control commands (linear velocity v and angular velocity w), which will affect the parameter setting of the MPC controller.

The objective function of the MPC controller consists of two parts: tracking error and motion constraints. Users can adjust the following 4 parameters:

```xml
<!-- adjust in advanced_param.xml -->
<param name="MPC/v_max" value="1.8" type="double"/>
<param name="MPC/w_max" value="1.2" type="double"/>
<param name="MPC/omega0" value="1.0" type="double"/>
<param name="MPC/omega1" value="0.5" type="double"/>
```

`MPC/v_max` and `MPC/w_max` are related to the maximum speed and acceleration set during planning and can be set slightly larger than the maximum planning speed `max_vel` .

`MPC/omega0` and `MPC/omega1` are the weights for the tracking error and motion constraints mentioned above respectively:

1. A larger value of `omega0` makes the MPC solver tend to reduce the error more, but it may exceed the robot's kinematic limits.
2. A larger value of `omega1` makes the MPC solver tend to make the robot's motion smoother, but it increases the error between the reference trajectory.

In the simulation environment provided by CMU, it is suggested to increase `omega0` because the motion of the car in this simulation environment is not restricted.

However, in a simulation environment with a physics engine, it is recommended to increase `omega1` appropriately. Otherwise, it is easy to encounter phenomena where the angular velocity and linear velocity are too large, resulting in unsmooth motion of the robot.

Plan to provide a version of ego-planner with integrated SLAM and a simulation environment with a physics engine in a [branch](https://github.com/Dangko/ego-planner-for-ground-robot/tree/dev-work-with-slam_module). 
It will be upload  once I have time to organize it.



# Update Logs

- 2023.10.31 

  Fixed the bug of the car switching its front end.

  Fixed the bug in MPC parameter adjustment and added parameter interface and documentation.

  Added an indoor scene of the CMU simulation environment, allowing users to test directly without downloading.



# THANKS

We are very grateful for Fei Gao and the EGO-Planner he proposed. Our algorithm is implemented under their algorithm framework. If you are interested in our algorithm, you may see the origin algorithm in :

[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)

