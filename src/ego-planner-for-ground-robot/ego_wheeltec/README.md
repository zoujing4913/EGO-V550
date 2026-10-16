# V550 unknown-area navigation

Use `ros2 launch ego_wheeltec v550_unknown_nav.launch.py` for online SLAM and RViz goals in unknown areas.
See [未知区域实车导航](../V550未知区域实车导航.md) for dependencies, commissioning, enabling and limitations.

# ego_wheeltec

Wheeltec V550/N10p dependencies are bundled under `../wheeltec_vendor`.
See [单工作空间流程](../V550单工作空间.md); no external vendor workspace is needed.
Use `start_base:=true start_lidar:=true` to include the bundled hardware drivers.
The default remains observation only; actual serial devices are required for hardware drivers.

V550 real robot integration for the ROS 2 ground EGO planner.
See [V550 实车部署流程](../V550实车部署.md) for setup, commissioning, model limitations and commands.

Build: `colcon build --packages-up-to ego_wheeltec`.

```bash
ros2 launch ego_wheeltec v550_mec_2d.launch.py
ros2 run ego_wheeltec preflight.py
```

This default is observation only: the guard outputs to `/ego/cmd_vel_checked`.
Real chassis output additionally requires `drive:=true`, the `/ego/set_enabled`
service and a new goal after enabling. `V550_akm` cannot drive with this MPC/FSM.
The confirmed profile is V550_mec with a single-plane 2D LiDAR: Patchwork is not
started, and the existing MPC uses forward/reverse speed and yaw rate, with
`linear.y=0`. Full lateral mecanum control would require a different MPC model.

| File | Purpose |
| --- | --- |
| `launch/v550_mec_2d.launch.py` | Dedicated entry point for the confirmed V550_mec + 2D LiDAR configuration |
| `launch/v550_ego.launch.py` | Real time launch, optional factory drivers, input selection and remapping |
| `config/ego_v550.yaml` | Conservative planner/MPC commissioning parameters |
| `config/ground_segmentation.yaml` | 3D segmentation template; measured height/frame required |
| `src/sensor_bridge.cpp` | Time-correct TF for obstacles and target poses |
| `src/cmd_vel_guard.cpp` | Disabled startup, fresh input checks, limits and latched stop |
| `scripts/preflight.py` | Read-only sensor/odometry/TF inspection |
| `tests/test_interfaces.py` | Isolated synthetic interface regression tests |
