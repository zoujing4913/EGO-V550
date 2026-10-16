# 当前工程内的 Wheeltec 驱动

从用户提供的 `JP6.2_wheeltec_ros2_src_20260903` 源码复制，原目录没有修改或删除。
本目录随 EGO 工程一起保存、复制和编译，不使用指向外部源码的软链接。
`UPSTREAM_MANIFEST.json` 记录每个文件的厂家相对路径和迁入前 SHA-256。
厂家版权、包名和许可证声明保留；本地适配见下文。

## 迁入范围

| ROS 包 | 内容 |
| --- | --- |
| `turn_on_wheeltec_robot` | 底盘串口协议、编码器/IMU、EKF 启动、车型 TF、雷达启动、厂家配置和原 udev 脚本 |
| `wheeltec_robot_msg` | 底盘使用的自定义消息 |
| `serial` | 厂家使用的 ROS 2 C++ 串口库源码 |
| `lslidar_driver` | N10p 对应 `N10Plus` 型号所在的镭神驱动及其共享代码、配置和说明 |
| `lslidar_msgs` | 雷达消息与服务 |
| `wheeltec_slam_toolbox` | 厂家在线 SLAM 配置、启动和示例源码 |
| `wheeltec_robot_urdf` | V550_mec、V550_4wd、V550_akm 的原厂 URDF 和全部引用的网格资源 |

当前硬件配置是 **V550_mec + N10p USB/串口 + 底盘内置 stm32 IMU**。
V550 以外的车型网格、外置 H30/N300pro IMU 驱动、超声波、相机、其他品牌雷达和 Nav2 源码不在当前运行链路中，没有整体复制。
厂家通用 launch 中仍保留这些可选分支；切换硬件时需要另外提供对应驱动。
V550_akm 模型仅保留用于显示，当前 EGO 入口仍禁止阿克曼底盘执行此控制模型。

## 本地适配

- `wheeltec_param.yaml` 默认车型改为 `V550_mec`，雷达改为 `ls_N10Plus_uart`；其余厂家参数保留，实际外参和屏蔽角度需上车核对。
- 底盘启动支持 `controller_port:=...`，雷达启动支持 `lidar_port:=...`。
- 补齐运行和构建依赖声明，`ego_wheeltec` 的构建依赖闭包包含这七个包。
- 雷达 CMake 显式查找 libpcap 的头文件和库，避免缺开发包时在链接阶段才报错。
- 移除车型描述 launch 未使用的 Nav2 Python 导入，EKF 调试输出改为 `/tmp/wheeltec_ekf_debug.txt`（默认不启用调试）。
- 修复 IMU 平方根倒数函数的 64 位适配：用 `uint32_t` 和 `memcpy` 保留原近似计算，避免 `long` 指针越界读取 `float`。适用于 amd64/arm64。
- 未改变底盘串口收发协议、编码器算法和雷达解析算法。

默认设备仍是 `/dev/wheeltec_controller` 和 `/dev/wheeltec_lidar`。
原厂 `wheeltec_udev.sh` 仅作为资料保留；它会写系统规则并包含其他设备设置，不会由构建或启动自动执行。
实际设备名、串口访问权限应在车上确认，也可以直接通过启动参数指定真实设备路径。

构建、运行和迁移检查见 [单工作空间使用说明](../V550单工作空间.md)。

未知环境中用 RViz 给目标的统一入口见 [未知区域实车导航](../V550未知区域实车导航.md)。
