# ego_ground_ws：V550 地面 EGO-Planner

直接在 VS Code 中打开这个目录。EGO、Gazebo 仿真、V550 实车接口及所需的厂家包都在 src 内。

- [未知区域实车导航：统一启动与 RViz 目标](src/ego-planner-for-ground-robot/V550未知区域实车导航.md)
- [单工作空间：依赖安装、编译与启动](src/ego-planner-for-ground-robot/V550单工作空间.md)
- [完整实车部署流程](src/ego-planner-for-ground-robot/V550实车部署.md)
- [厂家源码迁入清单](src/ego-planner-for-ground-robot/wheeltec_vendor/README.md)

不再需要 source 外面的 Wheeltec 工作空间。先按单工作空间文档安装 ROS 系统依赖，再编译。
VS Code 的“终端 → 运行任务”提供实车和仿真构建入口。
