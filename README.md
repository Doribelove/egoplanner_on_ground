# ego_on_groud

面向地面无人车的 EGO-Planner 改造工程。项目保留 EGO-Planner 的局部重规划、栅格地图、B 样条轨迹优化能力，并将原本面向无人机的三维轨迹执行链路改造成适合地面车的二维规划与 `/cmd_vel` 控制输出。

当前代码主要适配 Autolabor M2 一类阿克曼/类阿克曼底盘：规划器只负责生成局部 B 样条轨迹，`traj_server` 使用 MPC 跟踪轨迹并发布速度指令。

## 功能概览

- 基于 EGO-Planner 的局部避障与实时重规划。
- 使用二维膨胀栅格进行地面车路径搜索和碰撞检查。
- 固定轨迹高度，避免无人机版本的三维爬升行为影响地面车。
- 支持 RViz `2D Nav Goal` 下发目标点。
- 支持目标在车后方时切换倒车逻辑，并通过 `/direction` 同步给轨迹跟踪器。
- `traj_server` 使用 qpOASES 求解 MPC，输出 `geometry_msgs/Twist` 到 `/cmd_vel`。
- 支持两种实车点云预处理链路：
  - Patchwork 地面分割：过滤地面，使用非地面点 `/benchmark/N` 建图。
  - 虚拟高墙投影：将障碍点沿 Z 轴拉伸成墙，防止规划器从障碍物上方“飞越”。

## 目录结构

```text
ego_on_groud/
├── src/
│   ├── planner/
│   │   ├── plan_manage/       # ego_planner 主包、FSM、launch、轨迹服务器
│   │   ├── plan_env/          # 栅格地图、点云/深度融合、占据栅格
│   │   ├── path_searching/    # 动态 A* 搜索
│   │   ├── bspline_opt/       # B 样条优化
│   │   ├── traj_utils/        # 轨迹与可视化工具
│   │   └── mpc_controller/    # 地面车 MPC 跟踪控制器
│   ├── patchwork/             # Patchwork 地面分割
│   ├── qpOASES/               # QP 求解器
│   ├── model/                 # 车体模型
│   └── scripts/               # 调试脚本副本
├── .gitignore
└── README.md
```

`build/`、`devel/`、`logs/`、rosbag、点云采样数据和运行日志均为本地生成或大体积数据，已通过 `.gitignore` 排除，不建议直接提交到 GitHub。

## 核心数据流

```text
里程计 /Odometry 或 /state_estimation
        │
        ├── ego_planner_node
        │       ├── 订阅目标 /move_base_simple/goal
        │       ├── 订阅点云地图输入 /grid_map/cloud 或 /benchmark/N
        │       └── 发布 B 样条 /planning/bspline
        │
        └── traj_server
                ├── 订阅 /planning/bspline
                ├── 订阅 /direction 和 /emergency_stop
                └── 发布 /cmd_vel
```

点云输入有两条常用链路：

```text
Patchwork 链路:
/cloud_registered 或 /cloud_body
    -> patchwork/offline_own_data
    -> /benchmark/N
    -> ego_planner_node

虚拟高墙链路:
/cloud_registered
    -> cloud_flattener_node
    -> /planning/flat_cloud
    -> ego_planner_node
```

## 主要 ROS 节点

### `ego_planner_node`

位置：`src/planner/plan_manage/src/ego_planner_node.cpp`

主规划节点，内部启动 `EGOReplanFSM`。它负责接收目标点、读取里程计和地图、调用全局/局部规划，并发布局部 B 样条轨迹。

常用输入：

- `/odom_map`：里程计，launch 中通常 remap 到 `/Odometry` 或 `/state_estimation`。
- `/way_point`：目标点，launch 中通常 remap 到 `/move_base_simple/goal`。
- `/grid_map/cloud`：点云输入。
- `/grid_map/odom`：建图使用的里程计。

常用输出：

- `/planning/bspline`：局部 B 样条轨迹。
- `/planning/data_display`：调试显示数据。
- `/direction`：行驶方向，`0` 表示前进，`1` 表示倒车。
- `/emergency_stop`：急停标志。
- `/odom_adjust`：倒车逻辑调整后的调试里程计。

### `traj_server`

位置：`src/planner/plan_manage/src/traj_server.cpp`

轨迹跟踪节点。它订阅 `/planning/bspline`，使用 MPC 跟踪 B 样条参考轨迹，并发布 `/cmd_vel`。该节点包含针对阿克曼车的后处理：

- 限制最小转弯半径。
- 低速死区补偿。
- 到达终点后的低速伺服停车。
- 倒车时反向处理线速度。

当前源码中车辆参数：

```cpp
#define CAR_MIN_TURN_RADIUS 1.2
#define CAR_WHEELBASE 0.65
```

### `cloud_flattener_node`

位置：`src/planner/plan_manage/src/cloud_flattener.cpp`

将指定高度范围内的障碍点沿 Z 轴拉伸成虚拟墙，解决地面车场景中规划器试图从障碍物上方通过的问题。

常用参数：

- `cloud_flattener/input_topic`：输入点云，默认 `/livox/lidar`。
- `cloud_flattener/output_topic`：输出点云，默认 `/planning/flat_cloud`。
- `cloud_flattener/min_height`：障碍点最小高度。
- `cloud_flattener/max_height`：障碍点最大高度。
- `cloud_flattener/wall_height`：虚拟墙高度。
- `cloud_flattener/resolution`：虚拟墙垂直采样间隔。

### `patchwork/offline_own_data`

位置：`src/patchwork/nodes/offline_own_data.cpp`

订阅 `/pointcloud_scan`，输出：

- `/benchmark/cloud`：原始点云。
- `/benchmark/P`：地面点。
- `/benchmark/N`：非地面点，规划链路通常使用该话题作为障碍物输入。

参数文件：`src/patchwork/config/params_ouster128.yaml`。

### `transform_cloud.py`

位置：`src/planner/plan_manage/scripts/transform_cloud.py`

使用 TF2 将点云真正变换到目标坐标系，而不是只改 `frame_id`。`run_in_real.launch` 中用于将 `/cloud_registered` 转换为 `/cloud_body`。

## 环境依赖

推荐环境：

- Ubuntu 20.04
- ROS Noetic
- catkin_tools
- Eigen3
- PCL
- OpenCV / cv_bridge
- TF / TF2
- qpOASES，本项目已包含源码目录 `src/qpOASES`

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-cv-bridge \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-sensor-msgs \
  libeigen3-dev \
  libpcl-dev
```

如果使用 `gps_simple.py`，还需要：

```bash
pip3 install pyserial
```

## 编译

在工作空间根目录执行：

```bash
cd ~/ego_on_groud
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

也可以只编译主要包：

```bash
catkin build qpOASES plan_env path_searching bspline_opt traj_utils mpc_controller ego_planner patchwork
```

## 运行

### 1. 实车运行：Patchwork 地面分割链路

适用于点云中地面较多、需要先分离非地面障碍物的场景。

```bash
source /opt/ros/noetic/setup.bash
source ~/ego_on_groud/devel/setup.bash
roslaunch ego_planner run_in_real.launch
```

默认话题：

- 里程计：`/Odometry`
- 原始点云：`/cloud_registered`
- 转换后点云：`/cloud_body`
- Patchwork 非地面点：`/benchmark/N`
- RViz 目标点：`/move_base_simple/goal`
- 控制输出：`/cmd_vel`

### 2. 实车运行：虚拟高墙链路

适用于希望直接把障碍点转换成二维可避障墙体的场景。

```bash
source /opt/ros/noetic/setup.bash
source ~/ego_on_groud/devel/setup.bash
roslaunch ego_planner run_in_real2.launch
```

默认话题：

- 里程计：`/Odometry`
- 原始点云：`/cloud_registered`
- 虚拟高墙点云：`/planning/flat_cloud`
- RViz 目标点：`/move_base_simple/goal`
- 控制输出：`/cmd_vel`

### 3. 仿真/调试运行

```bash
source /opt/ros/noetic/setup.bash
source ~/ego_on_groud/devel/setup.bash
roslaunch ego_planner run_in_sim.launch
```

默认使用：

- 里程计：`/state_estimation`
- 点云：`/registered_scan`
- 目标：`/move_base_simple/goal`

## RViz 下发目标

启动 launch 后，在 RViz 中使用 `2D Nav Goal` 发布目标点。目标点会被 remap 到 `/move_base_simple/goal`，规划器收到后会：

1. 根据当前车体朝向判断目标在前方还是后方。
2. 必要时切换 `/direction`。
3. 生成全局参考轨迹。
4. 生成局部 B 样条轨迹并持续重规划。
5. 由 `traj_server` 输出 `/cmd_vel`。

## 关键参数

主要参数集中在 `src/planner/plan_manage/launch/advanced_param.xml`。

### 地图参数

- `grid_map/resolution`：栅格分辨率，当前为 `0.1 m`。
- `grid_map/map_size_x`、`map_size_y`、`map_size_z`：地图尺寸。
- `grid_map/local_update_range_x/y/z`：局部更新范围。
- `grid_map/obstacles_inflation`：障碍物膨胀半径。
- `grid_map/ground_height`：地图原点高度。
- `grid_map/frame_id`：地图坐标系，当前常用 `camera_init`。
- `grid_map/map_type`：当前 launch 使用 `1`，对应规划用途。

### 规划参数

- `manager/max_vel`：规划最大速度。
- `manager/max_acc`：规划最大加速度。
- `manager/max_jerk`：最大 jerk。
- `manager/control_points_distance`：B 样条控制点间距。
- `manager/planning_horizon`：局部规划前视距离。
- `optimization/lambda_smooth`：平滑项权重。
- `optimization/lambda_collision`：碰撞项权重。
- `optimization/lambda_feasibility`：动力学可行性权重。
- `optimization/lambda_fitness`：轨迹贴合权重。

### MPC 参数

- `MPC/v_max`：最大线速度。
- `MPC/w_max`：最大角速度。
- `MPC/omega0`：状态误差权重。
- `MPC/omega1`：控制输入权重。

注意：`traj_server.cpp` 中预测步长 `t_step` 和定时器周期均为 `0.03 s`，预测步数为 `N=15`。

## 坐标系约定

现有 launch 中常见静态 TF：

- `camera_init -> map`
- `body -> base_link`

`run_in_real.launch` 使用：

```xml
body -> base_link: 0.1 0 0 0 0 0
camera_init -> map: 0 0 0 0 0 0
```

`run_in_real2.launch` 使用：

```xml
body -> base_link: -0.1 0 -1.4 0 0 0
camera_init -> map: 0 0 0 0 0 0
```

实车部署前需要确认：

- 里程计 `header.frame_id` 与 `grid_map/frame_id` 一致或存在 TF。
- 点云坐标系能转换到规划使用的坐标系。
- `/cmd_vel` 的正方向与底盘驱动一致。
- 如果使用倒车，底盘驱动必须支持负线速度。

## GPS 辅助目标输入

脚本：`src/planner/plan_manage/scripts/gps_simple.py`

该脚本从串口读取 NMEA GGA 数据，将第一次读取的位置作为局部原点，然后把输入的目标经纬度转换成局部 XY，并发布到 `/move_base_simple/goal`。

默认串口：

```python
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200
```

运行：

```bash
source ~/ego_on_groud/devel/setup.bash
python3 src/planner/plan_manage/scripts/gps_simple.py
```

## 调试建议

检查关键话题：

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic hz /planning/bspline
rostopic echo /direction
rostopic echo /cmd_vel
```

检查 TF：

```bash
rosrun tf view_frames
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body base_link
```

检查规划地图：

```bash
rostopic hz /grid_map/occupancy_inflate
rostopic hz /map_2d
```

## 已知问题和部署注意事项

1. 两个实车启动文件将规划节点和轨迹跟踪节点的里程计输入都接到 `/Odometry`。仿真入口 `run_in_sim.launch` 使用 `/state_estimation`。

2. `single_run_in_exp.launch` 引用了不存在的 `advanced_param_real.xml`，当前不建议作为启动入口。可参考 `run_in_real.launch` 或 `run_in_real2.launch` 修改。

3. `run_in_real.launch` 中 `ego_planner_node` 的 `/cmd_vel` 被 remap 到 `/junk_cmd_vel_ego`，这是为了防止 FSM 直接控制底盘。真实速度输出来自 `traj_server` 的 `/cmd_vel`。

4. 地面车不应使用无人机式原地转向逻辑。当前 FSM 中 `ADJUST_POSE` 状态被跳过，起步速度方向会根据车头方向强制设置，以满足阿克曼约束。

5. 如果规划器频繁试图穿过低矮障碍物，优先使用 `run_in_real2.launch` 的虚拟高墙链路，或调大 `cloud_flattener/wall_height` 和 `grid_map/obstacles_inflation`。

6. 如果点云中车体自身被识别为障碍，调大 Patchwork 的 `min_r` 或检查雷达到车体坐标系外参。

## 常见故障

### RViz 发目标后没有规划

检查：

- `/Odometry` 或 `/state_estimation` 是否有数据。
- `/move_base_simple/goal` 是否有数据。
- `ego_planner_node` 是否打印 `no odom.` 或 `wait for goal.`。
- 目标点坐标系是否与 `camera_init`/地图坐标系一致。

### 有轨迹但车不动

检查：

- `/planning/bspline` 是否发布。
- `/cmd_vel` 是否发布。
- 运行 `rosnode info /traj_server`，确认订阅的里程计话题有数据。
- 底盘驱动是否订阅 `/cmd_vel`。

### 车体绕障过近

检查并调大：

- `grid_map/obstacles_inflation`
- `optimization/dist0`
- `cloud_flattener/wall_height`
- `cloud_flattener/resolution`

### 轨迹切弯或震荡

检查：

- `manager/max_vel`
- `manager/max_acc`
- `MPC/w_max`
- `MPC/omega0`
- `MPC/omega1`
- `CAR_MIN_TURN_RADIUS`

## 后续建议

- 把车辆轴距、最小转弯半径、MPC 预测步长从宏定义改为 ROS 参数。
- 增加一个统一的 `bringup.launch`，明确选择 `patchwork` 或 `cloud_flattener` 点云链路。
- 删除或修复失效 launch，例如 `single_run_in_exp.launch`。
- 为实车安全增加速度上限、急停输入和遥控接管优先级。
