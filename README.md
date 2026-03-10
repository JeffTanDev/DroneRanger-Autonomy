# DroneRanger-Autonomy

> 无人机自主飞行与避障算法（ROS2 + PX4），面向 Unity 3D 仿真场景。

## 项目简介

`DroneRanger-Autonomy` 是 Drone-Ranger 项目中的**算法子工程**，基于 **ROS2 + PX4 Offboard 模式**，实现：

- **路径规划**：栅格 A\*（含多种变体）、简化 RRT-like 追踪等。
- **深度避障**：使用来自 Unity 模拟相机的深度图，构建占据栅格或做前方障碍检测。
- **自主任务控制**：起飞、悬停、追踪目标点、绕障飞行等。
- **可视化支持**：向 Unity 发布路径/状态数据，配合 Unity 场景做 3D 可视化。

配套的 Unity 工程在上层仓库 `Drone-Ranger` 的 `Assets` 目录中，通过 ROS-TCP 与本项目交互。

---

## 文件作用速览（按包划分）

> 下面只列出本 ROS2 工程中的 Python 源码文件，说明它们的大致作用和主要包含的类/函数（如有）。

### `src/drone_autonomy/`

- **`src/drone_autonomy/setup.py`**
  - 作用：定义 `drone_autonomy` ROS2 Python 包的安装方式、入口点等。
  - 主要内容：`setup()` 调用，注册包名、依赖、入口脚本（例如各个 mission 可作为可执行节点）。

- **`src/drone_autonomy/test_pursuit.py`**
  - 作用：用于本地/仿真环境下测试追踪类算法（如 A\* 或 RRT-like 追踪）的简单脚本。
  - 主要内容：若干测试函数和 `main()`，调用对应 mission 节点或算法函数，验证追踪行为是否符合预期。

- **`src/drone_autonomy/launch/simple_takeoff.launch.py`**
  - 作用：ROS2 启动文件，用于启动一个“简单起飞”任务节点。
  - 主要内容：定义 `LaunchDescription`，创建/配置对应的 Python 节点（如起飞控制节点）的启动参数。

- **`src/drone_autonomy/drone_autonomy/__init__.py`**
  - 作用：包初始化文件，使 `drone_autonomy` 作为 Python 包可被导入。
  - 主要内容：通常为空或只导出子模块，方便 `from drone_autonomy import ...` 使用。

- **`src/drone_autonomy/drone_autonomy/missions/__init__.py`**
  - 作用：`missions` 子包初始化，使各个任务脚本可以通过包方式导入。
  - 主要内容：可能导出常用的任务类或帮助函数，方便外部引用。

#### `missions` 目录下的任务脚本

- **`src/drone_autonomy/drone_autonomy/missions/astar_grid_pursuit.py`**
  - 作用：核心 2D 栅格 A\* 路径规划 + 追踪任务节点。
  - 主要类/函数：
    - 类 `AStarGridPursuit`：ROS2 节点类，内部包含状态机、占据栅格构建、A\* 搜索和 PX4 控制逻辑。
    - 函数 `astar_2d(...)`：在 2D 栅格上运行 A\* 的搜索函数。
    - 若干坐标转换/网格更新函数（如 GPS ↔ NED、world_grid 维护等）。

- **`src/drone_autonomy/drone_autonomy/missions/rrt_pursuit_single.py`**
  - 作用：基于 RRT-like 直线步进策略的单目标追踪 + 深度避障任务节点。
  - 主要类/函数：
    - 类 `RRTPursuitFixed`：ROS2 节点类，实现追踪状态机、每步向目标推进的策略以及前向深度避障逻辑。
    - 辅助函数：处理深度图 ROI、前方是否被挡的判定、简单避障状态机更新等。

- **`src/drone_autonomy/drone_autonomy/missions/astar_world_buffer.py`**
  - 作用：带有“世界地图缓存（world buffer）”的 A\* 栅格规划任务变体，更多关注全局占据图的累积与更新。
  - 主要内容：实现与 `AStarGridPursuit` 类似的栅格构建和 A\* 搜索，但在 world grid 的记忆和复用策略上做了差异化设计。

- **`src/drone_autonomy/drone_autonomy/missions/astar_shortern_memory.py`**
  - 作用：在记忆/缓存占据地图的同时，加入“缩减/精简记忆”的 A\* 变体，控制地图规模和长期记忆。
  - 主要内容：与 `astar_world_buffer.py` 类似，包含 A\* 搜索和 world grid 维护的函数，侧重于如何裁剪历史障碍信息。

- **`src/drone_autonomy/drone_autonomy/missions/astar_no_memory_conservative.py`**
  - 作用：不保留长期 world memory、但在局部规划上采取保守策略的 A\* 任务脚本。
  - 主要内容：实现仅用局部占据栅格进行 A\* 规划的逻辑，规避“错误记忆”带来的风险。

- **`src/drone_autonomy/drone_autonomy/missions/astar_grid_pursuit_3D.py`**
  - 作用：将栅格 A\* 扩展到 3D（包含高度维度）的追踪任务。
  - 主要内容：在 3D 体素网格上运行 A\* 搜索的函数和相关栅格构建逻辑，适用于更复杂的三维障碍环境。

#### `haoran_alg_update` 目录（Unity 避障 & 追踪模板）

- **`src/drone_autonomy/drone_autonomy/haoran_alg_update/__init__.py`**
  - 作用：子包初始化，导出 `haoran_unity_pursuit` 等模块供 `setup.py` 入口点使用。

- **`src/drone_autonomy/drone_autonomy/haoran_alg_update/haoran_unity_pursuit.py`**
  - 作用：面向 Unity 仿真的**水平 VFH+ 避障 + 目标追踪**任务节点（单文件版）。
  - 主要类：`HaoranUnityPursuit`，状态机 WARMUP → ARMING → TAKEOFF → HOVER_STABLE → EXECUTE。
  - 核心逻辑：深度图水平扇区、VFH+ 选安全航向、速度门控与 waypoint 步长；无深度/无扇区时可退化为直线追目标或 yaw 扫描。
  - 话题：订阅 `/drone/gps`、`/static_cam/target_gps`、`/oak/depth/*`、`/fmu/out/vehicle_local_position_v1`；发布 `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`、`/fmu/in/vehicle_command`。

- **`src/drone_autonomy/drone_autonomy/haoran_alg_update/haoran_unity_pursuit02.py`**
  - 作用：在 `haoran_unity_pursuit` 基础上的**增强版**：前方全角度（2D 扇区）、上下避障、大障碍物防护。
  - 主要类：同上 `HaoranUnityPursuit`（节点名相同，入口脚本不同）。
  - 核心逻辑：
    - **2D 扇区**：深度图按水平×垂直划分网格，VFH 在水平+俯仰方向选最优方向，支持斜上/斜下飞行。
    - **上下避障**：根据图像上/下条带最近距离，上方近→向下避让，下方近→向上避让，微调高度 setpoint。
    - **大障碍物**：前方全局最近距离低于阈值时视为“墙”，停止前进并每周期指令爬升，避免撞大物体。
    - **扇区膨胀**：近障扇区向邻格膨胀，避免从大物体边缘钻过。
  - 话题：与 `haoran_unity_pursuit` 一致。

- **`src/drone_autonomy/drone_autonomy/haoran_alg_update/march_10_report.txt`**
  - 作用：算法进展与测试记录文档（纯文本）。

---

### `src/offboard_avoidance_unity/`

- **`src/offboard_avoidance_unity/setup.py`**
  - 作用：定义 `offboard_avoidance_unity` ROS2 Python 包的安装方式。
  - 主要内容：`setup()` 调用，注册包和入口点。

- **`src/offboard_avoidance_unity/offboard_avoidance_unity/__init__.py`**
  - 作用：子包初始化，使 `offboard_avoidance_unity` 可作为 Python 包导入。
  - 主要内容：通常为空或只暴露 `UnityHouseAvoidance` 类等公共接口。

- **`src/offboard_avoidance_unity/offboard_avoidance_unity/unity_house_avoidance.py`**
  - 作用：专为 Unity 房屋场景设计的深度避障任务节点。
  - 主要类/函数：
    - 类 `UnityHouseAvoidance`：ROS2 节点类，定义预设 ENU 航线，订阅 `/oak/depth/image_rect_raw` 和 `/drone/odom`，在飞行过程中根据深度信息添加侧向偏移实现绕障。
    - 若干辅助函数：计算前向/右侧向量、从深度图中提取 ROI 检测前方障碍、更新 `avoid_offset` 以及发送 PX4 Offboard 轨迹指令等。

---

### `src/offboard_test/`

- **`src/offboard_test/setup.py`**
  - 作用：定义 `offboard_test` ROS2 Python 包的安装方式。
  - 主要内容：`setup()` 调用，注册一系列测试节点作为可执行入口。

- **`src/offboard_test/offboard_test/offboard_takeoff.py`**
  - 作用：仅执行“起飞到指定高度”的 Offboard 模式测试任务。
  - 主要内容：一个 ROS2 节点类或 `main()` 函数，订阅 PX4 状态并向 `/fmu/in/*` 话题发布起飞相关 setpoint 和模式/解锁命令。

- **`src/offboard_test/offboard_test/offboard_square_mission.py`**
  - 作用：在平面上执行“方形航线”飞行的 Offboard 测试任务。
  - 主要内容：在 NED 平面上按顺序生成四个角点的 setpoint，控制无人机沿方形轨迹飞行。

- **`src/offboard_test/offboard_test/obstacle_publisher.py`**
  - 作用：在仿真环境中发布固定/动态障碍物信息的节点，供其他避障任务订阅。
  - 主要内容：发布障碍物位置/形状相关的话题（例如自定义消息或标准消息），为避障任务提供环境数据。

- **`src/offboard_test/offboard_test/obstacle_avoidance_mission.py`**
  - 作用：基于已知障碍物信息的避障任务示例。
  - 主要内容：订阅 `obstacle_publisher` 的话题和 PX4 状态，规划绕开障碍物的飞行路线。

- **`src/offboard_test/offboard_test/obstacle_avoidance_dynamic.py`**
  - 作用：用于测试**动态障碍物**场景下的避障算法。
  - 主要内容：处理障碍物随时间变化的情况，更新规划或控制策略应对移动障碍物。

- **`src/offboard_test/launch/obstacle_mission.launch.py`**
  - 作用：启动一组与障碍物任务相关的节点（如 `obstacle_publisher` + 对应避障任务）。
  - 主要内容：在 `LaunchDescription` 中组合多个节点及其参数。

- **`src/offboard_test/launch/around_house_mission.launch.py`**
  - 作用：启动围绕房屋飞行/避障相关的测试任务。
  - 主要内容：类似上一个 launch 文件，配置并启动与“绕房屋飞行”场景相关的节点。

#### `src/offboard_test/test/` 目录

- **`src/offboard_test/test/test_pep257.py`**
  - 作用：用于检查 docstring 规范（PEP 257）的测试脚本。
  - 主要内容：通过 `pytest` 或类似框架，调用 PEP257 检查工具，对包内 Python 文件的文档字符串进行静态检查。

- **`src/offboard_test/test/test_flake8.py`**
  - 作用：用于运行 Flake8 代码风格检查的测试脚本。
  - 主要内容：集成 Flake8，对源码进行静态代码风格/潜在错误检查。

- **`src/offboard_test/test/test_copyright.py`**
  - 作用：检查源码文件头部是否包含版权/许可证信息的测试脚本。
  - 主要内容：扫描源码文件并断言版权声明是否符合项目要求。

---

## 整体架构与数据流

整个系统由三部分组成：

- **Unity 仿真前端**
  - 提供 3D 场景（如房屋、低多边形环境）、无人机模型和相机。
  - 通过 C# 脚本发布 RGB 图像、深度图、静态相机生成的目标 GPS、无人机 odom 等 ROS 话题。

- **ROS2 算法后端（本仓库）**
  - 订阅 Unity 侧发布的传感器和目标信息。
  - 使用 PX4 Offboard 接口执行起飞、路径规划和避障控制。
  - 可向 Unity 发布规划路径等数据用于显示。

- **PX4 / 仿真或真机飞控**
  - 接收 `/fmu/in/*` 控制话题，执行位姿控制。
  - 发布 `/fmu/out/*` 状态话题，反馈给算法和 Unity。

典型数据流（以 A\* 追踪为例）：

1. Unity 中的 OAK 相机脚本发布：
   - `/oak/rgb/image_raw`、`/oak/rgb/camera_info`
   - `/oak/depth/image_rect_raw`、`/oak/depth/camera_info`
2. Unity 静态相机脚本将画面中心的 3D 点转换为 GPS，发布：
   - `/static_cam/target_gps`
3. 本仓库中的算法节点（如 `AStarGridPursuit`）订阅：
   - `/oak/depth/image_rect_raw`、`/oak/depth/camera_info`
   - `/drone/gps`（home）、`/static_cam/target_gps`（目标）
   - `/fmu/out/vehicle_local_position_v1`（PX4 局部 NED 位置）
4. 算法构建占据栅格并运行 A\*，生成路径，并发布：
   - `/planning/path`（`PoseArray`，用于可视化）
   - `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`、`/fmu/in/vehicle_command`
5. PX4 执行 Offboard 控制，无人机飞行状态通过 odom 回写到 Unity，可视化飞行过程。

---

## 目录结构（算法工程）

主要目录位于 `Algorithm_group/DroneRanger-Autonomy`：

- `src/drone_autonomy/`
  - `drone_autonomy/missions/`
    - `astar_grid_pursuit.py`：基于 2D 栅格 A\* 的追踪任务，使用深度图构建占据栅格和 world grid，规划路径并控制 PX4。
    - `rrt_pursuit_single.py`：RRT-like 直线步进追踪 + 前向深度避障的任务节点。
    - 其他 `astar_*` 变体：不同记忆/3D 扩展等 A\* 实现（如 `astar_world_buffer.py`、`astar_no_memory_conservative.py`、`astar_grid_pursuit_3D.py`）。
  - `drone_autonomy/haoran_alg_update/`
    - `haoran_unity_pursuit.py`：Unity 仿真用 VFH+ 水平避障 + 目标追踪节点（waypoint 步长、fallback 直线追目标）。
    - `haoran_unity_pursuit02.py`：增强版，2D 扇区全角度、上下避障、大障碍停+爬升、扇区膨胀。
    - `march_10_report.txt`：算法记录文档。
  - `launch/`
    - 各类简单起飞和任务的启动文件（如 `simple_takeoff.launch.py`）。
  - `docs/Algorithm_Design_Breakthrough.md`
    - 算法设计文档（详述路径规划与避障思路）。

- `src/offboard_avoidance_unity/`
  - `offboard_avoidance_unity/unity_house_avoidance.py`
    - 面向 Unity 房屋场景的避障任务，沿预设航线飞行并利用深度图在侧向自动绕障。
  - `setup.py`、`setup.cfg`、`package.xml`
    - ROS2 包定义与依赖配置。

- `src/offboard_test/`
  - `offboard_test/offboard_takeoff.py`
  - `offboard_test/offboard_square_mission.py`
  - `offboard_test/obstacle_avoidance_mission.py`
  - `offboard_test/obstacle_avoidance_dynamic.py`
  - `launch/*.launch.py`
    - 各种 Offboard 起飞/方形轨迹/障碍物场景的测试任务与启动文件。

---

## 关键算法节点概览

### `AStarGridPursuit`（`drone_autonomy/missions/astar_grid_pursuit.py`）

- **输入话题**
  - `/oak/depth/image_rect_raw`、`/oak/depth/camera_info`：来自 Unity OAK 相机的深度数据。
  - `/drone/gps`：当前无人机 GPS（用于设置 home）。
  - `/static_cam/target_gps`：Unity 静态相机生成的目标 GPS。
  - `/fmu/out/vehicle_local_position_v1`：PX4 局部位置（NED）。
- **输出话题**
  - `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`、`/fmu/in/vehicle_command`：用于控制 PX4 Offboard。
  - `/planning/path`：`PoseArray` 格式的规划路径，供可视化。
- **核心逻辑**
  - 利用深度图在本地 NED 平面上构建占据栅格，并维护一个累计的 world grid。
  - 在 2D 栅格上运行 A\* 搜索，从当前 NED 位置到目标 NED 目标点。
  - 通过状态机（WARMUP → ARMING → TAKEOFF → HOVER_STABLE → PLAN_FOLLOW）控制任务流程。

### `RRTPursuitFixed`（`drone_autonomy/missions/rrt_pursuit_single.py`）

- **思路**
  - 每一步朝目标 NED 方向推进一个固定步长（类似 RRT 的连线扩展），并利用深度图前方 ROI 做简单避障。
- **避障机制**
  - 从深度图中取一块前向 ROI，计算距离百分位，若低于阈值则认为“前方被挡”。
  - 进入一个简单状态机：停止 → 右转扫描 → 侧前方绕过 → 回到朝向目标的正常追踪。

### `UnityHouseAvoidance`（`offboard_avoidance_unity/unity_house_avoidance.py`）

- **场景定位**
  - 为 Unity 中的“房屋/建筑”场景设计的任务节点，沿一条预设 ENU 航线飞行。
- **功能**
  - 订阅 `/oak/depth/image_rect_raw` 和 `/drone/odom`。
  - 在每个航点之间飞行时，对前方深度图做障碍检测，如有障碍则添加侧向偏移 `avoid_offset` 实现绕障。

### `HaoranUnityPursuit` / `haoran_unity_pursuit02`（`drone_autonomy/haoran_alg_update/`）

- **定位**
  - 专为 Unity 仿真设计的避障 + 目标追踪模板节点，与现有 PX4 Offboard、Unity 话题接口一致，便于在 Unity 场景中直接测试算法。
- **输入话题**
  - `/fmu/out/vehicle_local_position_v1`：PX4 局部 NED 位置与航向。
  - `/drone/gps`：当前无人机 GPS，首次收到即设为 home。
  - `/static_cam/target_gps`：Unity 静态相机生成的目标点（可配置话题名）。
  - `/oak/depth/image_rect_raw`、`/oak/depth/camera_info`：Unity OAK 深度图。
- **输出话题**
  - `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`、`/fmu/in/vehicle_command`：PX4 Offboard 控制。
- **状态机**
  - WARMUP → ARMING → TAKEOFF → HOVER_STABLE → EXECUTE（算法在 EXECUTE 阶段工作）。
- **`haoran_unity_pursuit.py`（基础版）**
  - 水平 VFH+：深度图按水平 FOV 划分扇区，选“安全且朝向目标”的航向；按障碍距离做速度门控，直接给出前方 waypoint 步长由 PX4 飞过去。
  - 无深度/无可行扇区时：可退化为直线追目标（fallback）或原地 yaw 扫描。
  - 主要可调参数：`hfov_deg`、`num_sectors`、`min_clear_dist_m`、`stop_dist_m`、`max_speed_m_s`、`fallback_to_goal_when_no_depth`、`fallback_speed_m_s` 等。
- **`haoran_unity_pursuit02.py`（增强版）**
  - **前方全角度**：2D 扇区网格（水平×垂直），VFH 在水平+俯仰方向选方向，支持斜上/斜下飞行；参数 `use_2d_sectors`、`vfov_deg`、`num_sectors_vertical`。
  - **上下避障**：图像上/下条带最近距离，上方近→向下避让、下方近→向上避让，微调高度；参数 `vertical_avoid_enable`、`vertical_near_m`、`vertical_offset_max_m`。
  - **大障碍物防护**：前方全局最近距离低于 `large_obstacle_threshold_m` 时视为“墙”，停止水平前进并每周期爬升 `large_obstacle_climb_m`，避免撞比视野大的物体。
  - **扇区膨胀**：近障扇区向邻格膨胀（`sector_inflation_cells`），避免从大物体边缘钻过。
- **运行示例**
  - `ros2 run drone_autonomy haoran_unity_pursuit` 或 `ros2 run drone_autonomy haoran_unity_pursuit02`（需在 `setup.py` 中注册对应 entry point）。

---

## 与 Unity 工程的关系（简述）

上层仓库 `Drone-Ranger` 的 `Assets/scripts` 中，存在若干与本算法工程配套的 C# 脚本：

- `OakLitePublisher.cs`：发布 `/oak/rgb/*` 和 `/oak/depth/*`，为本工程提供 RGB + 深度传感器输入。
- `StaticCamTargetGPSPublisher.cs`：将 Unity 世界中的目标点转换为 GPS，发布 `/static_cam/target_gps`，作为追踪/规划目标。
- `DronePoseSubscriber`：订阅无人机 odom，驱动 Unity 中的无人机模型，实现飞行状态可视化。

因此，本仓库**不直接包含 Unity 代码**，但与 Unity 工程通过上述 ROS 话题强耦合，二者配合构成完整的“仿真 + 自主导航”系统。

---

## 运行方式（示意）

以下是可能的运行流程，仅供参考，具体以实际 launch 文件和部署环境为准：

1. **构建与环境准备**
   - 在 ROS2 工作区中克隆本仓库到 `Algorithm_group/DroneRanger-Autonomy`。
   - 安装依赖并执行：
     - `colcon build`
     - `source install/setup.bash`

2. **启动 PX4 / 仿真环境**
   - 启动 PX4 SITL 或连接真实 PX4，并确保 uXRCE-DDS / ROS2 桥已正确运行。

3. **启动算法节点**
   - 例如：
     - `ros2 launch drone_autonomy simple_takeoff.launch.py`
     - 或运行某个具体任务节点（如 A\* 追踪、Unity house 避障等）。

4. **启动 Unity 仿真**
   - 在 Unity 中打开上层 `Drone-Ranger` 工程，加载 `Scenes/SampleScene.unity`。
   - 确保 ROS-TCP 连接参数正确，相机和无人机对象挂载了相应脚本。
   - 点击 Play，确认与 ROS2 侧的连接正常。

在此基础上，Unity 中的相机和目标生成脚本会不断向 ROS2 发布数据，本工程的算法节点将利用这些数据完成无人机起飞、路径规划和避障飞行。

