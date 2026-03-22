# Drone scripts - README

## Project root and scripts symlink

From the project root (`~/drone_local`), you can create a `scripts` symlink so that `~/drone_local/scripts` points to the scripts inside the workspace:

```bash
cd ~/drone_local
ln -s drone_ws/src/scripts scripts
```

Then run scripts from the root with `./scripts/start_drone.sh` or `cd scripts && ./start_drone.sh`.

## Project layout (`tree -L 1`)

At the project root (`~/drone_local`):

```
.
├── Drone-Ranger          # Unity project (or symlink to it)
├── PX4-Autopilot         # PX4 firmware / SITL
├── docs
├── drone_ws              # ROS2 workspace (src, build, install)
├── logs                  # Runtime logs when using --no-tmux
└── scripts               # This folder (or symlink → drone_ws/src/scripts)
```

## One-click launcher: `start_drone.sh`

Start order: **MicroXRCEAgent → PX4 SITL → ROS-TCP-Endpoint → odom_bridge → Mission**.  
By default, it uses `$DRONE_ROOT/scripts/odom_bridge.py` (repo-local).

### Recommended workflow (Unity + PX4/ROS synced)

1. **Stop previous run** (if any): `./stop_drone.sh`
  - Kills tmux session and previous endpoint/odom_bridge/mission processes.
2. **Start**: `./start_drone.sh`
  - Starts Agent → PX4 → TCP endpoint → odom_bridge → mission (mission delayed by default).  
  - After ~18s it auto-sends `param set NAV_DLL_ACT 0` to PX4 (arming does not require a GCS).
  - Within the mission delay window, open Unity and press **Play**.
3. Mission will arm/takeoff and Unity will follow `/drone/odom`.

### Prerequisites

- ROS 2 Humble installed; `drone_ws` built at least once (`colcon build`).
- If starting PX4: `PX4-Autopilot` exists and `make px4_sitl_default none` works (or another SITL type).
- If using Unity: Unity ROSConnection IP/Port matches `ROS_IP`/`ROS_TCP_PORT`.

### Environment variables (recommended: set only `DRONE_ROOT`)


| Var                  | Meaning                     | Default                              |
| -------------------- | --------------------------- | ------------------------------------ |
| `DRONE_ROOT`         | Project root                | `/home/parallels/drone_local`        |
| `DRONE_WS`           | ROS 2 workspace             | `$DRONE_ROOT/drone_ws`               |
| `PX4_DIR`            | PX4 folder                  | `$DRONE_ROOT/PX4-Autopilot`          |
| `ROS_IP`             | TCP endpoint bind IP        | auto-detect                          |
| `ROS_TCP_PORT`       | TCP port                    | `10000`                              |
| `ODOM_BRIDGE_SCRIPT` | odom_bridge path            | `$DRONE_ROOT/scripts/odom_bridge.py` |
| `DEFAULT_MISSION`    | Default mission             | `offboard_takeoff`                   |
| `MISSION_DELAY_SEC`  | Delay before mission starts | `25`                                 |
| `PX4_SITL_TYPE`      | SITL type                   | `none`                               |
| `LOG_DIR`            | Logs directory              | `$DRONE_ROOT/logs`                   |


Example (set IP, start mission immediately):

```bash
export ROS_IP=10.211.55.3
./start_drone.sh --mission-delay 0
```

### Usage

```bash
cd /home/parallels/drone_local/scripts
chmod +x start_drone.sh
./start_drone.sh
```

Default: uses **tmux** (one window per component).  
Flags:

- `--no-tmux`: run in background and write logs into `$LOG_DIR`
- `--no-agent`: skip MicroXRCEAgent
- `--no-px4`: skip PX4
- `--no-tcp`: skip ROS-TCP-Endpoint
- `--no-odom-bridge`: skip odom_bridge
- `--mission-delay SEC`: delay before starting mission
- `--mission NAME`: select a mission node name

Examples:

```bash
# Only start ROS endpoint, no PX4
./start_drone.sh --no-px4 --no-agent

# Square mission, background mode
./start_drone.sh --no-tmux --mission offboard_square_mission
./start_drone.sh --mission offboard_square_mission

# Orbit mission (drone_autonomy)
./start_drone.sh --mission orbit_point

# Unity avoidance (odom_bridge enabled by default)
./start_drone.sh --mission unity_house_avoidance
```

### Supported `--mission` names

- **offboard_test**：`offboard_takeoff`、`offboard_square_mission`、`offboard_orbit_mission`、`obstacle_avoidance_mission`、`obstacle_publisher`
- **offboard_avoidance_unity**：`unity_house_avoidance`
- **drone_autonomy**：`orbit_point`、`forward_mission`、`simple_takeoff`、`square_flight`、`circle_flight`、`figure_eight`、`waypoint_mission`、`house_inspection`、`pursuit_flight`、`rrt_pursuit_single`、`haoran_unity_pursuit`

### Stop and re-run (repeatable)

```bash
./stop_drone.sh   # 结束 tmux 会话及上次 --no-tmux 的进程
./start_drone.sh  # 再次一键启动，行为与第一次一致
```

### Attach tmux session

脚本默认创建名为 `drone` 的 tmux 会话：

```bash
tmux attach -t drone
```

窗口顺序：`agent` → `px4` → `tcp` → `ros` → `mission`。用 `Ctrl+b n` / `Ctrl+b p` 切换窗口。

### Unity troubleshooting (not flying / transform changes but view looks static)

- **相机跟谁**：`CameraManager` 上的 **Drone to Follow** 必须指向**被 OdomSubscriber 更新的那个物体**（通常是挂了 `DronePoseSubscriber` 且 **Target** 为自己的 Drone 根节点）。未指定时相机会尝试自动查找并绑定。若 Console 出现 `[CameraSetup] Chase: Drone to Follow 为空`，说明相机没有跟随目标，Game 视图会看起来不动。
- **Game 视图用的是哪台相机**：确保 Game 标签页正在显示 **Main Camera** 的画面（而不是 Preview Camera / Map Camera 等）。`Simple Camera Setup` 只移动 Main Camera；若 Game 视图选错了相机，会看到静止画面。
- **看是否收到 odom**：在 Unity Console 里看是否有周期性的 `[DronePose] /drone/odom -> position=...` 日志（OdomSubscriber 的 Log Interval Sec > 0 时）。若一直没有，说明 Unity 没收到 `/drone/odom`（检查 ROS 连接、odom_bridge 是否在跑、Topic 是否为 `/drone/odom`）。
- **World Origin**：若飞机一直在建筑里或地上，检查 **World Origin** 是否指向场景里正确的起飞点空物体。若设为 Drone 自身，则用 Play 时 Drone 的初始位置作为原点，一般也可接受。
- **Missing (Mono Script)**：若 Drone 上有“Missing (Mono Script)”，请移除该空引用或恢复原脚本，否则可能影响模型显示或逻辑。

### Unity connectivity test & logs

- **OdomSubscriber** 勾选 **Log Connection And Params** 后，Play 时会打：
  - 一条 **Subscribe OK**：topic、target、worldOrigin 等，确认订阅参数正确；
  - 收到第一条 `/drone/odom` 时打 **First /drone/odom received -> ROS link OK**，说明与 ROS 的通讯成功；
  - 之后按 **Log Interval Sec** 周期打 **msg#=N** 和 Unity 位置，确认持续在收。
- **把 Console 记到文件**：在场景里任意物体上挂 **UnityLogToFile** 组件，Play 时会把 Console 同时写入 `Application.persistentDataPath/drone_ros_test.log`（Windows 一般在 `AppData/LocalLow/<CompanyName>/<ProductName>/`）。用该文件可事后检查 subscribe 与通讯是否成功。

### Fix No connection to the GCS / Arming denied (one-time setting; no QGC required)

**不需要打开 QGC 的 app。** 只要在 PX4 里关掉“必须连地面站才能解锁”的检查即可：

1. 在 PX4 的终端里找到 `**pxh>`** 提示符（若用 tmux：`tmux attach -t drone`，切到 px4 窗口）。
2. 输入下面这一行并回车：
  ```text
   param set NAV_DLL_ACT 0
  ```
3. 看到 `param set ...` 成功即可。**只需设置一次**，参数会保存，下次启动 PX4 仍然有效。

之后用 ROS 发解锁/起飞时就不会再报 “Preflight Fail: No connection to the GCS” 和 “Arming denied”。

### QGroundControl (optional)

- **不连 QGC 也能飞**：上面设置后，只用 ROS offboard 时**不必**打开 QGroundControl。
- **想用 QGC 看姿态/地图时**：PX4 默认 MAVLink 只在本机，需向 PX4 发 mavlink 命令。两种方式（**二选一**）：
  1. **脚本自动发**（推荐）：一键启动时加参数，脚本会在 PX4 启动约 30 秒后自动把命令发进 PX4：
    ```bash
     ./start_drone.sh --qgc-ip 10.211.55.2
    ```
     可选 `--qgc-wait 40` 若 PX4 启动较慢（默认 30 秒后再发）。
  2. **手动发**：PX4 已跑起来并出现 `pxh>` 后，在**本机**执行：
    ```bash
     ./enable_qgc.sh 10.211.55.2
    ```
     会把 `mavlink stop-all` 和 `mavlink start -x -u 14550 -r 40000 -t 10.211.55.2` 自动输入到 PX4 的 tmux 窗口。
  QGC 端添加 UDP 链接，端口 14550。

### Address already in use (TCP port 10000 is busy)

- **一键启动**：`./start_drone.sh` 会在起 TCP endpoint 前自动释放 10000 端口并等待空闲，一般不会再出现该错误。
- **单独起 TCP endpoint 时**：不要直接敲一长串 `source ... && ros2 run ...`，改用脚本（脚本会先释放端口再启动）：
  ```bash
  cd /home/parallels/drone_local/scripts
  ./run_tcp_endpoint.sh
  ```
  可选参数：`./run_tcp_endpoint.sh [ROS_IP] [PORT]`，例如 `./run_tcp_endpoint.sh 10.211.55.3 10000`。
- **仍报错时**：先执行 `./stop_drone.sh`，再执行 `./start_drone.sh` 或 `./run_tcp_endpoint.sh`。

