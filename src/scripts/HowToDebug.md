# PX4 + ROS 2 联调与排障流程

本文档面向在本仓库（`drone_local`）中调试 **PX4 SITL / 真机桥接 + Micro XRCE-DDS + ROS 2 Humble** 的工程师。目标是把问题**按层次隔离**：先保证飞控与仿真健康，再接地站与网络，再 DDS 与话题，最后 Offboard 与业务节点。

---

## 1. 总览：调试顺序（为什么必须按这个顺序）

链路依赖关系可以概括为：

```text
PX4（估计器 / 预检） →（可选）QGC / MAVLink
        ↓
MicroXRCEAgent ←UDP→ PX4 内 uxrce_dds_client
        ↓
ROS 2（/fmu/out、/fmu/in）
        ↓
Offboard 节点（offboard_control_mode + trajectory_setpoint + vehicle_command）
        ↓
（可选）Unity / ROS-TCP-Endpoint / 自定义桥
```

**原则**：下面某一环失败时，不要继续往后加组件（不要同时开 Unity、自定义脚本和多个 PX4）。每一阶段都有**可观测的通过条件**；未满足则只在本阶段排查。

**阶段索引（与下文章节编号一致）**：


| 阶段  | 内容                                  |
| --- | ----------------------------------- |
| 0   | 清理进程                                |
| 1   | 启动 PX4 / 仿真                         |
| 2   | `pxh>` 健康与预检                        |
| 3   | QGC / MAVLink（可选）                   |
| 4   | MicroXRCEAgent + `uxrce_dds_client` |
| 5   | `source` 工作空间、`px4_msgs`            |
| 6   | `/fmu/out` 话题                       |
| 7   | `/fmu/in` 话题                        |
| 8   | `vehicle_command_ack`               |
| 9   | Offboard 节点 / `test.py`             |


**与仓库脚本的对应关系**：


| 阶段    | 你手动调试时                            | 一键脚本 `start_drone.sh`                 |
| ----- | --------------------------------- | ------------------------------------- |
| PX4   | `make px4_sitl_default …`         | 默认 `PX4_SITL_TYPE=none`（无 Gazebo 动力学） |
| Agent | `MicroXRCEAgent udp4 -p 8888`     | 会起                                    |
| 工作空间  | `colcon build` + `source install` | 需事先编好；见 `scripts/README.md`           |


工作空间初始化与编译（子模块、`build_ros.sh`）见：**[scripts/README.md](./README.md)** 一节「drone_ws: submodule init and build」。

---

## 2. 前置条件（开始任何排障前）

- Ubuntu 上已安装 **ROS 2 Humble**，且能 `source /opt/ros/humble/setup.bash`。
- `drone_ws` 已按 README 完成 **子模块 + `colcon build`**，`install/setup.bash` 存在。
- 明确本机路径：下文以 `~/drone_local` 为例，请替换为你的 `**DRONE_ROOT**`。

---

## 3. 阶段 0：清理旧进程

**目的**：避免多个 PX4 / Gazebo / Agent / `ros2` 残留导致端口占用、时间戳错乱或 IMU 报错。

**步骤**：

1. 若使用仓库脚本启动过 tmux：
  ```bash
   tmux kill-session -t drone 2>/dev/null || true
  ```
2. 或执行仓库 `stop_drone.sh`（若存在）。
3. 结束常见相关进程（按需）：
  ```bash
   pkill -f 'px4|MicroXRCEAgent' 2>/dev/null || true
   ros2 daemon stop 2>/dev/null || true
  ```
4. 确认无残留：
  ```bash
   ps aux | egrep 'px4|gz|gazebo|MicroXRCEAgent' | grep -v grep
  ```

**通过条件**：无多余 PX4/仿真/Agent 进程（或仅保留你本次要用的那一个）。

---

## 4. 阶段 1：启动 PX4（任选一种仿真形态）

**目的**：先让飞控与（可选）仿真跑起来，再谈 ROS。

### 4.1 带 Gazebo 的机型（推荐用于「看飞机动」）

```bash
cd ~/drone_local/PX4-Autopilot
make px4_sitl gz_x500
```

**通过条件**：终端出现 Gazebo/模型就绪、PX4 进入 `pxh>`；初期 EKF 可能短暂告警，属常见现象。也可以使用make px4_sitl_default none，但是通常会有一些fmu/out/ 的初始化参数是nan.

### 4.2 无 Gazebo（`none`，本仓库 `start_drone.sh` 默认）

```bash
cd ~/drone_local/PX4-Autopilot
make px4_sitl_default none
```

**说明**：无完整动力学可视化时，**机体在屏幕上可能“看起来不动”**，但话题仍可能正常；若你要验证飞行力学，优先用 **4.1**。

**若本阶段失败**：不要进入 ROS；检查 PX4 编译、显示/GPU、VM 资源等。

---

## 5. 阶段 2：在 `pxh>` 做飞控健康检查

**目的**：确认 **预检、姿态、本地位置** 在可接受范围，避免后面 Offboard 全被 `Arming denied` 挡住。

**步骤**（在 PX4 控制台 `pxh>`）：

1. 总览预检：
  ```bash
   commander check
  ```
2. 抽样看估计（可选）：
  ```bash
   listener vehicle_attitude 1
   listener vehicle_local_position 1
  ```

**通过条件**：无持续 `Preflight Fail`（如 `Attitude failure (roll)`、`ekf2 missing data` 等）或你能解释并消除其原因。

**常见阻塞**：


| 现象                            | 排查方向                                      |
| ----------------------------- | ----------------------------------------- |
| `Attitude failure (roll)`     | 等待 EKF 收敛；机体水平；仿真中模型是否异常；勿过早发 ARM         |
| `No connection to the GCS`    | 见阶段 6；或参数 `NAV_DLL_ACT`（仅当你明确不需要 GCS 预检时） |
| `vehicle_imu timestamp error` | 清进程、单实例重开；减轻 VM 负载                        |


---

## 6. 阶段 3：地面站 QGC（可选）与 MAVLink

**目的**：若你**需要 QGC 看姿态/地图**，或预检要求 GCS，则把 PX4 的 MAVLink 指到运行 QGC 的机器。

**步骤**（在 `pxh>`）：

1. 如需重置链路：
  ```bash
   mavlink stop-all
  ```
2. 向 QGC 所在主机 IP 发 UDP（示例）：
  ```bash
   mavlink start -x -u 14550 -r 40000 -t <QGC_HOST_IP>
  ```
   其中 `-u 14550` 为 UDP 端口，**须与 QGC 侧链路配置的端口一致**。

3. **QGroundControl 侧：配置 UDP 监听（端口 14550）**

   下面以常见 QGC 界面为序（**中文界面**下菜单名称可能为「应用设置」「通信链接」等；英文为 **Application Settings → Comm Links**）。

   **3.1 打开通信链接设置**

   - 打开 QGroundControl。
   - 进入 **设置 / 应用设置**（齿轮图标）。
   - 左侧选择 **Comm Links**（通信链接）。

   **3.2 关于 AutoConnect（UDP）**

   - 页面上方常有 **AutoConnect** 区域，其中 **UDP** 开关若开启，可能与手动链路冲突。
   - 对话框中若出现提示：*For best performance, please disable AutoConnect to UDP devices on the General page*，建议在 **常规 / General** 中关闭对 UDP 设备的自动连接，再使用下面「手动添加链路」。

   **3.3 添加新链路**

   - 在 **Links** 区域找到 **Add New Link**（添加新链接）。
   - 点击 **添加 / Add**，打开 **Add New Link**（添加新链接）对话框。

   **3.4 在「添加新链接」对话框中填写**

   | 字段 | 建议值 | 说明 |
   |------|--------|------|
   | **Name** | 任意名称（如 `PX4_UDP`、`parallel`） | 仅用于在列表中区分链路。 |
   | **开始时自动连接** | 可按需关/开 | 若调试阶段频繁重启 QGC，可先关闭，手动点连接。 |
   | **高延迟** | 一般 **关** | 仅在链路很差时启用。 |
   | **Type** | **UDP** | 必须与 PX4 侧 `mavlink start -u 14550` 使用 UDP 一致。 |
   | **Port** | **14550** | **监听端口**，与 PX4 命令中的 `-u 14550` 一致。 |

   - **Server Addresses（可选）**：多数场景下 **留空** 即可——QGC 在本机 **监听** UDP `14550`，PX4 已用 `mavlink start -t <QGC_IP>` 把数据 **发到** 该主机；若你的 QGC 版本要求显式填写，可改为 `0.0.0.0:14550` 或本机 IP，以文档/版本为准。
   - 点击 **Save / 保存** 保存链路。

   **3.5 连接与确认**

   - 回到 **Comm Links** 列表，选中刚创建的链路，点击 **连接**（若尚未连接）。
   - 主界面应出现 **已连接** 的飞机/仿真器；若仍无数据，检查：
     - PX4 是否已执行上面的 `mavlink start … -t <QGC_HOST_IP>`，且 `<QGC_HOST_IP>` 就是运行 QGC 的那台机器的网卡 IP；
     - 防火墙是否放行 UDP `14550`；
     - 端口未被其它程序占用。

**通过条件**：QGC 显示已连接；再次 `commander check` 若依赖 GCS 则应变 OK。

**本仓库快捷方式**：`./start_drone.sh --qgc-ip <IP>` 可在延时后自动向 PX4 写入上述类命令（见 `scripts/README.md`）。

**仅 ROS 调试、不想开 QGC**：可在 `pxh>` 执行 `param set NAV_DLL_ACT 0`（含义与风险见 `README.md`），**不要**与真实法规要求冲突。

---

## 7. 阶段 4：Micro XRCE-DDS Agent 与 PX4 内 uxrce_dds_client

**目的**：让 ROS 2 能发现 PX4 的 DDS 话题（`/fmu/out/`*、`/fmu/in/*`）。

**步骤**：

1. **终端 A**（宿主机，与 PX4 同机或网络可达）：
  ```bash
   MicroXRCEAgent udp4 -p 8888
  ```
2. **PX4 `pxh>`** 确认或启动客户端（与 Agent 一致）：
  ```bash
   uxrce_dds_client status
   uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
  ```
   若已 `Running, connected`，则无需重复 `start`。

**通过条件**：`uxrce_dds_client status` 显示 **connected**；Agent 终端有流量统计。

---

## 8. 阶段 5：ROS 2 工作空间环境

**目的**：确认 `px4_msgs` 与当前工作空间已加载。

**步骤**：

```bash
source /opt/ros/humble/setup.bash
source ~/drone_local/drone_ws/install/setup.bash
ros2 pkg list | grep px4_msgs
```

**通过条件**：能列出 `px4_msgs`；若报错，回到 `scripts/README.md` 编译流程。

---

## 9. 阶段 6：验证 `/fmu/out`（PX4 → ROS）

**目的**：证明 **PX4 → uXRCE → ROS 2** 通路正常。

**步骤**：

```bash
ros2 topic list | grep '^/fmu/out'
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_odometry --once
```

**通过条件**：`vehicle_odometry` 有稳定频率；消息中有 `position`、`velocity`、`q` 等字段。

---

## 10. 阶段 7：验证 `/fmu/in`（ROS → PX4）

**目的**：确认订阅端存在**输入话题**（由桥接注册，不一定有发布者）。

**步骤**：

```bash
ros2 topic list | grep '^/fmu/in'
```

**至少应存在**（名称以你固件版本为准）：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

**通过条件**：列表中出现上述话题；否则检查 uXRCE 与 PX4 侧客户端。

---

## 11. 阶段 8：验证命令回执 `/fmu/out/vehicle_command_ack`

**目的**：Offboard / 解锁是否被飞控**拒绝**，以 ACK 为准。

**步骤**：

```bash
ros2 topic echo /fmu/out/vehicle_command_ack
```

**判读**（示例）：

- `result: 0`（`VEHICLE_CMD_RESULT_ACCEPTED`）通常表示接受。
- `command` 与 `VehicleCommand` 中命令号对应（如模式切换、ARM）。

**通过条件**：发命令时能看到对应 ACK；若持续 `DENIED`，回到阶段 2（预检）与阶段 6（GCS/参数）。

---

## 12. 阶段 9：最小 Offboard 与本地 `test.py`

**目的**：在**持续发布** `OffboardControlMode` + `TrajectorySetpoint` 的前提下切换模式并解锁，避免“未预热就 ARM”。

### 12.1 推荐：包内节点 `offboard_takeoff`

```bash
ros2 run offboard_test offboard_takeoff --ros-args -p verbose_setpoint_log:=true
```

（参数见节点源码与 `README.md`；含 ACK/重试逻辑时以当前版本为准。）

### 12.2 本地测试脚本 `test.py`（方形任务示例）

**路径（仓库内）**：

```text
drone_ws/src/offboard_test/offboard_test/test.py
```

**绝对路径示例**：

```text
~/drone_local/drone_ws/src/offboard_test/offboard_test/test.py
```

**说明**：该文件为包内 **Python 脚本**，默认**未**注册为 `console_scripts`；用于本地快速试验。运行前必须已 `source` 工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/drone_local/drone_ws/install/setup.bash
python3 ~/drone_local/drone_ws/src/offboard_test/offboard_test/test.py
```

**通过条件**：终端出现预热 setpoint、切换 Offboard、ARM、航点日志；同时 `vehicle_odometry` 中位置随任务变化（若仿真为 `none` 则可能变化不明显）。

---

## 13. 常见问题速查


| 症状                                | 优先检查                                   |
| --------------------------------- | -------------------------------------- |
| 无 `/fmu/out`                      | 阶段 4（Agent + `uxrce_dds_client`）→ 阶段 6 |
| `ros2 topic hz` 为 0               | 网络、防火墙、PX4 与 Agent 是否同机可达              |
| `px4_msgs` 找不到                    | 阶段 5；`scripts/README.md` 编译与子模块        |
| `Arming denied` / `preflight`     | 阶段 2；阶段 3（GCS / `NAV_DLL_ACT`）         |
| Offboard 命令 ACK 但飞机不动             | 阶段 2 模式；是否持续发设定点；`none` 仿真无动力学         |
| `The message type ... is invalid` | 阶段 5；`source install/setup.bash`       |


---

## 14. 总结：推荐检查清单（全部再打勾后再做 Unity/复杂任务）

- 阶段 0：无旧进程干扰  
- 阶段 1：PX4（或 `gz_x500`）能稳定启动  
- 阶段 2：`commander check` 无不可解释的持续失败  
- 阶段 6（若需要）：QGC 或 `NAV_DLL_ACT` 策略已明确  
- 阶段 4：Agent + `uxrce_dds_client` **connected**  
- 阶段 5：`px4_msgs` 可用  
- 阶段 6：`/fmu/out/vehicle_odometry` 有数据  
- 阶段 7：`/fmu/in/...` 控制话题存在  
- 阶段 8：发模式/ARM 时 `vehicle_command_ack` 合理  
- 阶段 9：`offboard_takeoff` 或 `test.py` 行为符合预期

---

## 15. 命令速查（复制用）

```bash
# PX4 + Gazebo
cd ~/drone_local/PX4-Autopilot && make px4_sitl gz_x500

# PX4 shell
commander check

# MAVLink → QGC（示例）
# mavlink start -x -u 14550 -r 40000 -t <QGC_IP>

# Agent
MicroXRCEAgent udp4 -p 8888

# ROS 2
source /opt/ros/humble/setup.bash
source ~/drone_local/drone_ws/install/setup.bash

# 话题
ros2 topic list | grep /fmu/
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_command_ack
```

更完整的 **子模块初始化、编译、一键启动** 见 **[scripts/README.md](./README.md)**。