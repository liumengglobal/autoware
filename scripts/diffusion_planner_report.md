## diffusion_planner 编译、运行、仿真报告

---

### 1. 环境配置

#### 容器创建（宿主机执行）

```bash
# 1. 下载 autoware 镜像
docker pull ghcr.io/autowarefoundation/autoware:universe-dependencies-cuda-jazzy-20260722

# 2. 创建容器（持久化，退出后不删除）
docker run -dit \
  --name autoware_diffusion_planner \
  --net host \
  --gpus all \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e HOST_UID=$(id -u) \
  -e HOST_GID=$(id -g) \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/liu/github/autoware:/home/aw/autoware \
  -v /home/liu/bag/autoware:/home/aw/data \
  -w /home/aw/autoware \
  --runtime=nvidia \
  ghcr.io/autowarefoundation/autoware:universe-dependencies-cuda-jazzy-20260722 \
  bash -c "source /opt/autoware/setup.bash && exec bash"

# 3. 安装 rviz2（基础镜像不含）
docker exec autoware_diffusion_planner bash -c "apt-get update -qq && apt-get install -y -qq ros-jazzy-rviz2"
```

#### 容器信息

| 项目 | 值 |
|------|-----|
| 镜像 | `ghcr.io/autowarefoundation/autoware:universe-dependencies-cuda-jazzy-20260722` |
| 容器名 | `autoware_diffusion_planner` |
| 基础镜像角色 | `universe-dependencies-cuda`（具备所有构建依赖 + CUDA/TensorRT） |
| 用户名 | `aw`（HOME=`/home/aw`） |
| ROS 发行版 | Jazzy |
| RMW | 脚本强制 `rmw_fastrtps_cpp`（见 §5.4：CycloneDDS 对 ~2KB tracked_objects 有 take 缺陷） |
| 目录映射 | `/home/liu/github/autoware` → `/home/aw/autoware`；`/home/liu/bag/autoware` → `/home/aw/data` |
| GPU | CUDA + TensorRT（`CMAKE_CUDA_ARCHITECTURES=86;87;89;90;110`） |

#### 模型文件下载与部署

模型文件下载在宿主机（HuggingFace 当前不可达，本次使用宿主已有完整副本）：

```bash
# 宿主模型目录（已下载完成，文件见下方清单）
ls /home/liu/github/autoware_data/ml_models/diffusion_planner/diffusion_planner/v5.0/

# 部署到容器：经目录映射（/home/liu/bag/autoware = 容器 /home/aw/data）
mkdir -p /home/liu/bag/autoware/ml_models/diffusion_planner
cp -a /home/liu/github/autoware_data/ml_models/diffusion_planner/. \
  /home/liu/bag/autoware/ml_models/diffusion_planner/
```

容器内路径即脚本 `MODEL_DATA_PATH = "/home/aw/data/ml_models/diffusion_planner"`。

#### 模型文件清单

```
/home/aw/data/ml_models/diffusion_planner/diffusion_planner/v5.0/
├── diffusion_planner.param.json          (107 KB)
├── diffusion_planner_encoder.onnx        (29 MB)
├── diffusion_planner_decoder.onnx        (28 MB)
└── diffusion_planner_turn_indicator.onnx (6.7 KB)
```

#### 源码路径

```
/home/liu/github/autoware/src/universe/autoware_universe/planning/autoware_diffusion_planner/
```

---

### 2. 编译

#### 核心包编译

```bash
# 进入容器
docker exec -it autoware_diffusion_planner bash

# 编译扩散规划器
source /opt/autoware/setup.bash
source /home/aw/autoware/install/setup.bash

colcon build --packages-select autoware_diffusion_planner \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 4
```

#### 依赖包编译

首次编译时需要先编译以下依赖包（仅需一次）：

```bash
# 消息包（纯 .msg 文件，编译快）
colcon build --packages-select \
  tier4_debug_msgs tier4_metric_msgs tier4_control_msgs tier4_external_api_msgs \
  tier4_system_msgs tier4_planning_msgs tier4_vehicle_msgs tier4_rtc_msgs \
  tier4_api_msgs tier4_api_utils tier4_perception_msgs \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 扩散规划器及其直接依赖
colcon build --packages-select \
  autoware_cuda_dependency_meta autoware_tensorrt_common \
  autoware_cuda_utils autoware_traffic_light_utils \
  autoware_tensorrt_plugins autoware_diffusion_planner \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 编译注意事项

| 问题 | 说明 | 解决 |
|------|------|------|
| `autoware_tensorrt_plugins` 缺少 `ament_index` | spconv 不可用，编译为空壳，但缺少 `local_setup.bash` 和 `package.xml` | 手动创建缺失文件 |
| `autoware_launch` 无法完整编译 | 依赖大量未编译的 universe 包，容器内存/时间限制 | 改用 `run_diffusion_planner_sim.py` 直接启动各组件 |

---

### 3. 运行

#### 一键启动脚本：`run_diffusion_planner_sim.py`

**路径**：`/home/liu/github/autoware/scripts/run_diffusion_planner_sim.py`

**功能**：启动以下组件并协调运行。脚本在 `main()` 中强制 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 并清空 `CYCLONEDDS_URI`（见 §5.4）。

| 组件 | 启动方式 | 说明 |
|------|---------|------|
| 地图加载器 | `ros2 run autoware_map_loader` | 加载测试地图（`LocalCartesianUTM` 投影） |
| 输入发布器 | Python `rclpy` 节点 | 发布 route / odometry / acceleration / tracked_objects(20Hz) / traffic_signals / turn_indicators / projector(2s 重发) |
| diffusion_planner | `ros2 launch` | 加载模型，规划轨迹 |
| rviz2 | `rviz2 -d /home/aw/autoware/scripts/diffusion_planner_sim.rviz` | 可视化显示 |

#### 输入话题

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `~/input/odometry` | `nav_msgs/Odometry` | 10 Hz | 车辆沿路线移动（速度跟随规划轨迹） |
| `~/input/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` | 10 Hz | 空 |
| `~/input/route` | `autoware_planning_msgs/LaneletRoute` | 1次（transient_local） | 24 个 lanelets，497.6m |
| `~/input/vector_map` | `autoware_map_msgs/LaneletMapBin` | 周期性重发（transient_local） | 183 个 road lanelets |
| `~/input/tracked_objects` | `autoware_perception_msgs/TrackedObjects` | 20 Hz | 2 个障碍物（慢车 + 对向来车） |
| `~/input/traffic_signals` | `autoware_perception_msgs/TrafficLightGroupArray` | 10 Hz | 空 |
| `~/input/turn_indicators` | `autoware_vehicle_msgs/TurnIndicatorsReport` | 10 Hz | DISABLE |

#### 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/output/trajectory` | `autoware_planning_msgs/Trajectory` | 80 个轨迹点 |
| `~/output/trajectories` | `autoware_internal_planning_msgs/CandidateTrajectories` | 候选轨迹 |
| `~/output/predicted_objects` | `autoware_perception_msgs/PredictedObjects` | 预测对象 |
| `~/debug/route_marker` | `visualization_msgs/MarkerArray` | 路线标记（lifetime 0.2s） |
| `~/debug/lane_marker` | `visualization_msgs/MarkerArray` | 车道标记 |
| `~/debug/linestring_marker` | `visualization_msgs/MarkerArray` | 线串标记 |

---

### 4. 仿真工况

#### 测试地图

- `test_map/lanelet2_map.osm`（28,913 行，日本筑波地区）
- 坐标：`(35.902°N, 139.932°E)`
- 183 个 road lanelets
- 最长路线：24 个 lanelets，**497.6m**

#### 路线

```
起点: (129.9, 130.6) → 终点: (19.9, 10.3)
```

#### 障碍物（动态）

| 名称 | 类型 | 位置（progress） | 横向偏移 | 速度 | 尺寸 | 颜色 |
|------|------|-----------------|---------|------|------|------|
| parked_car_1 | 慢车（沿路线前进） | 起点 5% | 0.0m | 1.0 m/s | 4.0×2.0×1.5m | 红色 |
| oncoming_car | 对向来车（对向车道、逆向行驶） | 起点 6% | 左偏 2.0m | 4.0 m/s | 4.0×2.0×1.5m | 蓝色 |

> 说明：`parked_car_1` 原为静态停车（v=0），因模型不避让静态障碍物（§5.6），改为 1.0 m/s 慢车走动态通道。`oncoming_car` 沿对向车道（左偏 2.0m）逆向驶来，progress 递减、heading 翻转，会车点约在 progress 6%（ego 巡航段）。

#### 车辆运动

- ego 速度**跟随规划轨迹**（trajectory[10] 前瞻速度 + 低通平滑，见 §5.7），非固定速度
- 沿路线行驶（progress 0% → 95% → 0% 循环）

---

### 5. 关键技术问题与解决

#### 5.1 坐标系对齐

**问题**：`MapProjectorInfo` 使用 `MGRS` 投影时，地图坐标在 MGRS 网格坐标系（~3811, 73803），而路线在 UTM 坐标系（~130, 130），rviz2 中无法重叠显示。

**解决**：改用 `LocalCartesianUTM` 投影，地图和路线使用相同坐标系。

#### 5.2 调试标记持续显示

**问题**：`publish_debug_markers` 中 markers 的 `lifetime = 0.2s`，而规划频率 10 Hz，但 `on_timer()` 在数据不可用时提前返回，导致 markers 过期消失。

**解决**：`run_diffusion_planner_sim.py` 以 6.7 Hz 订阅并重新发布所有 debug markers（route_marker、lane_marker、linestring_marker），覆盖 0.2s 的 lifetime。

#### 5.3 地图可视化

**问题**：`LaneletMapBin` 无法直接在 rviz2 中显示。

**解决**：`run_diffusion_planner_sim.py` 使用 lanelet2 Python 库将 183 个 lanelets 转换为 `MarkerArray`（lanelet 边界 + 中心线），每 2 秒用 `transient_local` QoS 发布到 `/vector_map_marker`。

#### 5.4 InterProcessPollingSubscriber 兼容性（2 障碍物收不到）

**问题**：`Latest` 策略的 `take_data()` 单次 `take()` 在队列深度为 1 时存在竞态。但实测关键问题是：**CycloneDDS 对 ~2KB 的 `TrackedObjects`（含 2 个障碍物，2188 字节）polling take() 持续失败**（1 个障碍物 ~1KB 正常），且 `InterProcessPollingSubscriber` 禁止 depth>1（改 depth 启动即抛 `std::invalid_argument`）。

**解决**：改用 **FastDDS**（`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`），并在脚本 `main()` 中**清空 `CYCLONEDDS_URI`**（残留的 CycloneDDS URI 会破坏 FastDDS 的 map_loader 投影仪订阅）。2 障碍物即可正常接收。

> 注：曾尝试按上游思路修改 `polling_subscriber.hpp` 的 `Latest::take_data()` 为循环 drain，实测无效（depth-1 下 drain ≈ 单次 take），已回退还原。

#### 5.5 残留进程

**问题**：`Ctrl+C` 退出脚本后，子进程（map_loader、diffusion_planner_node、rviz2）残留。

**解决**：`cleanup()` 函数依次执行：
1. `os.killpg` 发 SIGTERM
2. 等待 3s
3. `proc.kill()` 发 SIGKILL
4. `pkill -9 -f` 按名称清理

#### 5.6 静态障碍物不避让（上游 TODO）

**问题**：静态停放车（v=0）模型完全无视（轨迹直穿、无减速）。根因：模型有独立 `static_objects` 输入（5×10，训练时仅含锥桶/护栏/C区标志/generic），但节点 `diffusion_planner_core.cpp` 中该输入**硬编码为全 0**（源码注释 `// TODO(Daniel): add static objects`），静态障碍物永远不被模型感知；而 v=0 的车走动态 `neighbor_agents` 通道也被忽略。

**解决（方案 B）**：把"停放车"改为 **1.0 m/s 慢车**（走动态通道），模型识别为动态障碍物并**减速跟车/近停**（实测：距障碍物 3-8m 时轨迹速度降至 ~0.3-0.4 m/s）。0.3 m/s 太慢仍被当静态无视，1.0 m/s 可靠触发。

#### 5.7 车辆穿过障碍物（sim ego 不执行轨迹）

**问题**：rviz 里车辆直穿障碍物。根因：sim 的 ego 以固定 3.0 m/s 沿路线前进，**从不执行规划轨迹**（轨迹只用于显示）。

**解决**：实现 ego **轨迹速度跟随**：
- `on_trajectory` 保存最新轨迹 `self.latest_traj`
- `follow_planned_trajectory(dt)`：以 `trajectory[10]`（~3m 前瞻）速度为命令 + 低通滤波（`0.5*prev+0.5*target`）平滑，沿路线推进并发布 odom
- 注意：`trajectory[0]` 速度会回声当前速度（无法作为命令），必须用前瞻点

#### 5.8 障碍物 marker 跳动

**问题**：障碍物 marker 挂在 2.0s 定时器上，1 m/s 障碍物每 2s 跳 ~2m。

**解决**：拆出独立 `publish_obstacle_markers()`，挂 **0.1s（10Hz）** 定时器；静态地图 marker（车道线/中心线）保持 2s。

#### 5.9 地图收不到（FastDDS transient_local）

**问题**：切换 FastDDS 后 planner 卡 "Waiting for map data"——590KB 的 `LaneletMapBin` 地图 transient_local 一次性发布，FastDDS 未重放给后订阅的 planner。

**解决**：`publish_projector()` 每 **2s 重发 `MapProjectorInfo`**，触发 map_loader 每次收到后重新加载并重发地图（transient_local），planner 即可收到。

#### 5.10 rviz 配置丢失

**问题**：`/home/aw/.rviz2/diffusion_planner.rviz` 在容器重建时丢失（不在挂载目录），rviz 加载默认空配置无显示。

**解决**：新建 `diffusion_planner_sim.rviz` 放入工作区（`/home/aw/autoware/scripts/`，随挂载持久化），含 Trajectory / MapMarkers / RouteMarker / LaneMarker / LinestringMarker 5 个显示层，脚本改用该路径。

---

### 6. 验证结果

#### 轨迹验证脚本：`validate_trajectory.py`

**路径**：`/home/liu/github/autoware/validate_trajectory.py`

| 检查项 | 结果 | 说明 |
|--------|------|------|
| point_count | ✅ 80 points | 轨迹点数正确 |
| total_length | ✅ 25~50m | 随车辆进度变化 |
| step_distance | ✅ max_step < 0.01m | 步长平滑 |
| velocity | ✅ max ~3.5 m/s | 速度合理 |
| timestamps | ✅ 8.0s 单调递增 | 时间戳正常 |
| start_pose | ✅ 距 odom < 0.3m | 起点与车辆一致 |
| end_pose | ❌ 距 goal 较远 | 预期内（仅覆盖 8s 规划） |
| **综合评分** | **~86%** | |

#### RViz2 配置

**路径**：`/home/aw/autoware/scripts/diffusion_planner_sim.rviz`（工作区内，随挂载持久化）

包含 5 个显示层：

| 显示层 | 插件类型 | 话题 |
|--------|---------|------|
| Trajectory | `rviz_plugins/Trajectory` | `/diffusion_planner_node/output/trajectory` |
| MapMarkers | `rviz_default_plugins/MarkerArray` | `/vector_map_marker` |
| RouteMarker | `rviz_default_plugins/MarkerArray` | `/diffusion_planner_node/debug/route_marker` |
| LaneMarker | `rviz_default_plugins/MarkerArray` | `/diffusion_planner_node/debug/lane_marker` |
| LinestringMarker | `rviz_default_plugins/MarkerArray` | `/diffusion_planner_node/debug/linestring_marker` |

#### 本次避让行为实测

| 场景 | 行为 |
|------|------|
| 动态慢车（1.0 m/s，同车道） | ego 减速跟车/近停（距 3-8m 时轨迹速度 ~0.3-0.4 m/s），不穿过 |
| 对向来车（4 m/s，对向车道） | ego 巡航状态会车、短暂减速后通过（不停车） |
| 静态障碍物（v=0） | 模型完全无视（直穿）——见 §5.6 |
| 横向避让（轻微绕行） | 模型**不产生横向偏移**（轨迹贴中心线）——见 §8 |

---

### 7. 启动命令汇总

```bash
# 进入容器
docker exec -it autoware_diffusion_planner bash

# 启动仿真（脚本内部自动设置 FastDDS，无需手动 export）
source /opt/autoware/setup.bash
source /home/aw/autoware/install/setup.bash
python3 /home/aw/autoware/scripts/run_diffusion_planner_sim.py

# 单独启动 rviz2（如果脚本已启动）
rviz2 -d /home/aw/autoware/scripts/diffusion_planner_sim.rviz

# 停止所有进程
docker exec autoware_diffusion_planner bash -c \
  "pkill -f run_diffusion_planner_sim; pkill -f diffusion_planner_node; pkill -f 'map_loader'; pkill -f rviz2"
```

---

### 8. 已知限制

1. **`autoware_launch` 未编译**：`planning_simulator` 完整仿真需要编译 `autoware_launch` 及其依赖链（约 100+ 包），因容器内存限制未完成。当前使用 `run_diffusion_planner_sim.py` 直接启动各组件。

2. **`autoware_tensorrt_plugins` 未完全编译**：spconv 不可用，插件库 `libautoware_tensorrt_plugins.so` 未生成，但不影响模型推理（仅影响 TensorRT 自定义算子插件）。

3. **模型不产生横向避让（轻微绕行）**：实测 lateral 2.0/2.5/3.0 + `centerline_guidance.start_time_s` 2.0/8.0 组合，规划轨迹在障碍物处横向偏移均 ~0（贴中心线）。根因是训练配置 `coeff_neighbor_collision_loss: 0.0`（无邻居碰撞损失），模型未学习横向避让行为，位置/参数均无法触发。对向会车表现为"短暂减速后通过"，非横向绕行。

4. **静态障碍物（停稳车 v=0）不被避让**：模型 `static_objects` 输入上游 `TODO(Daniel): add static objects` 未实现（恒为 0），静态车走动态通道也被忽略。当前用 1.0 m/s 慢车代理（方案 B），非真实静态障碍处理。

5. **planner 周期性 OOM 被杀**：宿主仅 15G 内存，TensorRT 节点运行数分钟后偶被 OOM killer 杀掉（`exit -9`）。缓解手段有限，长时间运行需观察或重启。

6. **测试地图非真实场景**：使用的 `test_map/lanelet2_map.osm` 为单元测试用地图，非真实自动驾驶场景地图，路线长度和复杂度有限。

7. **无点云地图**：`planning_simulator` 需要 `.pcd` 点云地图，当前测试地图仅有 `.osm` 文件，不影响规划器但会影响地图加载器的完整性检查。