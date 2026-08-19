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
| RMW | `rmw_cyclonedds_cpp` |
| GPU | CUDA + TensorRT（`CMAKE_CUDA_ARCHITECTURES=86;87;89;90;110`） |

#### 模型文件下载

```bash
mkdir -p /home/liu/github/autoware_data/ml_models/diffusion_planner/diffusion_planner/v5.0
cd /home/liu/github/autoware_data/ml_models/diffusion_planner/diffusion_planner/v5.0
wget https://huggingface.co/AutowareFoundation/diffusion_planner/resolve/main/diffusion_planner.param.json
wget https://huggingface.co/AutowareFoundation/diffusion_planner/resolve/main/diffusion_planner_encoder.onnx
wget https://huggingface.co/AutowareFoundation/diffusion_planner/resolve/main/diffusion_planner_decoder.onnx
wget https://huggingface.co/AutowareFoundation/diffusion_planner/resolve/main/diffusion_planner_turn_indicator.onnx
```

#### 模型文件复制到容器

```bash
docker exec autoware_diffusion_planner mkdir -p /home/aw/autoware_data/ml_models
docker cp /home/liu/github/autoware_data/ml_models/diffusion_planner \
  autoware_diffusion_planner:/home/aw/autoware_data/ml_models/diffusion_planner
```

#### 模型文件清单

```
/home/aw/autoware_data/ml_models/diffusion_planner/diffusion_planner/v5.0/
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
| `autoware_launch` 无法完整编译 | 依赖大量未编译的 universe 包，容器内存/时间限制 | 改用 `run_sim.py` 直接启动各组件 |

---

### 3. 运行

#### 一键启动脚本：`run_sim.py`

**路径**：`/home/liu/github/autoware/run_sim.py`

**功能**：启动以下组件并协调运行

| 组件 | 启动方式 | 说明 |
|------|---------|------|
| 地图加载器 | `ros2 run autoware_map_loader` | 加载测试地图（`LocalCartesianUTM` 投影） |
| 输入发布器 | Python `rclpy` 节点 | 发布 route / odometry / acceleration / tracked_objects / traffic_signals / turn_indicators |
| diffusion_planner | `ros2 launch` | 加载模型，规划轨迹 |
| rviz2 | `rviz2 -d` | 可视化显示 |

#### 输入话题

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `~/input/odometry` | `nav_msgs/Odometry` | 10 Hz | 车辆沿路线移动 |
| `~/input/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` | 10 Hz | 空 |
| `~/input/route` | `autoware_planning_msgs/LaneletRoute` | 1次（transient_local） | 24 个 lanelets，497.6m |
| `~/input/vector_map` | `autoware_map_msgs/LaneletMapBin` | 1次（transient_local） | 183 个 road lanelets |
| `~/input/tracked_objects` | `autoware_perception_msgs/TrackedObjects` | 10 Hz | 4 个障碍物 |
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

#### 障碍物（固定位置，路线旁边）

| 名称 | 位置（progress） | 横向偏移 | 速度 | 尺寸 | 颜色 |
|------|-----------------|---------|------|------|------|
| parked_car_1 | 5% | 偏右 0.5m | 0 m/s | 4.0×2.0×1.5m | 红色 |
| parked_car_2 | 9% | 偏左 0.5m | 0 m/s | 4.0×2.0×1.5m | 蓝色 |
| roadside_cone | 7% | 偏右 1.0m | 0 m/s | 0.5×0.5×1.0m | 橙色 |
| pedestrian | 12% | 偏右 1.5m | 0.5 m/s | 0.5×0.5×1.0m | 绿色 |

#### 车辆运动

- 速度：3.0 m/s
- 沿路线循环行驶（progress 0% → 95% → 0%）

---

### 5. 关键技术问题与解决

#### 5.1 坐标系对齐

**问题**：`MapProjectorInfo` 使用 `MGRS` 投影时，地图坐标在 MGRS 网格坐标系（~3811, 73803），而路线在 UTM 坐标系（~130, 130），rviz2 中无法重叠显示。

**解决**：改用 `LocalCartesianUTM` 投影，地图和路线使用相同坐标系。

#### 5.2 调试标记持续显示

**问题**：`publish_debug_markers` 中 markers 的 `lifetime = 0.2s`，而规划频率 10 Hz，但 `on_timer()` 在数据不可用时提前返回，导致 markers 过期消失。

**解决**：`run_sim.py` 以 6.7 Hz 订阅并重新发布所有 debug markers（route_marker、lane_marker、linestring_marker），覆盖 0.2s 的 lifetime。

#### 5.3 地图可视化

**问题**：`LaneletMapBin` 无法直接在 rviz2 中显示。

**解决**：`run_sim.py` 使用 lanelet2 Python 库将 183 个 lanelets 转换为 `MarkerArray`（lanelet 边界 + 中心线），每 2 秒用 `transient_local` QoS 发布到 `/vector_map_marker`。

#### 5.4 InterProcessPollingSubscriber 兼容性

**问题**：`Latest` 策略的 `take_data()` 只调用一次 `take()`，当队列深度为 1 时，若 `take()` 在发布者发送前调用，`data_` 始终为 `nullptr`，导致 `TrackedObjects` 无法接收。

**解决**：修改 `autoware_utils_rclcpp/polling_subscriber.hpp`，将 `Latest::take_data()` 改为循环 drain 队列，确保取到最新消息。

#### 5.5 残留进程

**问题**：`Ctrl+C` 退出脚本后，子进程（map_loader、diffusion_planner_node、rviz2）残留。

**解决**：`cleanup()` 函数依次执行：
1. `os.killpg` 发 SIGTERM
2. 等待 3s
3. `proc.kill()` 发 SIGKILL
4. `pkill -9 -f` 按名称清理

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

**路径**：`/home/aw/.rviz2/diffusion_planner.rviz`

包含 5 个显示层：

| 显示层 | 插件类型 | 话题 |
|--------|---------|------|
| Trajectory | `rviz_plugins/Trajectory` | `/diffusion_planner_node/output/trajectory` |
| MapMarkers | `rviz_default_plugins/MarkerArray` | `/vector_map_marker` |
| RouteMarker | `rviz_default_plugins/MarkerArray` | `/diffusion_planner_node/debug/route_marker` |
| LaneMarker | `rviz_default_plugins/MarkerArray` | `/diffusion_planner_node/debug/lane_marker` |
| LinestringMarker | `rviz_default_plugins/MarkerArray` | `/diffusion_planner_node/debug/linestring_marker` |

---

### 7. 启动命令汇总

```bash
# 进入容器
docker exec -it autoware_diffusion_planner bash

# 启动仿真
source /opt/autoware/setup.bash
source /home/aw/autoware/install/setup.bash
python3 /home/aw/autoware/run_sim.py

# 单独启动 rviz2（如果脚本已启动）
rviz2 -d /home/aw/.rviz2/diffusion_planner.rviz

# 验证轨迹
source /opt/autoware/setup.bash
source /home/aw/autoware/install/setup.bash
python3 /home/aw/autoware/validate_trajectory.py

# 停止所有进程
docker exec autoware_diffusion_planner bash -c \
  "pkill -f run_sim; pkill -f diffusion_planner_node; pkill -f 'map_loader'; pkill -f rviz2"
```

---

### 8. 已知限制

1. **`autoware_launch` 未编译**：`planning_simulator` 完整仿真需要编译 `autoware_launch` 及其依赖链（约 100+ 包），因容器内存限制未完成。当前使用 `run_sim.py` 直接启动各组件。

2. **`autoware_tensorrt_plugins` 未完全编译**：spconv 不可用，插件库 `libautoware_tensorrt_plugins.so` 未生成，但不影响模型推理（仅影响 TensorRT 自定义算子插件）。

3. **障碍物避让效果待验证**：planner 已接收障碍物数据，但轨迹是否绕行需在 rviz2 中观察。如果模型训练数据不包含避让场景，可尝试调整 `temperature` 参数增加轨迹多样性。

4. **测试地图非真实场景**：使用的 `test_map/lanelet2_map.osm` 为单元测试用地图，非真实自动驾驶场景地图，路线长度和复杂度有限。

5. **无点云地图**：`planning_simulator` 需要 `.pcd` 点云地图，当前测试地图仅有 `.osm` 文件，不影响规划器但会影响地图加载器的完整性检查。