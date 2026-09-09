# Autoware 感知模块介绍

## 目录

1. [3D目标检测](#3d目标检测)
2. [2D目标检测与分类](#2d目标检测与分类)
3. [交通灯感知](#交通灯感知)
4. [多目标跟踪](#多目标跟踪)
5. [目标处理与融合](#目标处理与融合)
6. [点云处理](#点云处理)
7. [占据栅格地图](#占据栅格地图)
8. [预测](#预测)
9. [其他工具](#其他工具)

---

## 3D目标检测

### autoware_lidar_centerpoint
- **用途**: 基于LiDAR的3D动态目标检测
- **算法**: CenterPoint + PointPillars + TensorRT
- **输入**: PointCloud2
- **输出**: DetectedObjects
- **关键参数**: encoder_onnx_path, head_onnx_path, trt_precision

### autoware_bevfusion
- **用途**: 基于LiDAR或Camera-LiDAR融合的3D目标检测
- **算法**: BEVFusion + TensorRT + spconv
- **输入**: PointCloud2, Image, CameraInfo
- **输出**: DetectedObjects
- **关键参数**: lidar_only, use_detection_class_remapping

### autoware_lidar_transfusion
- **用途**: 基于LiDAR数据的3D目标检测
- **算法**: TransFusion + TensorRT
- **输入**: PointCloud2
- **输出**: DetectedObjects
- **关键参数**: pointcloud_range, voxel_size

### autoware_lidar_frnet
- **用途**: 基于LiDAR的3D语义分割
- **算法**: FRNet + TensorRT
- **输入**: PointCloud2
- **输出**: 分割点云、可视化点云、过滤点云
- **关键参数**: filter.output_format

### autoware_tensorrt_bevdet
- **用途**: 基于多视角图像的3D目标检测
- **算法**: BEVDet + TensorRT
- **输入**: 6个相机图像和CameraInfo
- **输出**: DetectedObjects
- **关键参数**: precision, debug_mode

### autoware_tensorrt_bevformer
- **用途**: 基于多视角图像的3D目标检测（带时序融合）
- **算法**: BEVFormer + TensorRT
- **输入**: 6个相机图像、CameraInfo、CAN总线数据
- **输出**: DetectedObjects
- **关键参数**: precision, debug_mode

### autoware_ptv3
- **用途**: 3D LiDAR分割
- **算法**: Point Transformers V3 + TensorRT + spconv
- **输入**: PointCloud2
- **输出**: 分割点云、可视化点云、过滤点云、检测对象
- **关键参数**: filter.output_format, voxels_num

---

## 2D目标检测与分类

### autoware_tensorrt_yolox
- **用途**: 基于图像的目标检测和语义分割
- **算法**: YOLOX + TensorRT
- **输入**: Image
- **输出**: 2D边界框、分割掩码
- **关键参数**: label_file, precision

### autoware_tensorrt_classifier
- **用途**: 使用TensorRT进行高效动态批量推理的分类
- **算法**: TensorRT
- **输入**: 图像
- **输出**: 分类结果
- **关键参数**: 支持GPU和DLA推理

### autoware_tensorrt_common
- **用途**: TensorRT通用工具库
- **算法**: TensorRT
- **输入**: N/A（库文件）
- **输出**: N/A（库文件）
- **关键参数**: N/A

### autoware_tensorrt_plugins
- **用途**: TensorRT插件
- **算法**: TensorRT
- **输入**: N/A（插件）
- **输出**: N/A（插件）
- **关键参数**: N/A

---

## 交通灯感知

### autoware_traffic_light_classifier
- **用途**: 交通灯标签分类
- **算法**: CNN分类器（EfficientNet-b1/MobileNet-v2）或HSV分类器
- **输入**: Image, TrafficLightRoiArray
- **输出**: TrafficLightArray
- **关键参数**: classifier_type, model_file_path

### autoware_traffic_light_fine_detector
- **用途**: 使用YOLOX-s进行交通灯检测
- **算法**: YOLOX-s
- **输入**: Image, TrafficLightRoiArray
- **输出**: TrafficLightRoiArray
- **关键参数**: model_file_path

### autoware_traffic_light_map_based_detector
- **用途**: 基于HD地图计算交通灯在图像中的位置
- **算法**: 地图投影
- **输入**: LaneletMapBin, CameraInfo, LaneletRoute
- **输出**: TrafficLightRoiArray
- **关键参数**: max_detection_range

### autoware_traffic_light_multi_camera_fusion
- **用途**: 融合多个摄像头的交通灯识别结果
- **算法**: 贝叶斯更新
- **输入**: 多个摄像头的CameraInfo、检测ROI、分类结果
- **输出**: TrafficLightGroupArray
- **关键参数**: camera_namespaces, signal_consistency_check

### autoware_traffic_light_occlusion_predictor
- **用途**: 使用点云计算交通灯的遮挡率
- **算法**: 点云投影
- **输入**: LaneletMapBin, TrafficLightArray, TrafficLightRoiArray, CameraInfo, PointCloud2
- **输出**: TrafficLightArray
- **关键参数**: N/A

### autoware_traffic_light_selector
- **用途**: 从精确检测的交通灯列表中选择感兴趣的交通灯
- **算法**: ROI匹配
- **输入**: DetectedObjectsWithFeature, TrafficLightRoiArray
- **输出**: TrafficLightRoiArray
- **关键参数**: N/A

### autoware_traffic_light_arbiter
- **用途**: 合并感知和外部（如V2X）的交通信号
- **算法**: 置信度或外部偏好方法
- **输入**: LaneletMapBin, TrafficLightGroupArray（感知）, TrafficLightGroupArray（外部）
- **输出**: TrafficLightGroupArray
- **关键参数**: enable_signal_matching, source_priority

### autoware_traffic_light_category_merger
- **用途**: 合并车辆和行人交通灯的分类结果
- **算法**: 简单合并
- **输入**: TrafficLightArray（车辆）, TrafficLightArray（行人）
- **输出**: TrafficLightArray
- **关键参数**: N/A

### autoware_traffic_light_visualization
- **用途**: 交通灯可视化
- **算法**: 标记可视化
- **输入**: TrafficLightGroupArray, LaneletMapBin, Image, TrafficLightRoiArray
- **输出**: MarkerArray, Image
- **关键参数**: N/A

### autoware_crosswalk_traffic_light_estimator
- **用途**: 估计行人交通信号
- **算法**: 基于地图和车辆信号的估计
- **输入**: LaneletMapBin, TrafficLightGroupArray
- **输出**: TrafficLightGroupArray
- **关键参数**: use_last_detect_color, use_pedestrian_signal_detect

---

## 多目标跟踪

### autoware_multi_object_tracker
- **用途**: 多目标跟踪，分配ID并估计速度
- **算法**: 数据关联 + EKF
- **输入**: 多个检测输入（可配置）
- **输出**: TrackedObjects
- **关键参数**: input/detection**, tracking_config_directory

### autoware_bytetrack
- **用途**: 多目标跟踪
- **算法**: ByteTrack + Kalman滤波
- **输入**: DetectedObjectsWithFeature
- **输出**: DetectedObjectsWithFeature
- **关键参数**: track_buffer_length

### autoware_radar_object_tracker
- **用途**: 雷达目标跟踪
- **算法**: 数据关联 + 跟踪模型
- **输入**: DetectedObjects, LaneletMapBin
- **输出**: TrackedObjects
- **关键参数**: publish_rate, world_frame_id

### autoware_detection_by_tracker
- **用途**: 将跟踪对象反馈到检测模块以保持稳定性
- **算法**: 形状拟合 + 跟踪器信息
- **输入**: DetectedObjectsWithFeature, TrackedObjects
- **输出**: DetectedObjects
- **关键参数**: tracker_ignore_label.*

---

## 目标处理与融合

### autoware_cluster_merger
- **用途**: 合并点云聚类作为检测对象
- **算法**: 简单连接
- **输入**: DetectedObjectsWithFeature（多个）
- **输出**: DetectedObjectsWithFeature
- **关键参数**: N/A

### autoware_object_merger
- **用途**: 通过数据关联合并两个检测方法的结果
- **算法**: 连续最短路径算法
- **输入**: DetectedObjects（两个）
- **输出**: DetectedObjects
- **关键参数**: distance_threshold_list, precision_threshold_to_judge_overlapped

### autoware_simple_object_merger
- **用途**: 低计算成本合并多个检测对象话题
- **算法**: 简单合并（无数据关联）
- **输入**: DetectedObjects（多个）
- **输出**: DetectedObjects
- **关键参数**: input_topics, timeout_sec

### autoware_tracking_object_merger
- **用途**: 合并来自不同传感器的跟踪对象
- **算法**: 数据关联 + 状态融合
- **输入**: TrackedObjects（主）, TrackedObjects（子）
- **输出**: TrackedObjects
- **关键参数**: main_sensor_type, sub_sensor_type

### autoware_object_sorter
- **用途**: 基于距离和速度过滤对象
- **算法**: 范围和速度过滤
- **输入**: DetectedObjects或TrackedObjects
- **输出**: DetectedObjects或TrackedObjects
- **关键参数**: N/A

### autoware_image_projection_based_fusion
- **用途**: 融合图像和LiDAR感知信息
- **算法**: 多种融合算法（roi_cluster_fusion, roi_detected_object_fusion, pointpainting_fusion等）
- **输入**: 3D数据（点云/边界框）, 2D RoIs
- **输出**: 融合后的对象
- **关键参数**: matching_strategy, rois_timestamp_offsets

### autoware_radar_fusion_to_detected_object
- **用途**: 雷达检测对象与3D检测对象的传感器融合
- **算法**: 速度附加和置信度改进
- **输入**: DetectedObjects（3D）, DetectedObjects（雷达）
- **输出**: DetectedObjects（带速度）
- **关键参数**: bounding_box_margin, threshold_yaw_diff

---

## 点云处理

### autoware_euclidean_cluster
- **用途**: 将点聚类成更小的部分以分类对象
- **算法**: 欧几里得聚类、体素网格聚类、基于标签的聚类
- **输入**: PointCloud2
- **输出**: DetectedObjectsWithFeature
- **关键参数**: cluster_tolerance, min_size, max_size

### autoware_ground_segmentation
- **用途**: 从输入点云中移除地面点
- **算法**: 射线地面滤波、扫描地面滤波、RANSAC地面滤波
- **输入**: PointCloud2, Indices
- **输出**: PointCloud2
- **关键参数**: input_frame, output_frame

### autoware_ground_segmentation_cuda
- **用途**: 使用CUDA加速的地面分割
- **算法**: 扫描地面滤波（CUDA实现）
- **输入**: PointCloud2
- **输出**: PointCloud2
- **关键参数**: N/A（参考autoware_ground_segmentation）

### autoware_compare_map_segmentation
- **用途**: 使用地图信息过滤地面点
- **算法**: 比较高程图、距离比较、体素比较等
- **输入**: PointCloud2, ElevationMap/LaneletMapBin
- **输出**: PointCloud2
- **关键参数**: height_diff_thresh, distance_threshold

### autoware_raindrop_cluster_filter
- **用途**: 雨滴聚类过滤器（README未找到）
- **算法**: N/A
- **输入**: N/A
- **输出**: N/A
- **关键参数**: N/A

---

## 占据栅格地图

### autoware_probabilistic_occupancy_grid_map
- **用途**: 输出障碍物概率作为占据栅格地图
- **算法**: 二值贝叶斯滤波器
- **输入**: PointCloud2/LaserScan
- **输出**: OccupancyGrid
- **关键参数**: grid_map.*, laserscan_based_occupancy_grid_map.*

### autoware_occupancy_grid_map_outlier_filter
- **用途**: 基于占据栅格地图的异常值过滤器
- **算法**: 占据概率分离 + 半径搜索2D过滤
- **输入**: PointCloud2, OccupancyGrid
- **输出**: PointCloud2
- **关键参数**: use_radius_search_2d_filter, search_radius

---

## 预测

### autoware_map_based_prediction
- **用途**: 根据地图和周围环境预测其他车辆和行人的未来路径
- **算法**: 基于地图的路径预测
- **输入**: TrackedObjects, LaneletMapBin, TrafficLightGroupArray
- **输出**: PredictedObjects
- **关键参数**: prediction_time_horizon, lateral_control_time_horizon

### autoware_predicted_path_postprocessor
- **用途**: 对预测路径进行后处理
- **算法**: RefineBySpeed, RefinePenetrationByStaticObjects
- **输入**: PredictedObjects, LaneletMapBin
- **输出**: PredictedObjects
- **关键参数**: processors

### autoware_simpl_prediction
- **用途**: 基于ML模型的3D目标运动预测
- **算法**: SIMPL + TensorRT
- **输入**: TrackedObjects, LaneletMapBin, Odometry
- **输出**: PredictedObjects
- **关键参数**: preprocess.labels, preprocess.max_num_agent

---

## 其他工具

### autoware_detected_object_feature_remover
- **用途**: 将DetectedObjectWithFeatureArray转换为DetectedObjects
- **算法**: 类型转换
- **输入**: DetectedObjectWithFeatureArray
- **输出**: DetectedObjects
- **关键参数**: N/A

### autoware_detected_object_validation
- **用途**: 消除明显的假阳性检测对象
- **算法**: 多种验证器（障碍点云、占据网格、对象车道过滤等）
- **输入**: DetectedObjects/TrackedObjects
- **输出**: 过滤后的对象
- **关键参数**: 各验证器参数

### autoware_image_object_locator
- **用途**: 使用2D图像检测的对象生成3D对象检测
- **算法**: 边界框对象定位器
- **输入**: 2D图像检测
- **输出**: 3D对象检测
- **关键参数**: N/A

### autoware_elevation_map_loader
- **用途**: 为autoware_compare_map_segmentation提供高程图
- **算法**: 从点云地图和向量地图生成高程图
- **输入**: PointCloud2, LaneletMapBin
- **输出**: GridMap
- **关键参数**: map_layer_name, elevation_map_directory

### autoware_lidar_apollo_instance_segmentation
- **用途**: 基于CNN模型和障碍聚类方法分割3D点云数据
- **算法**: Apollo CNN分割 + TensorRT
- **输入**: PointCloud2
- **输出**: DetectedObjectsWithFeature
- **关键参数**: N/A（预训练模型）

### autoware_radar_tracks_msgs_converter
- **用途**: 将radar_msgs转换为autoware消息格式
- **算法**: 消息格式转换
- **输入**: RadarTracks, Odometry
- **输出**: DetectedObjects, TrackedObjects
- **关键参数**: update_rate_hz, new_frame_id

### perception_utils
- **用途**: 感知模块通用函数库
- **算法**: N/A（库文件）
- **输入**: N/A（库文件）
- **输出**: N/A（库文件）
- **关键参数**: N/A

---

## 附录：模块依赖关系

### 检测流程
1. 传感器输入（LiDAR/Camera/Radar）
2. 预处理（地面分割、点云过滤）
3. 3D/2D检测（CenterPoint, BEVFusion, YOLOX等）
4. 目标验证和过滤
5. 多目标跟踪

### 交通灯流程
1. 地图-based检测
2. 精细检测
3. 分类
4. 多摄像头融合
5. 遮挡预测
6. 仲裁（感知+V2X）

### 预测流程
1. 跟踪对象输入
2. 基于地图的预测
3. 后处理
4. 输出预测路径

---

*文档生成时间: 2026-09-09*
*基于Autoware Universe Perception模块*
