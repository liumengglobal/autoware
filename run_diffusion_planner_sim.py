#!/usr/bin/env python3
"""
Minimal planning simulator for diffusion_planner with longer route + moving vehicle.
"""

import subprocess
import threading
import time
import os
import signal
import math
import sys

import lanelet2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from autoware_planning_msgs.msg import LaneletRoute, LaneletSegment, LaneletPrimitive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    AccelWithCovarianceStamped, Pose, Point, Quaternion,
    TwistWithCovariance, PoseWithCovariance,
)
from autoware_perception_msgs.msg import TrackedObjects, TrafficLightGroupArray, TrackedObject, ObjectClassification, Shape
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from autoware_map_msgs.msg import MapProjectorInfo
from autoware_planning_msgs.msg import Trajectory
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from unique_identifier_msgs.msg import UUID
from uuid import uuid4

from lanelet2.io import Origin
from lanelet2.projection import UtmProjector
from lanelet2.routing import RoutingGraph
import autoware_lanelet2_extension_python.regulatory_elements
import autoware_lanelet2_extension_python.projection
from autoware_lanelet2_extension_python.utility.query import roadLanelets

MAP_PATH = "/home/aw/autoware/src/universe/autoware_universe/planning/autoware_diffusion_planner/test_map/lanelet2_map.osm"
MODEL_DATA_PATH = "/home/aw/autoware_data/ml_models/diffusion_planner"
ORIGIN_LAT = 35.902
ORIGIN_LON = 139.932
MGRS_GRID = "54SVE036736"

procs = []


def start_process(cmd, name):
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    procs.append(proc)

    def print_output():
        for line in proc.stdout:
            print(f"[{name}] {line.decode().strip()}", flush=True)

    t = threading.Thread(target=print_output, daemon=True)
    t.start()
    return proc


def cleanup():
    # Kill child processes by process group, then force kill
    for proc in reversed(procs):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except:
            pass
    for proc in reversed(procs):
        try:
            proc.wait(timeout=3)
        except:
            pass
    # Force kill any remaining
    for proc in reversed(procs):
        try:
            proc.kill()
            proc.wait(timeout=1)
        except:
            pass
    # Fallback: pkill any leftover processes by name
    for name in ["diffusion_planner_node", "lanelet2_map_loader",
                  "map_hash_generator", "rviz2"]:
        try:
            subprocess.run(["pkill", "-9", "-f", name],
                           capture_output=True, timeout=3)
        except:
            pass


class SimNode(Node):
    def __init__(self):
        super().__init__("sim_orchestrator")

        self.load_map_and_route()

        transient_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.projector_pub = self.create_publisher(
            MapProjectorInfo, "/map/map_projector_info", transient_qos)
        self.route_pub = self.create_publisher(
            LaneletRoute, "/diffusion_planner_node/input/route", transient_qos)
        self.odom_pub = self.create_publisher(
            Odometry, "/diffusion_planner_node/input/odometry", 1)
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, "/diffusion_planner_node/input/acceleration", 1)
        self.objects_pub = self.create_publisher(
            TrackedObjects, "/diffusion_planner_node/input/tracked_objects", 1)
        self.traffic_pub = self.create_publisher(
            TrafficLightGroupArray, "/diffusion_planner_node/input/traffic_signals", 1)
        self.turn_pub = self.create_publisher(
            TurnIndicatorsReport, "/diffusion_planner_node/input/turn_indicators", 1)
        self.traj_sub = self.create_subscription(
            Trajectory, "/diffusion_planner_node/output/trajectory",
            self.on_trajectory, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/vector_map_marker", transient_qos)

        # Debug markers: diffusion_planner publishes with 0.2s lifetime,
        # so we re-publish at 5Hz (0.2s) to keep them alive
        self.debug_markers = {}
        for name in ["route_marker", "lane_marker", "linestring_marker"]:
            topic = f"/diffusion_planner_node/debug/{name}"
            self.debug_markers[name] = None
            setattr(self, f"sub_{name}",
                    self.create_subscription(MarkerArray, topic,
                        lambda msg, n=name: self.on_debug_marker(n, msg), 10))
            setattr(self, f"pub_{name}",
                    self.create_publisher(MarkerArray, topic, 10))

        self.progress = 0.0
        self.speed = 3.0
        self.obstacles = self.init_obstacles()

    def load_map_and_route(self):
        origin = Origin(ORIGIN_LAT, ORIGIN_LON)
        projector = UtmProjector(origin)
        self.lanelet_map = lanelet2.io.load(MAP_PATH, projector)

        road = list(roadLanelets(list(self.lanelet_map.laneletLayer)))
        self.get_logger().info(f"Loaded map: {len(road)} road lanelets")

        # Build routing graph and find longest path
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle)
        graph = RoutingGraph(self.lanelet_map, traffic_rules)

        longest = []
        for start in road[:30]:
            path = [start]
            current = start
            for _ in range(30):
                next_ls = graph.following(current, False)
                if not next_ls:
                    break
                current = next_ls[0]
                path.append(current)
            if len(path) > len(longest):
                longest = path

        self.route_lanelets = longest
        self.get_logger().info(
            f"Route: {len(longest)} lanelets, "
            f"IDs: {[l.id for l in longest[:5]]}...{[l.id for l in longest[-3:]]}")

        # Build full centerline waypoints for the entire route
        self.waypoints = []
        for l in longest:
            for pt in l.centerline:
                self.waypoints.append((pt.x, pt.y, pt.z))
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")

        # Calculate total route length
        self.route_length = 0.0
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i][0] - self.waypoints[i-1][0]
            dy = self.waypoints[i][1] - self.waypoints[i-1][1]
            self.route_length += math.sqrt(dx*dx + dy*dy)
        self.get_logger().info(f"Total route length: {self.route_length:.1f}m")

        # Create route message with all lanelets
        self.route_msg = LaneletRoute()
        self.route_msg.header = Header(frame_id="map")
        self.route_msg.uuid = UUID(uuid=[0] * 16)
        self.route_msg.allow_modification = False

        start_pt = self.waypoints[0]
        end_pt = self.waypoints[-1]
        self.route_msg.start_pose = Pose(
            position=Point(x=start_pt[0], y=start_pt[1], z=start_pt[2]),
            orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        self.route_msg.goal_pose = Pose(
            position=Point(x=end_pt[0], y=end_pt[1], z=end_pt[2]),
            orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))

        segments = []
        for l in longest:
            seg = LaneletSegment()
            seg.preferred_primitive.id = l.id
            seg.preferred_primitive.primitive_type = "lane"
            prim = LaneletPrimitive()
            prim.id = l.id
            prim.primitive_type = "lane"
            seg.primitives = [prim]
            segments.append(seg)
        self.route_msg.segments = segments

        self.get_logger().info(
            f"Route: start=({start_pt[0]:.1f},{start_pt[1]:.1f}), "
            f"goal=({end_pt[0]:.1f},{end_pt[1]:.1f})")

    def get_position_on_route(self, progress):
        """Get interpolated position along route (progress: 0~1)."""
        target_dist = progress * self.route_length
        accumulated = 0.0
        for i in range(1, len(self.waypoints)):
            dx = self.waypoints[i][0] - self.waypoints[i-1][0]
            dy = self.waypoints[i][1] - self.waypoints[i-1][1]
            seg_len = math.sqrt(dx*dx + dy*dy)
            if accumulated + seg_len >= target_dist:
                t = (target_dist - accumulated) / seg_len if seg_len > 0 else 0
                x = self.waypoints[i-1][0] + t * dx
                y = self.waypoints[i-1][1] + t * dy
                z = self.waypoints[i-1][2] + t * (self.waypoints[i][2] - self.waypoints[i-1][2])
                yaw = math.atan2(dy, dx)
                return (x, y, z, yaw)
            accumulated += seg_len
        return (self.waypoints[-1][0], self.waypoints[-1][1], self.waypoints[-1][2], 0.0)

    def get_position_by_offset(self, base_progress, offset_m):
        offset_progress = offset_m / self.route_length
        return self.get_position_on_route(max(0.0, min(1.0, base_progress + offset_progress)))

    def init_obstacles(self):
        obstacles = []
        # Fixed positions along the route (progress 0~1)
        configs = [
            {"name": "parked_car_1", "progress": 0.05, "speed": 0.0, "lateral": 0.5, "label": ObjectClassification.CAR},
        ]
        for cfg in configs:
            uid = [int(b) for b in uuid4().bytes]
            p = self.get_position_on_route(cfg["progress"])
            yaw = p[3]
            lat = cfg["lateral"]
            fx = p[0] + lat * math.cos(yaw + math.pi/2)
            fy = p[1] + lat * math.sin(yaw + math.pi/2)
            obstacles.append({
                **cfg,
                "fixed_x": fx,
                "fixed_y": fy,
                "fixed_yaw": yaw,
                "uuid": uid,
                "shape": (4.0, 2.0, 1.5) if cfg["label"] == ObjectClassification.CAR else (0.5, 0.5, 1.0),
            })
        return obstacles

    def create_obstacle(self, cfg, now):
        obj = TrackedObject()
        obj.object_id.uuid = cfg["uuid"]
        obj.kinematics.pose_with_covariance.pose.position.x = cfg["fixed_x"]
        obj.kinematics.pose_with_covariance.pose.position.y = cfg["fixed_y"]
        obj.kinematics.pose_with_covariance.pose.position.z = 0.0
        qz = math.sin(cfg["fixed_yaw"] / 2)
        qw = math.cos(cfg["fixed_yaw"] / 2)
        obj.kinematics.pose_with_covariance.pose.orientation.w = qw
        obj.kinematics.pose_with_covariance.pose.orientation.z = qz
        obj.kinematics.twist_with_covariance.twist.linear.x = cfg["speed"]
        obj.kinematics.twist_with_covariance.twist.linear.y = 0.0
        obj.shape.type = Shape.BOUNDING_BOX
        obj.shape.dimensions.x = cfg["shape"][0]
        obj.shape.dimensions.y = cfg["shape"][1]
        obj.shape.dimensions.z = cfg["shape"][2]
        obj.existence_probability = 0.9
        cls = ObjectClassification()
        cls.label = cfg["label"]
        cls.probability = 0.95
        obj.classification.append(cls)
        return obj
        obj.shape.dimensions.y = cfg["shape"][1]
        obj.shape.dimensions.z = cfg["shape"][2]
        obj.existence_probability = 0.9
        cls = ObjectClassification()
        cls.label = cfg["label"]
        cls.probability = 0.95
        obj.classification.append(cls)
        return obj

    def publish_route(self):
        self.route_pub.publish(self.route_msg)
        self.get_logger().info("Route published")

    def publish_inputs(self):
        dt = 0.1
        self.progress += self.speed * dt / self.route_length
        if self.progress > 0.95:
            self.progress = 0.0  # loop back

        pos = self.get_position_on_route(self.progress)
        now = self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header = Header(stamp=now, frame_id="map")
        qz = math.sin(pos[3] / 2)
        qw = math.cos(pos[3] / 2)
        odom.pose = PoseWithCovariance(
            pose=Pose(
                position=Point(x=pos[0], y=pos[1], z=pos[2]),
                orientation=Quaternion(w=qw, x=0.0, y=0.0, z=qz)))
        odom.twist = TwistWithCovariance()
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)

        # Acceleration
        accel = AccelWithCovarianceStamped()
        accel.header = Header(stamp=now, frame_id="map")
        self.accel_pub.publish(accel)

        # Tracked objects with obstacles
        objects = TrackedObjects()
        objects.header = Header(stamp=now, frame_id="map")
        for cfg in self.obstacles:
            objects.objects.append(self.create_obstacle(cfg, now))
        self.objects_pub.publish(objects)

        # Empty traffic signals
        traffic = TrafficLightGroupArray()
        traffic.stamp = now
        self.traffic_pub.publish(traffic)

        # Turn indicators
        turn = TurnIndicatorsReport()
        turn.stamp = now
        turn.report = TurnIndicatorsReport.DISABLE
        self.turn_pub.publish(turn)

    def on_trajectory(self, msg):
        pts = msg.points
        total_len = 0.0
        for i in range(1, len(pts)):
            a = pts[i-1].pose.position
            b = pts[i].pose.position
            dx = b.x - a.x; dy = b.y - a.y; dz = b.z - a.z
            total_len += math.sqrt(dx*dx + dy*dy + dz*dz)
        speeds = [abs(p.longitudinal_velocity_mps) for p in pts]
        avg_speed = sum(speeds) / len(speeds) if speeds else 0
        self.get_logger().info(
            f"Trajectory: {len(pts)} pts, {total_len:.1f}m, "
            f"avg_speed={avg_speed:.1f}m/s, "
            f"progress={self.progress*100:.0f}%")

    def on_debug_marker(self, name, msg):
        self.debug_markers[name] = msg

    def republish_debug_markers(self):
        for name, msg in self.debug_markers.items():
            if msg is not None:
                getattr(self, f"pub_{name}").publish(msg)

    def publish_all_markers(self):
        markers = []
        mid = 0
        # Lanelet bounds and centerlines
        for l in self.lanelet_map.laneletLayer:
            for bound, color in [(l.leftBound, (0.8, 0.8, 0.8)),
                                 (l.rightBound, (0.6, 0.6, 0.6))]:
                pts = [Point(x=p.x, y=p.y, z=p.z) for p in bound]
                if len(pts) < 2:
                    continue
                m = Marker()
                m.header.frame_id = "map"
                m.ns = "bounds"; m.id = mid; mid += 1
                m.type = Marker.LINE_STRIP; m.action = Marker.ADD
                m.points = pts; m.scale.x = 0.08
                m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.6)
                markers.append(m)
            c = l.centerline
            pts = [Point(x=p.x, y=p.y, z=p.z) for p in c]
            if len(pts) < 2:
                continue
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "centerlines"; m.id = mid; mid += 1
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.points = pts; m.scale.x = 0.04
            m.color = ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.4)
            markers.append(m)
        # Obstacle markers
        now = self.get_clock().now().to_msg()
        colors = [(0.8, 0.2, 0.2), (0.2, 0.5, 0.8), (1.0, 0.6, 0.0), (0.2, 0.8, 0.2)]
        for i, cfg in enumerate(self.obstacles):
            obj = self.create_obstacle(cfg, now)
            p = obj.kinematics.pose_with_covariance.pose.position
            c = colors[i % len(colors)]
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "obstacles"; m.id = mid; mid += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = 1.0
            m.pose.orientation.w = 1.0
            m.scale.x = obj.shape.dimensions.x
            m.scale.y = obj.shape.dimensions.y
            m.scale.z = obj.shape.dimensions.z
            m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.7)
            markers.append(m)
            t = Marker()
            t.header.frame_id = "map"
            t.ns = "obstacle_labels"; t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = p.x; t.pose.position.y = p.y; t.pose.position.z = 3.0
            t.scale.z = 0.8
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = cfg["name"]
            markers.append(t)
        self.marker_pub.publish(MarkerArray(markers=markers))


def signal_handler(sig, frame):
    print(f"\nReceived signal {sig}, shutting down...")
    try:
        rclpy.shutdown()
    except:
        pass
    cleanup()
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    print("=" * 60)
    print("Starting diffusion_planner simulation (long route + moving vehicle)")
    print("=" * 60)

    # 1. Map loader
    print("\n[1/5] Map loader...")
    start_process([
        "ros2", "run", "autoware_map_loader", "autoware_lanelet2_map_loader",
        "--ros-args",
        "-p", f"lanelet2_map_path:={MAP_PATH}",
        "-p", "allow_unsupported_version:=true",
        "-p", "center_line_resolution:=5.0",
        "-p", "use_waypoints:=true",
        "-r", "/map/vector_map:=/map/vector_map",
    ], "map_loader")
    time.sleep(1.0)

    # 2. No map visualizer - run_sim directly publishes lanelet markers
    print("\n[2/5] Map visualizer (built-in)...")
    time.sleep(1.0)

    # 3. Orchestrator
    print("\n[3/5] Starting publisher...")
    rclpy.init()
    node = SimNode()

    projector_msg = MapProjectorInfo()
    projector_msg.projector_type = "LocalCartesianUTM"
    projector_msg.mgrs_grid = ""
    projector_msg.vertical_datum = "WGS84"
    projector_msg.map_origin.latitude = ORIGIN_LAT
    projector_msg.map_origin.longitude = ORIGIN_LON
    projector_msg.scale_factor = 0.9996
    node.projector_pub.publish(projector_msg)
    print("   MapProjectorInfo published")
    time.sleep(2.0)

    node.publish_route()
    time.sleep(0.5)

    timer = node.create_timer(0.1, node.publish_inputs)
    marker_timer = node.create_timer(2.0, node.publish_all_markers)
    debug_timer = node.create_timer(0.15, node.republish_debug_markers)
    print("   Input publisher started (10Hz)")

    # 4. Diffusion planner
    print("\n[4/5] Diffusion planner...")
    start_process([
        "ros2", "launch", "autoware_diffusion_planner",
        "diffusion_planner.launch.xml",
        f"data_path:={MODEL_DATA_PATH}",
        "input_vector_map:=/map/vector_map",
    ], "diffusion_planner")
    time.sleep(2.0)

    # 5. RViz2
    print("\n[5/5] RViz2...")
    start_process([
        "rviz2", "-d", "/home/aw/.rviz2/diffusion_planner.rviz",
    ], "rviz2")

    print("\n" + "=" * 60)
    print("All components running. Press Ctrl+C to stop.")
    print("=" * 60)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup()


if __name__ == "__main__":
    main()