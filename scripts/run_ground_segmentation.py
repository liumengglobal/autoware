#!/usr/bin/env python3
"""
Test script for autoware_ground_segmentation module.
Supports loading PCD files and manual point cloud generation.
Displays ground segmentation results in RViz.
"""

import subprocess
import threading
import time
import os
import signal
import sys
import math
import struct
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# Configuration
WORKSPACE_DIR = "/home/aw/autoware"
RVIZ_CONFIG = os.path.join(WORKSPACE_DIR, "scripts", "ground_segmentation.rviz")
PIDFILE = "/tmp/run_ground_segmentation.pid"

# Default PCD file path
DEFAULT_PCD_FILE = os.path.join(
    WORKSPACE_DIR,
    "src/universe/autoware_universe/perception/autoware_ground_segmentation/test/data/test.pcd"
)

# Default topics
DEFAULT_INPUT_TOPIC = "/test/input/pointcloud"
DEFAULT_OUTPUT_TOPIC = "/test/output/pointcloud"

# Default test parameters
DEFAULT_NUM_POINTS = 1000
DEFAULT_GROUND_RATIO = 0.5
DEFAULT_PUBLISH_RATE = 10.0

procs = []


def kill_previous_instance():
    """Kill previous instance of this script."""
    try:
        with open(PIDFILE) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                pid = int(line)
                if pid == os.getpid():
                    continue
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
    except (FileNotFoundError, ValueError):
        pass
    with open(PIDFILE, "w") as f:
        f.write(f"{os.getpid()}\n")


def start_process(cmd, name):
    """Start a subprocess and print its output."""
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
    """Cleanup all started processes."""
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
    for name in ["scan_ground_filter_node", "ray_ground_filter_node",
                  "ransac_ground_filter_node", "rviz2"]:
        try:
            subprocess.run(["pkill", "-9", "-f", name],
                           capture_output=True, timeout=3)
        except:
            pass


def load_pcd_file(pcd_path):
    """Load PCD file and return valid points as numpy array.

    Filters out invalid points (x=0, y=0, z=0) which are typical in LiDAR scans
    where no return was received.
    """
    print(f"Loading PCD file: {pcd_path}")

    with open(pcd_path, 'r') as f:
        lines = f.readlines()

    # Parse header
    header = {}
    data_start = 0
    for i, line in enumerate(lines):
        if line.startswith('FIELDS'):
            header['fields'] = line.split()[1:]
        elif line.startswith('WIDTH'):
            header['width'] = int(line.split()[1])
        elif line.startswith('HEIGHT'):
            header['height'] = int(line.split()[1])
        elif line.startswith('POINTS'):
            header['points'] = int(line.split()[1])
        elif line.startswith('DATA'):
            data_start = i + 1
            break

    # Parse data points
    points = []
    total_count = 0
    for line in lines[data_start:]:
        line = line.strip()
        if not line:
            continue
        total_count += 1
        values = list(map(float, line.split()))
        points.append(values)

    points_array = np.array(points, dtype=np.float32)
    print(f"Parsed {total_count} points from PCD file")

    # Filter out invalid points (x=0, y=0, z=0)
    if len(points_array) > 0:
        valid_mask = (points_array[:, 0] != 0) | (points_array[:, 1] != 0) | (points_array[:, 2] != 0)
        valid_points = points_array[valid_mask]
        print(f"Filtered to {len(valid_points)} valid points (removed {total_count - len(valid_points)} invalid points at origin)")
        return valid_points
    else:
        print("Warning: No points found in PCD file")
        return points_array


def generate_ground_plane(num_points=5000, area_size=20.0):
    """Generate ground plane point cloud."""
    x = np.random.uniform(-area_size/2, area_size/2, num_points)
    y = np.random.uniform(-area_size/2, area_size/2, num_points)
    z = np.random.uniform(-0.1, 0.1, num_points)
    intensity = np.random.uniform(0, 1, num_points)
    return np.column_stack([x, y, z, intensity])


def generate_object_points(num_points=1000, center=(5.0, 5.0, 1.0), size=(2.0, 2.0, 2.0)):
    """Generate object point cloud (e.g., vehicle, pedestrian)."""
    x = np.random.uniform(center[0]-size[0]/2, center[0]+size[0]/2, num_points)
    y = np.random.uniform(center[1]-size[1]/2, center[1]+size[1]/2, num_points)
    z = np.random.uniform(center[2]-size[2]/2, center[2]+size[2]/2, num_points)
    intensity = np.random.uniform(0.5, 1.0, num_points)
    return np.column_stack([x, y, z, intensity])


def generate_test_scene(num_points=1000, ground_ratio=0.5):
    """Generate complete test scene with ground and objects."""
    num_ground = int(num_points * ground_ratio)
    num_objects = num_points - num_ground

    # Ground points
    ground_points = generate_ground_plane(num_points=num_ground)

    # Object points (vehicles, pedestrians)
    object1 = generate_object_points(
        num_points=num_objects//2,
        center=(5.0, 5.0, 1.0),
        size=(4.0, 2.0, 1.5)
    )
    object2 = generate_object_points(
        num_points=num_objects//2,
        center=(-5.0, 3.0, 0.8),
        size=(0.5, 0.5, 1.0)
    )

    # Combine all points
    all_points = np.vstack([ground_points, object1, object2])
    return all_points


def create_pointcloud2_msg(points, frame_id="base_link"):
    """Create PointCloud2 message with PointXYZIRC format.

    PointXYZIRC layout (16 bytes):
      x: FLOAT32, offset 0
      y: FLOAT32, offset 4
      z: FLOAT32, offset 8
      intensity: UINT8, offset 12
      return_type: UINT8, offset 13
      channel: UINT16, offset 14
    """
    msg = PointCloud2()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.header.frame_id = frame_id

    # Define fields (PointXYZIRC format required by pointcloud_preprocessor)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
    ]

    msg.height = 1
    msg.width = len(points)
    msg.point_step = 16  # x(4) + y(4) + z(4) + intensity(1) + return_type(1) + channel(2)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    # Pack data in PointXYZIRC format
    data = bytearray()
    for point in points:
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        intensity = int(point[3] * 255) & 0xFF if len(point) > 3 else 0
        data.extend(struct.pack('fff', x, y, z))
        data.append(intensity)       # intensity: UINT8
        data.append(0)               # return_type: UINT8 (0=unknown)
        data.extend(struct.pack('H', 0))  # channel: UINT16

    msg.data = bytes(data)

    return msg


class PointCloudPublisher(Node):
    """Node to publish point cloud data from PCD file or generated data."""

    def __init__(self, points, input_topic, publish_rate):
        super().__init__('ground_segmentation_test_publisher')

        # Use SensorDataQoS to match C++ filter node's BEST_EFFORT reliability
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher_ = self.create_publisher(PointCloud2, input_topic, sensor_qos)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_pointcloud)
        self.points = points
        self.point_count = 0
        self.input_topic = input_topic

        self.get_logger().info(f'Point cloud publisher started: {input_topic} (using SensorDataQoS)')
        self.get_logger().info(f'Publishing {len(points)} points at {publish_rate} Hz')

    def publish_pointcloud(self):
        """Publish point cloud."""
        msg = create_pointcloud2_msg(self.points)
        self.publisher_.publish(msg)
        self.point_count += 1

        if self.point_count % 50 == 0:  # Print every 5 seconds
            self.get_logger().info(f'Published {self.point_count} frames')


class OutputSubscriber(Node):
    """Node to subscribe to output point cloud and collect statistics."""

    def __init__(self, input_topic, output_topic):
        super().__init__('ground_segmentation_test_subscriber')

        self.input_count = 0
        self.output_count = 0
        self.input_points_total = 0
        self.output_points_total = 0

        # Use SensorDataQoS to match C++ filter node's BEST_EFFORT reliability
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.input_sub = self.create_subscription(
            PointCloud2, input_topic, self.input_callback, sensor_qos)
        self.output_sub = self.create_subscription(
            PointCloud2, output_topic, self.output_callback, sensor_qos)

        self.get_logger().info(f'Output subscriber started (using SensorDataQoS)')
        self.get_logger().info(f'  Input topic: {input_topic}')
        self.get_logger().info(f'  Output topic: {output_topic}')

    def input_callback(self, msg):
        """Callback for input point cloud."""
        self.input_count += 1
        self.input_points_total += msg.width

    def output_callback(self, msg):
        """Callback for output point cloud."""
        self.output_count += 1
        self.output_points_total += msg.width

        if self.output_count % 10 == 0:
            self.get_logger().info(
                f'Input: {self.input_count} frames ({self.input_points_total} pts), '
                f'Output: {self.output_count} frames ({self.output_points_total} pts)')

    def print_final_stats(self):
        """Print final statistics."""
        print("\n" + "=" * 60)
        print("Final Statistics")
        print("=" * 60)
        print(f"Total input frames: {self.input_count}")
        print(f"Total input points: {self.input_points_total}")
        print(f"Total output frames: {self.output_count}")
        print(f"Total output points: {self.output_points_total}")
        if self.input_points_total > 0:
            removal_rate = (1 - self.output_points_total / self.input_points_total) * 100
            print(f"Ground point removal rate: {removal_rate:.1f}%")


def signal_handler(sig, frame):
    """Handle signals for graceful shutdown."""
    print(f"\nReceived signal {sig}, shutting down...")
    try:
        rclpy.shutdown()
    except:
        pass
    cleanup()
    sys.exit(0)


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Test autoware_ground_segmentation module')

    # Input source
    parser.add_argument('--pcd-file', type=str, default=DEFAULT_PCD_FILE,
                        help=f'Path to PCD file to load (default: {DEFAULT_PCD_FILE})')

    # Manual generation options
    parser.add_argument('--num-points', type=int, default=DEFAULT_NUM_POINTS,
                        help=f'Number of points to generate (default: {DEFAULT_NUM_POINTS})')
    parser.add_argument('--ground-ratio', type=float, default=DEFAULT_GROUND_RATIO,
                        help=f'Ratio of ground points (default: {DEFAULT_GROUND_RATIO})')

    # Topic configuration
    parser.add_argument('--input-topic', type=str, default=DEFAULT_INPUT_TOPIC,
                        help=f'Input point cloud topic (default: {DEFAULT_INPUT_TOPIC})')
    parser.add_argument('--output-topic', type=str, default=DEFAULT_OUTPUT_TOPIC,
                        help=f'Output point cloud topic (default: {DEFAULT_OUTPUT_TOPIC})')

    # Filter type
    parser.add_argument('--filter', type=str, default='scan',
                        choices=['ray', 'scan', 'ransac'],
                        help='Ground filter type (default: scan)')

    # Other options
    parser.add_argument('--publish-rate', type=float, default=DEFAULT_PUBLISH_RATE,
                        help=f'Publish rate in Hz (default: {DEFAULT_PUBLISH_RATE})')
    parser.add_argument('--rviz-config', type=str, default=RVIZ_CONFIG,
                        help=f'RViz configuration file (default: {RVIZ_CONFIG})')
    parser.add_argument('--no-rviz', action='store_true',
                        help='Disable RViz launch')

    return parser.parse_args()


def main():
    """Main function."""
    args = parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Clean up previous instances
    print("Cleaning up leftover processes from previous runs...")
    kill_previous_instance()
    cleanup()
    time.sleep(2.0)

    print("=" * 60)
    print("Starting autoware_ground_segmentation simulation")
    print("=" * 60)

    # Load or generate point cloud data
    print("\n[1/5] Loading point cloud data...")
    if args.pcd_file:
        if not os.path.exists(args.pcd_file):
            print(f"Error: PCD file not found: {args.pcd_file}")
            return 1
        points = load_pcd_file(args.pcd_file)
    else:
        print(f"Generating test point cloud: {args.num_points} points, "
              f"{args.ground_ratio*100:.0f}% ground")
        points = generate_test_scene(
            num_points=args.num_points,
            ground_ratio=args.ground_ratio
        )
        print(f"Generated {len(points)} points")

    # 2. Build package (if needed)
    print("\n[2/5] Building autoware_ground_segmentation package...")
    build_proc = subprocess.run(
        ["colcon", "build", "--packages-select", "autoware_ground_segmentation",
         "--symlink-install"],
        cwd=WORKSPACE_DIR,
        capture_output=True,
        text=True
    )
    if build_proc.returncode != 0:
        print(f"Build failed: {build_proc.stderr}")
        return 1
    print("✓ Build successful")

    # 3. Start ground segmentation node
    print(f"\n[3/5] Starting ground segmentation node ({args.filter}_ground_filter)...")
    start_process([
        "bash", "-c",
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WORKSPACE_DIR}/install/setup.bash && "
        f"ros2 launch autoware_ground_segmentation {args.filter}_ground_filter.launch.xml "
        f"input/pointcloud:={args.input_topic} "
        f"output/pointcloud:={args.output_topic}"
    ], "ground_segmentation")
    time.sleep(3.0)  # Wait for node to start

    # 4. Initialize ROS 2 nodes
    print("\n[4/5] Starting publisher and subscriber nodes...")
    rclpy.init()

    publisher_node = PointCloudPublisher(points, args.input_topic, args.publish_rate)
    subscriber_node = OutputSubscriber(args.input_topic, args.output_topic)

    # 5. Start RViz (if enabled)
    if not args.no_rviz:
        print("\n[5/5] Starting RViz...")
        if os.path.exists(args.rviz_config):
            start_process([
                "rviz2", "-d", args.rviz_config,
            ], "rviz2")
        else:
            print(f"Warning: RViz config not found: {args.rviz_config}")
            start_process(["rviz2"], "rviz2")
    else:
        print("\n[5/5] RViz disabled (--no-rviz)")

    print("\n" + "=" * 60)
    print("All components running. Press Ctrl+C to stop.")
    print(f"Input topic: {args.input_topic}")
    print(f"Output topic: {args.output_topic}")
    print(f"Filter type: {args.filter}")
    print("=" * 60)

    try:
        while rclpy.ok():
            rclpy.spin_once(publisher_node, timeout_sec=0.1)
            rclpy.spin_once(subscriber_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        subscriber_node.print_final_stats()

        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()
        cleanup()


if __name__ == "__main__":
    main()
