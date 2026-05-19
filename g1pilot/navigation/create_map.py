#!/usr/bin/env python3
import os
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        default_yaml = os.path.join(
            get_package_share_directory('g1pilot'), 'config', 'simple_map.yaml'
        )

        self.declare_parameter('yaml_path', default_yaml)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)

        yaml_path = self.get_parameter('yaml_path').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value

        self._load_map(yaml_path)

        self.pub = self.create_publisher(OccupancyGrid, '/map', qos_profile=1)
        self.timer = self.create_timer(1.0 / rate, self.publish_map)
        self.get_logger().info(
            f'MapPublisher ready: {self.w}x{self.h} @ {self.res} m/px from {yaml_path}'
        )

    def _load_map(self, yaml_path: str):
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        self.res = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.ox = float(origin[0])
        self.oy = float(origin[1])
        self.origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
        negate = int(meta.get('negate', 0))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.196))
        mode = meta.get('mode', 'trinary')

        image_file = meta['image']
        if not os.path.isabs(image_file):
            image_file = os.path.join(os.path.dirname(yaml_path), image_file)

        img = Image.open(image_file).convert('L')
        pixels = np.array(img, dtype=np.float64)

        # ROS map_server convention: row 0 in the image is the top,
        # but in the map the origin is bottom-left, so flip vertically.
        pixels = np.flipud(pixels)

        if negate:
            occ = pixels / 255.0
        else:
            occ = (255.0 - pixels) / 255.0

        self.h, self.w = pixels.shape

        if mode == 'trinary':
            grid = np.full(pixels.shape, -1, dtype=np.int8)
            grid[occ >= occupied_thresh] = 100
            grid[occ <= free_thresh] = 0
        elif mode == 'scale':
            grid = np.clip((occ * 100.0), 0, 100).astype(np.int8)
        else:  # raw
            grid = np.clip(pixels, 0, 255).astype(np.int8)

        self.map_data = grid.flatten().tolist()

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.width = self.w
        grid.info.height = self.h
        grid.info.resolution = self.res

        origin = Pose()
        origin.position.x = self.ox
        origin.position.y = self.oy
        origin.orientation.w = 1.0
        if self.origin_yaw != 0.0:
            import math
            origin.orientation.z = math.sin(self.origin_yaw / 2.0)
            origin.orientation.w = math.cos(self.origin_yaw / 2.0)
        grid.info.origin = origin

        grid.data = self.map_data
        self.pub.publish(grid)


class DummyMapPublisher(Node):
    """Procedural map publisher (no image file needed)."""

    def __init__(self):
        super().__init__('dummy_map_publisher')
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('obstacles', '')
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.res = float(self.get_parameter('resolution').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.map_data = [0]*(self.w*self.h)
        self.ox = -(self.w*self.res)/2.0
        self.oy = -(self.h*self.res)/2.0

        # self.add_obstacle(2.0, 1.75, 1.2, 0.2)
        # self.add_obstacle(2.0, 0.25, 1.2, 0.2)
        # self.add_obstacle(0.8, 1.0, 0.2, 1.5)   

    def world_to_grid(self, x, y):
        ix = int((x - self.ox)/self.res)
        iy = int((y - self.oy)/self.res)
        return ix, iy

    def add_obstacle(self, x, y, largo, ancho):
        cx, cy = self.world_to_grid(x, y)
        hx = max(1, int(round((largo/2.0)/self.res)))
        hy = max(1, int(round((ancho/2.0)/self.res)))
        x0 = max(0, cx - hx); x1 = min(self.w-1, cx + hx)
        y0 = max(0, cy - hy); y1 = min(self.h-1, cy + hy)
        for yy in range(y0, y1+1):
            base = yy*self.w
            for xx in range(x0, x1+1):
                self.map_data[base+xx] = 100

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = self.frame_id
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.width = self.w
        grid.info.height = self.h
        grid.info.resolution = self.res
        origin = Pose()
        origin.position.x = self.ox
        origin.position.y = self.oy
        origin.orientation.w = 1.0
        grid.info.origin = origin
        grid.data = self.map_data
        self.pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = DummyMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
