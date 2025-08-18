#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class DummyMapPublisher(Node):
    def __init__(self):
        super().__init__('dummy_map_publisher')
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('frame_id', 'odom')
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

        self.add_obstacle(2.0, 1.75, 1.2, 0.2)
        self.add_obstacle(2.0, 0.25, 1.2, 0.2)
        self.add_obstacle(1.0, 1.0, 0.2, 1.5)   

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
