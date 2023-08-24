#!/usr/bin/env python

import time
import threading
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class DelayGenerator(Node):
    def __init__(self, role_name, delay_time):
        super().__init__('delay_generator')

        self.markers_sub = self.create_subscription(
            MarkerArray, "/carla/markers", self._markers_updated, 10)
        self.markers_delay_pub = self.create_publisher(
            MarkerArray, f"/carla/{role_name}/markers/delay", 10)
        
        self._role_name = role_name
        # [second]
        self.delay_time = delay_time
    
    def _markers_updated(self, marker_array):
        t = threading.Thread(target=self.publish_delay_marker_array, args=(marker_array,))
        t.start()
    
    def publish_delay_marker_array(self, marker_array):
        time.sleep(self.delay_time)

        for marker in marker_array.markers:
            self._convert_marker_color(marker)
            marker.header.stamp = self.get_clock().now().to_msg()

        self.markers_delay_pub.publish(marker_array)

    def _convert_marker_color(self, marker):
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
    
    def set_delay_time(self, delay_time):
        self.delay_time = delay_time


def main(args=None):
    rclpy.init(args=args)

    role_name = 'ego_vehicle'
    delay_time = 0.1
    delay_generator = DelayGenerator(role_name, delay_time)

    try:
        rclpy.spin(delay_generator)
    except KeyboardInterrupt:
        pass
    
    delay_generator.markers_sub.destroy()
    delay_generator.markers_delay_pub.destroy()
    delay_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
