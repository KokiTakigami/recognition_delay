# import rclpy
# import rclpy.duration
# from visualization_msgs.msg import Marker, MarkerArray
# import random
# from datetime import timedelta


# def main(args=None):
#     rclpy.init(args=args)
#     node = rclpy.create_node('marker_array_publisher')
#     pub = node.create_publisher(MarkerArray, '/carla/markers', 10)
    
#     id = 0
#     lifetime = 2
#     while rclpy.ok():
#         marker_array1 = MarkerArray()
#         marker_array2 = MarkerArray()
#         for i in range(2):
#             marker = Marker()
#             marker.header.frame_id = 'map'
#             marker.header.stamp = node.get_clock().now().to_msg()
#             marker.id = id
#             marker.type = Marker.CUBE
#             marker.action = Marker.ADD
#             marker.pose.position.x = random.uniform(-2.5, 2.5)
#             marker.pose.position.y = random.uniform(-2.5, 2.5)
#             marker.pose.position.z = random.uniform(-2.5, 2.5)
#             marker.pose.orientation.x = random.uniform(-1.0, 1.0)
#             marker.pose.orientation.y = random.uniform(-1.0, 1.0)
#             marker.pose.orientation.z = random.uniform(-1.0, 1.0)
#             marker.pose.orientation.w = random.uniform(-1.0, 1.0)
#             marker.scale.x = random.uniform(0.1, 1.0)
#             marker.scale.y = random.uniform(0.1, 1.0)
#             marker.scale.z = random.uniform(0.1, 1.0)
#             marker.color.r = 1.0
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.color.a = 0.80
#             # marker.lifetime = node.get_clock().now().to_msg()
#             # marker.lifetime = timedelta(seconds=5).to_msg()

#             # marker.lifetime = rclpy.duration.Duration(seconds=lifetime)
#             marker_array1.markers.append(marker)
#             id += 1

#         for i in range(2):
#             marker = Marker()
#             marker.header.frame_id = 'map'
#             marker.header.stamp = node.get_clock().now().to_msg()
#             marker.id = id
#             marker.type = Marker.CUBE
#             marker.action = Marker.ADD
#             marker.pose.position.x = random.uniform(-2.5, 2.5)
#             marker.pose.position.y = random.uniform(-2.5, 2.5)
#             marker.pose.position.z = random.uniform(-2.5, 2.5)
#             marker.pose.orientation.x = random.uniform(-1.0, 1.0)
#             marker.pose.orientation.y = random.uniform(-1.0, 1.0)
#             marker.pose.orientation.z = random.uniform(-1.0, 1.0)
#             marker.pose.orientation.w = random.uniform(-1.0, 1.0)
#             marker.scale.x = random.uniform(0.1, 1.0)
#             marker.scale.y = random.uniform(0.1, 1.0)
#             marker.scale.z = random.uniform(0.1, 1.0)
#             marker.color.r = 0.0
#             marker.color.g = 1.0
#             marker.color.b = 0.0
#             marker.color.a = 0.80
#             # marker.lifetime = rclpy.duration.Duration(seconds=lifetime)
#             # marker.lifetime = node.get_clock().now().to_msg()
#             marker_array2.markers.append(marker)
#             id += 1

#         pub.publish(marker_array1)
#         pub.publish(marker_array2)
#         rclpy.spin_once(node, period_sec=5)

#     pub.destroy()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import random
from rclpy.time import Duration

class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, '/carla/markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)  # 1.0秒ごとに実行

    def publish_markers(self):
        marker_array_msg = MarkerArray()
        
        for i in range(5):  # 5つのランダムなマーカーを生成
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = random.uniform(-1.0, 1.0)
            marker.pose.position.y = random.uniform(-1.0, 1.0)
            marker.pose.position.z = random.uniform(0.0, 2.0)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = random.uniform(0.0, 1.0)
            marker.color.g = random.uniform(0.0, 1.0)
            marker.color.b = random.uniform(0.0, 1.0)
            
            # マーカーのlifetimeを設定（ここでは5秒）
            # marker.lifetime = Duration(seconds=1)             
            marker_array_msg.markers.append(marker)
        
        self.publisher_.publish(marker_array_msg)
        self.get_logger().info("Published MarkerArray")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

