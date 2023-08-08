import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('LidarSub')
        self.subscription = self.create_subscription(PointCloud2, '/lidar/Lidar1', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I point: "%d"' % msg.point_step)
        self.get_logger().info('I row: "%d"' % msg.row_step)
        self.get_logger().info('I height: "%d"' % msg.height)
        self.get_logger().info('I width: "%d"' % msg.width)
        self.get_logger().info('I heard: "%d"' % msg.data[0])
        self.get_logger().info('I lenght "%d"' % len(msg.data))

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()