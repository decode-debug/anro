import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray 

class PositionReader(Node):
    def __init__(self):
        super().__init__('position_reader_node')

        self.current_pose = [0.0, 0.0, 0.0, 0.0]

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/dobot_pose',
            self.listener_callback,
            10 
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('PositionReader node has been started and is listening to /dobot_pose topic.')

    def listener_callback(self, msg):
        """This function runs quietly in the background. It only triggers when the robot sends data."""
        self.current_pose = msg.data

    def timer_callback(self):
        """This is the body of your loop. It runs at the frequency of the Timer."""
        self.get_logger().info(f'X, Y, Z, R: {self.current_pose}')


def main(args=None):
    rclpy.init(args=args)
    node = PositionReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()