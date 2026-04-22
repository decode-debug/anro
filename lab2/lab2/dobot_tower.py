import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # klient akcji
        self.move_client = ActionClient(
            self,
            PointToPoint,
            'PTP_action',
        )

        # klient usługi

        # declare 'height' parameter (default 3) and read its integer value
        self.declare_parameter('height', 3)
        try:
            self.height = int(self.get_parameter('height').value)
        except (TypeError, ValueError):
            self.get_logger().warn('Invalid parameter "height", defaulting to 1')
            self.height = 1

        self.gripper_client = self.create_client(GripperControl, '/dobot_gripper_service')
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Could not find active gripper service server.')


        self.get_logger().info('Waiting for action server...')
        while not self.move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Could not find active action server.')

        # wykonanie sekwencji
        self.run_sequence()



    def move_sync(self, x, y, z, theta):

        goal = PointToPoint.Goal()
        goal.motion_type = 1
        goal.target_pose = [x, y, z, theta]
        goal.velocity_ratio = 1.0
        goal.acceleration_ratio = 1.0

        self.get_logger().info(f'Moving robot to target position: {(x, y, z, theta)}')

        send_goal_future = self.move_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Motion goal rejected by action server.')
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        if result_future.exception() is not None:
            self.get_logger().error(f'Motion failed: {result_future.exception()}')
            return False

        self.get_logger().info('Motion finished')
        return True
    
    def gripper_sync(self, state, hold):

        request = GripperControl.Request()

        request.gripper_state = state
        request.keep_compressor_running = hold  

        self.get_logger().info(f'Gripper state set to: {state}, keep_compressor_running={hold}')

        future = self.gripper_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(f'Gripper command failed: {future.exception()}')
            return False

        self.get_logger().info(f'Gripper command completed: {state}')
        return True


    def run_sequence(self):
        for i in range(self.height):
            # first cube
            if not self.gripper_sync('open', False):
                return
            if not self.move_sync(246 - i * 30, 86, 40, 0):
                return
            if not self.move_sync(246 - i * 30, 86, 1, 0):
                return
            # Keep compressor running while transporting the object.
            if not self.gripper_sync('close', True):
                return
            if not self.move_sync(246 - i * 30, 86, 40 + i * 20, 0):
                return
            if not self.move_sync(200, 0, 40 + i * 20, 0):
                return
            if not self.move_sync(200, 0, 1 + i * 20, 0):
                return
            self.gripper_sync('open', False)
            if not self.move_sync(200, 0, 40 + i * 20, 0):
                return

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

