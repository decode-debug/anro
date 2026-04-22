"""
Kontroler żółwia - moduł do zarządzania operacjami żółwi w turtlesim.

Ten moduł implementuje węzeł ROS2, który wykonuje następujące kroki:
1. Obraca żółwia o 180 stopni w lewo.
2. Tworzy drugiego żółwia o nazwie podanej w parametrze.
3. Obraca żółwia o 270 stopni w prawo.
4. Tworzy trzeciego żółwia o nazwie podanej w parametrze.
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlesim.action import RotateAbsolute
from turtlesim.srv import Spawn


class TurtleController(Node):
    """Klasa kontrolera żółwia, zarządza sekwencją operacji na żółwiu."""

    def __init__(self):
        """Inicjalizuje węzeł sterujący żółwiem."""
        super().__init__('turtle_controller')

        # 1. Deklaracja parametrów dla nazw żółwi
        self.declare_parameter('turtle2_name', 'drugi_zolw')
        self.declare_parameter('turtle3_name', 'trzeci_zolw')

        # 2. Utworzenie klientów Akcji i Usług
        self.action_client = ActionClient(
            self, RotateAbsolute, '/turtle1/rotate_absolute'
        )
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Oczekiwanie na uruchomienie węzła turtlesim
        self.get_logger().info('Czekam na serwery turtlesim...')
        self.action_client.wait_for_server()
        self.spawn_client.wait_for_service()

        # Pobranie parametrów z węzła
        self.name2 = (
            self.get_parameter('turtle2_name')
            .get_parameter_value()
            .string_value
        )
        self.name3 = (
            self.get_parameter('turtle3_name')
            .get_parameter_value()
            .string_value
        )

        # Rozpoczęcie zadania
        self.step1_rotate_left()

    def step1_rotate_left(self):
        """Obracanie w lewo o 180 stopni."""
        self.get_logger().info('Krok 1: Obrót 180 stopni w lewo...')
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = -math.pi  # 180 stopni (w radianach)

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.step1_response)

    def step1_response(self, future):
        """Sprawdzenie odpowiedzi na obrót w lewo."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Krok 1 nieudany: serwer odrzucił żądanie obrotu do -pi.')
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.step2_spawn_first)

    def step2_spawn_first(self, future):
        """Tworzenie drugiego żółwia. i sprawdzenie czy został utworzony."""
        self.get_logger().info(
            f'Krok 2: Tworzenie drugiego żółwia: {self.name2}'
        )
        req = Spawn.Request()
        req.x = 2.0
        req.y = 2.0
        req.theta = 0.0
        req.name = self.name2

        self.spawn_future = self.spawn_client.call_async(req)
        self.spawn_future.add_done_callback(self.step3_rotate_right)

    def step3_rotate_right(self, future):
        """Obracanie w prawo o 270 stopni."""
        self.get_logger().info('Krok 3.1: Ustawiam kąt turtle1 na pi/2 (90 stopni).')
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = math.pi / 2.0

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.step3_response)

    def step3_response(self, future):
        """Sprawdzenie odpowiedzi na obrót w prawo."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Krok 3.1 nieudany: serwer odrzucił żądanie obrotu do pi/2.')
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.step3_1_rotate_right)

    def step3_1_rotate_right(self, future):
        """Obracanie w prawo o 270 stopni."""
        self.get_logger().info('Krok 3.2: Ustawiam kąt turtle1 na 0 rad.')
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = 0.0

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.step3_1_response)

    def step3_1_response(self, future):
        """Sprawdzenie odpowiedzi na obrót w prawo."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Krok 3.2 nieudany: serwer odrzucił żądanie obrotu do 0 rad.')
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.step3_2_rotate_right)

    def step3_2_rotate_right(self, future):
        """Obracanie w prawo o 270 stopni."""
        self.get_logger().info('Krok 3.4: Ustawiam kąt turtle1 na -pi/2 (-90 stopni).')
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = -math.pi / 2.0

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.step3_2_response)

    def step3_2_response(self, future):
        """Sprawdzenie odpowiedzi na obrót w prawo."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Krok 3.4 nieudany: serwer odrzucił żądanie obrotu do -pi/2.')
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.step4_spawn_second)

    def step4_spawn_second(self, future):
        """Tworzenie trzeciego żółwia i sprawdzenie czy został utworzony."""
        self.get_logger().info(
            f'Krok 4: Tworzenie trzeciego żółwia: {self.name3}'
        )
        req = Spawn.Request()
        req.x = 8.0
        req.y = 8.0
        req.theta = 0.0
        req.name = self.name3

        self.spawn_future = self.spawn_client.call_async(req)
        self.spawn_future.add_done_callback(self.sequence_finished)

    def sequence_finished(self, future):
        """Końcowa funkcja po utworzeniu trzeciego żółwia."""
        self.get_logger().info('Sukces! Sekwencja zakończona.')


def main(args=None):
    """Funkcja główna uruchamiająca węzeł sterujący."""
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
