#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import AiapaecPropeller
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int16

class CountUntilServer5Node(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_server5")  # MODIFY NAME
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()

        # Acción del servidor
        self.count_until_server_ = ActionServer(
            self,
            AiapaecPropeller, 
            "aiapaec_propeller", 
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())

        # Publicadores de Int16
        self.tp1_publisher_ = self.create_publisher(Int16, "micro_ros_node_subscriber_right_propeller", 10)
        self.tp2_publisher_ = self.create_publisher(Int16, "micro_ros_node_subscriber_left_propeller", 10)

        # Variables para almacenar la frecuencia y datos
        self.tp1_data_ = None
        self.tp2_data_ = None
        self.publish_frequency_ = None  # Frecuencia en Hz
        self.timer_ = None

        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: AiapaecPropeller.Goal):
        self.get_logger().info("Received a goal")
        
        # Validar la solicitud de objetivo
        if not (1100 <= goal_request.tp1 <= 1900) or not (1100 <= goal_request.tp2 <= 1900):
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        # Validar la frecuencia (si es válida, debe estar en el rango adecuado)
        if goal_request.period <= 0:
            self.get_logger().info("Rejecting the goal, invalid period")
            return GoalResponse.REJECT

        # Política: abortar objetivo existente cuando se reciba uno nuevo
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received a cancel request')
        return CancelResponse.ACCEPT  # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        # Obtener datos de la solicitud de objetivo
        tp1 = goal_handle.request.tp1
        tp2 = goal_handle.request.tp2
        period = goal_handle.request.period  # Frecuencia de publicación

        # Actualizar la frecuencia de publicación
        self.publish_frequency_ = 1.0 / period  # Convertir el período a frecuencia (Hz)

        # Ejecutar la acción
        self.get_logger().info("Executing the goal")
        feedback = AiapaecPropeller.Feedback()
        result = AiapaecPropeller.Result()

        # Configurar el temporizador solo cuando se reciba un goal
        self.setup_publish_timer()

        while rclpy.ok():
            if not goal_handle.is_active:
                result.tp1 = tp1
                result.tp2 = tp2
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.tp1 = tp1
                result.tp2 = tp2
                return result

            self.get_logger().info(str(tp1) + " and " + str(tp2))
            feedback.tp1 = tp1  # Modificar
            feedback.tp2 = tp2  # Modificar
            goal_handle.publish_feedback(feedback)

            # Almacenar los valores para publicarlos
            self.tp1_data_ = tp1
            self.tp2_data_ = tp2

            # Esperar el siguiente ciclo
            time.sleep(period)

        # Una vez completado, establecer el estado final del objetivo
        goal_handle.succeed()

        # Enviar el resultado
        result.tp1 = tp1
        result.tp2 = tp2
        return result

    def publish_data_callback(self):
        # Publicar los valores tp1 y tp2 a la frecuencia especificada
        if self.tp1_data_ is not None and self.tp2_data_ is not None:
            msg_tp1 = Int16()
            msg_tp1.data = self.tp1_data_
            self.tp1_publisher_.publish(msg_tp1)

            msg_tp2 = Int16()
            msg_tp2.data = self.tp2_data_
            self.tp2_publisher_.publish(msg_tp2)

    def setup_publish_timer(self):
        # Asegurarse de que el temporizador se configure solo una vez
        if self.publish_frequency_ is not None and self.timer_ is None:
            self.timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_data_callback)

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer5Node()  # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
