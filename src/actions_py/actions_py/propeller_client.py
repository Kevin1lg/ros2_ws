#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import AiapaecPropeller
import curses


class PropellerClientNode(Node):
    def __init__(self):
        super().__init__("propeller_client")
        self.propeller_client_ = ActionClient(
            self, 
            AiapaecPropeller, 
            "aiapaec_propeller")

    def send_goal(self, tp1, tp2, period):
        # Wait for the server
        self.propeller_client_.wait_for_server()

        # Create a goal
        goal = AiapaecPropeller.Goal()
        goal.tp1 = tp1
        goal.tp2 = tp2
        goal.period = period

        # Send the goal
        self.get_logger().info(f"Sending goal: tp1={tp1}, tp2={tp2}, period={period}")
        self.propeller_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Result: tp1_result={result.tp1_result}, tp2_result={result.tp2_result}")

    def goal_feedback_callback(self, feedback_msg):
        tp1 = feedback_msg.feedback.tp1_feedback
        tp2 = feedback_msg.feedback.tp2_feedback
        self.get_logger().info(f"Got feedback: tp1_feedback={tp1}, tp2_feedback={tp2}")


def get_user_input(stdscr, node):
    # Limpiar pantalla
    stdscr.clear()
    stdscr.refresh()

    # Instrucciones
    stdscr.addstr("Press 'q' to quit.\n")
    stdscr.addstr("Type values manually for tp1 and tp2 (1100-1900).\n")
    stdscr.addstr("Press 'p' to send the current goal after entering values.\n")
    stdscr.refresh()

    current_line = 4  # Línea donde empieza la entrada dinámica
    tp1 = 1500 # rigth propeller
    tp2 = 1500 # left propeller
    period = 0.1 # period of the action

    while True:
        # Captura de entrada para tp1
        stdscr.addstr(current_line, 0, "Enter tp1 (1100-1900): ")
        tp1_str = ""
        tp1_pos = 23  # Empezamos a mostrar después del prompt y un espacio
        while True:
            key = stdscr.getch()
            if key == 10:  # Tecla Enter
                break
            elif key == curses.KEY_BACKSPACE:  # Tecla de retroceso (Backspace)
                tp1_str = tp1_str[:-1]
            elif 48 <= key <= 57:  # Solo se permiten caracteres numéricos
                tp1_str += chr(key)
            # Actualizar la entrada de forma dinámica en la pantalla
            stdscr.addstr(current_line, tp1_pos, tp1_str + " " * 10)  # Limpiar los caracteres sobrantes
            stdscr.refresh()

        # Validar entrada de tp1
        try:
            tp1 = int(tp1_str)
            if not (1100 <= tp1 <= 1900):
                raise ValueError("Out of range")
        except ValueError:
            stdscr.addstr(current_line + 1, 0, "Invalid input for tp1! Please try again.\n")
            stdscr.refresh()
            continue

        current_line += 1

        # Captura de entrada para tp2
        stdscr.addstr(current_line, 0, "Enter tp2 (1100-1900): ")
        tp2_str = ""
        tp2_pos = 23  # Empezamos a mostrar después del prompt y un espacio
        while True:
            key = stdscr.getch()
            if key == 10:  # Tecla Enter
                break
            elif key == curses.KEY_BACKSPACE:  # Tecla de retroceso (Backspace)
                tp2_str = tp2_str[:-1]
            elif 48 <= key <= 57:  # Solo se permiten caracteres numéricos
                tp2_str += chr(key)
            # Actualizar la entrada de forma dinámica en la pantalla
            stdscr.addstr(current_line, tp2_pos, tp2_str + " " * 10)  # Limpiar los caracteres sobrantes
            stdscr.refresh()

        # Validar entrada de tp2
        try:
            tp2 = int(tp2_str)
            if not (1100 <= tp2 <= 1900):
                raise ValueError("Out of range")
        except ValueError:
            stdscr.addstr(current_line + 1, 0, "Invalid input for tp2! Please try again.\n")
            stdscr.refresh()
            continue

        current_line += 1
        stdscr.addstr(current_line, 0, f"Values set: tp1={tp1}, tp2={tp2}\n")
        current_line += 1

        stdscr.addstr(current_line, 0, "Press 'p' to send the goal or 'q' to quit.\n")
        stdscr.refresh()

        key = stdscr.getch()
        if key == ord('q'):
            break  # Salir si se presiona 'q'
        elif key == ord('p'):
            node.send_goal(tp1, tp2, period)
            stdscr.addstr(current_line, 0, f"Goal sent: tp1={tp1}, tp2={tp2}, period={period}\n")
            current_line += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = PropellerClientNode()

    # Start curses mode for key input
    curses.wrapper(get_user_input, node)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
