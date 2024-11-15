#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_client") # MODIFY NAME
        self.count_until_client_ = ActionClient(
            self, 
            CountUntil, 
            "count_until")

    def send_goal(self, target_number, period):
        # Wait for the server
        self.count_until_client_.wait_for_server()

        # Create a goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self. get_logger().info("Sending goal")
        self.count_until_client_.send_goal_async(goal)


        

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode() 
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()