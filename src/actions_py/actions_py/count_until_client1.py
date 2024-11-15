#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil


class CountUntilClient1Node(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_client1") # MODIFY NAME
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
        self.count_until_client_. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected")            

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: "+ str(result.reached_number))



        

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient1Node() 
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()