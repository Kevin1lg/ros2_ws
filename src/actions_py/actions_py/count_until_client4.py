#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import AiapaecPropeller


class CountUntilClient4Node(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("count_until_client2") # MODIFY NAME
        self.count_until_client_ = ActionClient(
            self, 
            AiapaecPropeller, 
            "aiapaec_propeller")

    def send_goal(self, tp1, tp2, period):
        # Wait for the server
        self.count_until_client_.wait_for_server()

        # Create a goal
        goal = AiapaecPropeller.Goal()
        goal.tp1 = tp1
        goal.tp2 = tp2
        goal.period = period

        # Send the goal
        self. get_logger().info("Sending goal")
        self.count_until_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)

        # Send a cancel request 2 seconds later
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)
    
    def cancel_goal(self):
        self.get_logger().info("Send a cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()


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
        self.get_logger().info("Result: "+ str(result.tp1) + " and " + str(result.tp2))

    def goal_feedback_callback(self, feedback_msg):
        tp1 = feedback_msg.feedback.tp1
        tp2 = feedback_msg.feedback.tp2
        self.get_logger().info("Got feedback: " + str(tp1) + " and " + str(tp2))


        

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient4Node() 
    node.send_goal(1600, 1600, 0.1)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()