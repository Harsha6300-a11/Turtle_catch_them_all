#!/usr/bin/env python3

import rclpy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleController(Node):
    def __init__(self):
        super().__init__("Turtle_controller")
        self.declare_parameter("catch_closest_turtle_first",True)
        self.catch_closest_turtle_first_=self.get_parameter("catch_closest_turtle_first").value
        self.pose_ = None 
        self.turtle_to_catch_ = None
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray , "alive_turtles",self.callback_alive_turtles_,10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_alive_turtles_(self, msg: TurtleArray):
        if len(msg.turtles)>0 :
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                self.turtle_to_catch_ = msg.turtles[0]

    def pose_callback(self, pose: Pose):
        self.pose_ = pose  

    def control_loop(self):
        if self.pose_ is None or self.turtle_to_catch_ is None:
            return

        cmd = Twist()
        dist_x = self.turtle_to_catch_.x- self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        if distance > 0.5:
            cmd.linear.x = 2.0 * distance
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2.0 * math.pi
            elif diff < -math.pi:
                diff += 2.0 * math.pi
            cmd.angular.z = 7.0 * diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_pub_.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for catch turtle service...")
        
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle_service, turtle_name=turtle_name))

    def callback_call_catch_turtle_service(self, future, turtle_name):
        response: CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error("Turtle " + turtle_name + " could not be removed")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
