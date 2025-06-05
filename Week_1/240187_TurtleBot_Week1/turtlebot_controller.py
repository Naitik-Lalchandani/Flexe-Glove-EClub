#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawShapesNode(Node):
    def __init__(self, shape):
        super().__init__("draw_shapes_node")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.01, self.send_velocity_command)
        self.shape = shape
        self.completed = False

        if self.shape == "square":
            self.state = 0
            self.move_time = 0.0
            self.turn_time = 0.0
            self.side_count = 0
            self.turn_duration = 1.0
            self.angular_speed = 1.57079632675
        elif self.shape == "spiral":
            self.linear_velocity = 0.5
            self.angular_velocity = 2.0
            self.time_elapsed = 0.0
            self.max_time = 15.0

    def send_velocity_command(self):
        if self.completed:
            return

        msg = Twist()

        if self.shape == "circle":
            msg.linear.x = 2.0
            msg.angular.z = 1.0
        elif self.shape == "square":
            if self.state == 0:
                msg.linear.x = 2.0
                self.move_time += 0.01
                if self.move_time >= 2.0:
                    self.state = 1
                    self.move_time = 0.0
            elif self.state == 1:
                msg.angular.z = self.angular_speed
                self.turn_time += 0.01
                if self.turn_time >= self.turn_duration:
                    self.state = 0
                    self.turn_time = 0.0
                    self.side_count += 1
                    if self.side_count >= 4:
                        self.completed = True
        elif self.shape == "spiral":
            if self.time_elapsed < self.max_time:
                msg.linear.x = self.linear_velocity + (self.time_elapsed * 0.2)
                msg.angular.z = self.angular_velocity
                self.time_elapsed += 0.01
            else:
                self.completed = True

        if self.completed:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    valid_shapes = ["circle", "square", "spiral"]
    try:
        shape = sys.argv[1].lower()
        if shape not in valid_shapes:
            raise IndexError
    except IndexError:
        shape = "circle"
    
    node = DrawShapesNode(shape)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
