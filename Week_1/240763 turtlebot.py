import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def draw_square(turt, publisher, side_length=2.0, linear_speed=1.0, angular_speed=1.57):
    twist = Twist()
    for _ in range(4):
        twist.linear.x = linear_speed
        twist.angular.z = 0.0
        publisher.publish(twist)
        time.sleep(side_length/linear_speed)
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        publisher.publish(twist)
        time.sleep(1)
        twist.angular.z = 0.0
        publisher.publish(twist)

def draw_spiral(turt, publisher, radius = 0.0, final_radius = 5.5, angular_speed=2.5):
    twist = Twist()
    while radius <=  final_radius:
        twist.linear.x = radius * angular_speed
        twist.angular.z = angular_speed
        publisher.publish(twist)
        time.sleep(radius/angular_speed)
        radius += 0.15
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    publisher.publish(twist)

rclpy.init()
turt = Node('turtle')
publisher = turt.create_publisher(Twist, '/turtle1/cmd_vel', 10)
time.sleep(1)
draw_square(turt, publisher)
draw_spiral(turt, publisher)
turt.destroy_node()
rclpy.shutdown
