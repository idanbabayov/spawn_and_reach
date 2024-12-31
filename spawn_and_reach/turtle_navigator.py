#!/usr/BIN/python3.10
import rclpy
import time
import numpy as np
from rclpy.node import Node
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2


class TurtleNavigator(Node):
    def __init__(self):
        super().__init__("Turtle_navigator")
        self.get_logger().info("Starting the Navigator!!")
        
        # Turtle's start and target positions
        self.master_turtle = [None, None, None, None]
        self.target_turtle = [None, None, None, None]
        self.alive_turtles = []
        

        # Create subscribers and publisher
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 1)
        self.main_turtle_location_subscriber = self.create_subscription(Pose, "turtle1/pose", self.callback_main_turtle_location, 1)
        self.velocity_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 1)

        self.create_timer(1.0/45, self.control_loop)#call the control loop in a 45 hz frequency

    def control_loop(self):
        if self.target_turtle[0] is not  None and self.master_turtle[0] is not None:
            
            self.target_turtle = np.array(self.target_turtle)
            self.master_turtle = np.array(self.master_turtle)
            dx = np.linalg.norm(self.target_turtle[0]-self.master_turtle[0])
            dy = np.linalg.norm(self.target_turtle[1]-self.master_turtle[1])
            arctan = atan2(dy,dx)
            dtheta = self.master_turtle[2]-arctan

            if dx**2+dy**2<=1: #stop condition and clear turtle
                dx = 0.0
                dy = 0.0
                dtheta = 0.0
                self.target_turtle[:] = [None, None, None, None]
                self.alive_turtles.pop(0)

            msg = Twist()
            msg.angular.z = dtheta
            msg.linear.x = dx
            msg.linear.y = dy

            self.velocity_publisher.publish(msg)


        
        else:
            print("check!!#")
            pass




    def callback_main_turtle_location(self, msg):
        self.master_turtle[0] = msg.x
        self.master_turtle[1] = msg.y
        self.master_turtle[2] = msg.theta
        

    def callback_alive_turtles(self, msg):
        self.alive_trutles = msg.turtles
        self.target_turtle[0] = (msg.turtles[0].x)
        self.target_turtle[1] = (msg.turtles[0].y)
        self.target_turtle[2] = (msg.turtles[0].theta)
        #self.target_turtle[3] = msg.turtles[0].name
        print(self.target_turtle)




def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
"""
    def start_algorithm(self):
        stastic_P0 = np.copy(self.P0)

        if not np.array_equal(self.P0, [-0.1, 0.0]) and not np.array_equal(self.P3, [-0.1, 0.0]):  # Check if both P0 and P3 are initialized
            self.compute_bezier_control_points(self.P0, self.P3)
            counter = 0
            for t in np.linspace(0, 1, num=50):  # Generate points along the curve
                if np.linalg.norm(stastic_P0 - self.P3) <= 1.5:  # Stop condition
                    stop_cmd = Twist()
                    stop_cmd.linear.x = 0.0
                    stop_cmd.linear.y = 0.0
                    stop_cmd.angular.z = 0.0
                    self.velocity_publisher.publish(stop_cmd)
                    print("Stopping the turtle")
                    break

                waypoint = self.bezier(t, self.P0, self.P1, self.P2, self.P3)
                tangent = self.bezier_derivative(t, self.P0, self.P1, self.P2, self.P3)
                desired_heading = atan2(tangent[1], tangent[0])
                rotation_to_align_with_path = self.current_heading - desired_heading

                counter += 1
                print("Counter:", counter)
                print("The location of the target:", self.P3)

                move_cmd = Twist()
                move_cmd.linear.x = tangent[0]
                move_cmd.linear.y = tangent[1]
                move_cmd.angular.z = rotation_to_align_with_path
                self.velocity_publisher.publish(move_cmd)
                time.sleep(2.0 / 49)  # Sleep for 0.2 seconds
"""