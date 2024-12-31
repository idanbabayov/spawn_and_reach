#!/usr/BIN/python3.10
import rclpy
import time
import numpy as np
from rclpy.node import Node
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2


class TurtleNavigator(Node):
    def __init__(self):
        super().__init__("Turtle_navigator")
        self.get_logger().info("Starting the Navigator!!")
        
        # Turtle's start and target positions
        self.master_turtle = [None, None, None]
        self.target_turtle = [None, None, None]
        self.target_name = None
        self.alive_turtles = []
        

        # Create subscribers and publisher
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.main_turtle_location_subscriber = self.create_subscription(Pose, "turtle1/pose", self.callback_main_turtle_location, 10)
        self.velocity_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        self.create_timer(1.0/100, self.control_loop)#call the control loop in a 100 hz frequency

    def control_loop(self):
        if self.target_name is not  None and self.master_turtle[0] is not None:
             
            self.target_turtle = np.array(self.target_turtle)
            self.master_turtle = np.array(self.master_turtle)
            dx = (self.target_turtle[0]-self.master_turtle[0])
            dy = (self.target_turtle[1]-self.master_turtle[1])
            arctan = atan2(dy,dx)
            dtheta = arctan - self.master_turtle[2]
            pi = 3.14

            if dtheta>pi:
                dtheta-=2*pi
            if dtheta<-pi:
                dtheta+=2*pi

            if (dx**2+dy**2)**0.5<=1: #stop condition and clear turtle
                dx = 0.0
                dy = 0.0
                dtheta = 0.0
                
                print("stop!!")

                #call the service /catch_turtle advertised by the turtle_spawner node#############
                self.call_catch_turtle(self.target_name)

            msg = Twist()
            msg.angular.z = 6.0*dtheta
            msg.linear.x = 2*dx
            msg.linear.y = 2*dy

            self.velocity_publisher.publish(msg)


        
        else:
            
            pass


    ################This Only passes the catched turtle name to the turtles_manager node!########################
    def call_catch_turtle(self,name):
        client = self.create_client(CatchTurtle,"catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server catch_turtle")
        
        request = CatchTurtle.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle))  # this is like spining
        
    def callback_call_catch_turtle(self,future):
        try:
            response = future.result()
            self.get_logger().info("success:" + str(response.success)) #can be True or False
        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))

     ############################################################################   


    def callback_main_turtle_location(self, msg):
        self.master_turtle[0] = msg.x
        self.master_turtle[1] = msg.y
        self.master_turtle[2] = msg.theta


    def callback_alive_turtles(self, msg):
        self.alive_trutles = msg.turtles
        self.target_turtle[0] = (msg.turtles[0].x)
        self.target_turtle[1] = (msg.turtles[0].y)
        self.target_turtle[2] = (msg.turtles[0].theta)
        self.target_name = msg.turtles[0].name
        print("the target name is:",self.target_name)
        




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