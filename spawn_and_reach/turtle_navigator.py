#!/usr/BIN/python3.10
import rclpy
import time
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import atan2



#####################################
#In this code we will use the  Cubic Bézier curve Method in it's Polynomial form!.
#The key idea is to compute the waypoints or positions along the Bézier curve, 
#and then send these positions as velocity commands(linear/angular) (using geometry_msgs/Twist) to control the robot's motion.
#Steps:
#1)Define the Bézier Curve: 
#Define the curve as before with start, end, and control points.
#2)Calculate the Points Along the Bézier Curve: Using the Bézier equation, 
#calculate the positions (waypoints) at discrete time steps along the curve.
#3)Compute the Tangent (Velocity): At each waypoint, calculate the robot's desired velocity vector. 
#This is typically the tangent (derivative) of the curve at that point.
#4)Publish the Twist Messages: Send the calculated velocity vectors as geometry_msgs/Twist messages to control the robot's movement.
#5)Move the Robot Step-by-Step: Move the robot incrementally along the path by continuously sending the velocity commands 
# based on the current position and tangent of the curve.
###########################################
#Here's how to compute the angular velocity:

#Compute the Robot's Heading: At each point on the curve, calculate the direction of movement (tangent of the curve)
#  and convert this direction into an angle (heading).

#Current Orientation: At each time step, compare the robot's current orientation (the angle at the robot's position)
#  with the desired direction of motion (the angle along the tangent of the Bézier curve).

#Compute Angular Velocity: The angular velocity angular.z can be computed by taking the difference between the current heading and the desired heading.
#  This difference represents how much the robot needs to turn to align itself with the path.
######################################

P0 = [-0.1, 0.0]  # Initial position
P1 = [0.0, 0.0]  # Control points, to be computed
P2 = [0.0, 0.0]
P3 = [-0.1, 0.0]  # Target position
current_heading = 0.0

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__("Turtle_navigator")
        self.get_logger().info("Starting the Navigator!")
        self.turtle_location_subscriber = self.create_subscription(Pose, "turtle_position", self.callback_turtle_location, 1)
        self.main_turtle_location_subscriber = self.create_subscription(Pose, "turtle1/pose", self.callback_main_turtle_location, 1)
        self.velocity_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 1)
        self.create_timer(2.0, self.start_algorithm)

    def start_algorithm(self):
        global P0, P1, P2, P3, current_heading

        if P0 != [-0.1, 0.0] and P3 != [-0.1, 0.0]:  # Check if both P0 and P3 are initialized
            self.compute_bezier_control_points(P0, P3)
            counter = 0
            for t in np.linspace(0, 1, num=50):  # Generate 8 points along the curve
                if np.linalg.norm(np.array(P0)-np.array(P3))<=1.0:
                        stop_cmd = Twist()
                        stop_cmd.linear.x = 0.0
                        stop_cmd.linear.y = 0.0
                        stop_cmd.angular.z = 0.0
                        self.velocity_publisher.publish(stop_cmd)
                        print("stop the turtle@@@@@@@@@@@@@@@@@@@@@@@@@")
                        break
                waypoint = self.bezier(t, P0, P1, P2, P3)
                tangent = self.bezier_derivative(t, P0, P1, P2, P3)
                desired_heading = atan2(tangent[1], tangent[0])
                rotation_to_align_with_path = current_heading - desired_heading


                
                #print("rot:",rotation_to_align_with_path)
                #print("linearx,y:",tangent)
                counter+=1
                print("counter",counter)
                print("the location of the target:",np.array(P3))

                move_cmd = Twist()
                
                move_cmd.linear.x = tangent[0]
                move_cmd.linear.y = tangent[1]
                move_cmd.angular.z = rotation_to_align_with_path
                self.velocity_publisher.publish(move_cmd)
                time.sleep(2.0/49)  # Sleep for 0.2 seconds


    def compute_bezier_control_points(self, P0, P3, tangent_scale=0.6):
        global P1, P2
        P0 = np.array(P0)
        P3 = np.array(P3)
        tangent_P0 = (P3 - P0) * tangent_scale / 3  # P1 controls start direction
        tangent_P3 = (P3 - P0) * tangent_scale / 3  # P2 controls end direction
        
        # Compute P1 and P2 based on the tangents
        P1 = P0 + tangent_P0
        P2 = P3 - tangent_P3

    def callback_main_turtle_location(self, msg):
        global P0, current_heading
        P0[0] = msg.x
        P0[1] = msg.y
        current_heading = msg.theta

    def callback_turtle_location(self, msg):
        global P3
        P3[0] = msg.x
        P3[1] = msg.y

    def bezier(self, t, P0, P1, P2, P3):
        P0 = np.array(P0)
        P1 = np.array(P1)
        P2 = np.array(P2)
        P3 = np.array(P3)
        x = (1 - t)**3 * P0[0] + 3 * (1 - t)**2 * t * P1[0] + 3 * (1 - t) * t**2 * P2[0] + t**3 * P3[0]
        y = (1 - t)**3 * P0[1] + 3 * (1 - t)**2 * t * P1[1] + 3 * (1 - t) * t**2 * P2[1] + t**3 * P3[1]
        return np.array([x, y])

    def bezier_derivative(self, t, P0, P1, P2, P3):
        P0 = np.array(P0)
        P1 = np.array(P1)
        P2 = np.array(P2)
        P3 = np.array(P3)
        dx = -3 * (1 - t)**2 * P0[0] + 3 * (1 - t)**2 * P1[0] + 6 * (1 - t) * t * P2[0] - 3 * t**2 * P3[0]
        dy = -3 * (1 - t)**2 * P0[1] + 3 * (1 - t)**2 * P1[1] + 6 * (1 - t) * t * P2[1] - 3 * t**2 * P3[1]
        if dx>8.0:
            dx = 8.0
        elif dx<-8:
            dx =-8.0
        if dy>8.0:
            dy = 8.0
        elif dy<-8.0:
            dy =-8.0    
        return np.array([dx, dy])

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()