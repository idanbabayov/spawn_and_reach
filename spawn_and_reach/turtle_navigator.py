#!/usr/BIN/python3.10
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist


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

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__("Turtle_navigator") #the name of the node
        self.get_logger().info("Starting the Navigator!")
        self.turtle_location_subscriber = self.create_subscription(Pose2D,"turtle_position",self.callback_turtle_location,10)
        #TODO NEED TO PASS ALSO TURTLE1 INIT POSITION! 
    
    def callback_turtle_location(self,msg):
        pass
    
    def bezier (self,t,PO,P1,P2,P3):
        x = (1 - t)**3 * P0[0] + 3 * (1 - t)**2 * t * P1[0] + 3 * (1 - t) * t**2 * P2[0] + t**3 * P3[0]
        y = (1 - t)**3 * P0[1] + 3 * (1 - t)**2 * t * P1[1] + 3 * (1 - t) * t**2 * P2[1] + t**3 * P3[1]
        return np.array([x, y])
    
    def bezier_derivative(self, t, P0, P1, P2, P3):
        #Derivative of the cubic Bézier curve to calculate direction at time t
        dx = -3 * (1 - t)**2 * P0[0] + 3 * (1 - t)**2 * P1[0] + 6 * (1 - t) * t * P2[0] - 3 * t**2 * P3[0]
        dy = -3 * (1 - t)**2 * P0[1] + 3 * (1 - t)**2 * P1[1] + 6 * (1 - t) * t * P2[1] - 3 * t**2 * P3[1]
        return np.array([dx, dy])






def main(args=None):
    rclpy.init(args = args) #starts the ros2 communication. a must in every project
    node = TurtleNavigator()
    rclpy.spin(node) #keeps the node alive to the enviroment
    rclpy.shutdown()#turn off comuunication at the end of the program and kills the nodes

if __name__ == "__main__":
    main()