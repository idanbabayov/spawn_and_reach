#!/usr/BIN/python3.10
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
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

P0 = [] #the main turtle position
P1 = [] #extra point
P2 = [] #extra point
P3 = [] #the desired position of the spawned turtle
current_heading = 0.0 #the current theta of the turtle
class TurtleNavigator(Node):
    def __init__(self):
        super().__init__("Turtle_navigator") #the name of the node
        self.get_logger().info("Starting the Navigator~~~~~~")
        self.turtle_location_subscriber = self.create_subscription(Pose,"turtle_position",self.callback_turtle_location,10)
        self.main_turtle_location_subscriber = self.create_subscription(Pose,"turtle1/pose",self.callback_main_turtle_location , 10)
        self.velocity_publisher = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.create_timer(4.0, self.start_algorithm) # Pass the function reference, not call it.


    
    def start_algorithm(self):
        global P0
        global P1
        global P2
        global P3
        global current_heading

        self.compute_bezier_control_points(P0,P3)
         # Loop over 't' values from 0 to 1 (discretized path)
        for t in np.linspace(0, 1, num=8):  # Generate 8 points along the curve
            # Compute the next point on the Bézier curve
            waypoint = self.bezier(t, P0, P1, P2, P3)

            # Calculate the velocity (tangent) at that point
            tangent = self.bezier_derivative(t, P0, P1, P2, P3)
            
            # Calculate the desired heading (orientation//angle) based on the tangent
            desired_heading = atan2(tangent[1], tangent[0])

            rotation_to_align_with_path = current_heading - desired_heading


            # Create a Twist message
            move_cmd = Twist()

            # Set linear velocity 
            move_cmd.linear.x = tangent

            # Set angular velocity in radians per sec
            move_cmd.angular.z = rotation_to_align_with_path

            self.velocity_publisher.publish(move_cmd)



    # Function to compute P1 and P2
    def compute_bezier_control_points(P0, P3, tangent_scale=1.0):
        """
        Compute the control points P1 and P2 for a cubic Bézier curve given P0 and P3.
        
        Parameters:
        - P0: Start point (x0, y0)
        - P3: End point (x3, y3)
        - tangent_scale: Scaling factor to control how far P1 and P2 are from P0 and P3 (default=1.0)
        
        Returns:
        - P1, P2: Control points for the cubic Bézier curve
        """
        global P1
        global P2
        # Convert points to numpy arrays for easier manipulation
        P0 = np.array(P0)
        P3 = np.array(P3)
        
        # Compute tangent vectors (basic direction approach)
        tangent_P0 = (P3 - P0) * tangent_scale / 3  # P1 controls start direction
        tangent_P3 = (P3 - P0) * tangent_scale / 3  # P2 controls end direction
        
        # Compute P1 and P2 based on the tangents
        P1 = P0 + tangent_P0
        P2 = P3 - tangent_P3
    
        
    def callback_main_turtle_location(self,msg):
        global P0
        global current_heading
        P0[0] = msg.x
        P0[1] = msg.y
        current_heading = msg.theta
        pass
    
    def callback_turtle_location(self,msg):
        global P3
        P3[0] = msg.x
        P3[1] = msg.y
        pass
    
    def bezier (self,t,PO,P1,P2,P3):
        # Convert points to numpy arrays for easier manipulation
        P0 = np.array(P0)
        P1 = np.array(P1)
        P2 = np.array(P2)
        P3 = np.array(P3)
        x = (1 - t)**3 * P0[0] + 3 * (1 - t)**2 * t * P1[0] + 3 * (1 - t) * t**2 * P2[0] + t**3 * P3[0]
        y = (1 - t)**3 * P0[1] + 3 * (1 - t)**2 * t * P1[1] + 3 * (1 - t) * t**2 * P2[1] + t**3 * P3[1]
        return np.array([x, y])
    
    def bezier_derivative(self, t, P0, P1, P2, P3):
        # Convert points to numpy arrays for easier manipulation
        P0 = np.array(P0)
        P1 = np.array(P1)
        P2 = np.array(P2)
        P3 = np.array(P3)
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