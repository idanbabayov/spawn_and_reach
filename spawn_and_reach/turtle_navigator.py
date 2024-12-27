#!/usr/BIN/python3.10
import rclpy
from rclpy.node import Node

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__("Turtle_navigator") #the name of the node
        self.get_logger().info("Starting the Navigator!")



def main(args=None):
    rclpy.init(args = args) #starts the ros2 communication. a must in every project
    node = TurtleNavigator()
    rclpy.spin(node) #keeps the node alive to the enviroment
    rclpy.shutdown()#turn off comuunication at the end of the program and kills the nodes

if __name__ == "__main__":
    main()