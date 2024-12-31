#!/usr/BIN/python3.10
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen,Kill,Spawn
from functools import partial
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from turtlesim.msg import Pose
import random

turtle_counter = 1
class TurtlesManager(Node):
    def __init__(self):
        super().__init__("Turtles_manager") #the name of the node
        self.get_logger().info("Welcome to the game!")
        self.create_timer(5.0, self.turtle_spawner) # Pass the function reference, not call it.
        self.alive_turtles_publisher = self.create_publisher(TurtleArray,"alive_turtles",10)
        self.alive_turtles = []

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtles_publisher.publish(msg) #publish the updated alive turtles list
    

    def turtle_spawner(self):
        global turtle_counter
        turtle_counter +=1

        x = random.uniform(0 , 10)
        y = random.uniform(0 , 10)
        theta = 0.0
        name = "turtle_" + str(turtle_counter)
        self.call_spawn_turtle( x,y,theta,name)




    def call_spawn_turtle(self,x,y,theta,name):
        client = self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server spawn")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_turtle,x = x,y=y,theta=theta,name=name))  # this is like spining

    def callback_call_spawn_turtle(self, future,x ,y,theta,name):
        try:
            response = future.result()
            self.get_logger().info("The turtle: " +response.name + " has been spawned!")

            msg = Turtle()
            msg.x = x
            msg.y = y
            msg.theta = theta
            msg.name = name
            
            self.alive_turtles.append(msg)
            self.publish_alive_turtles()
            

        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args = args) #starts the ros2 communication. a must in every project
    node = TurtlesManager()
    rclpy.spin(node) #keeps the node alive to the enviroment
    rclpy.shutdown()#turn off comuunication at the end of the program and kills the nodes

if __name__ == "__main__":
    main()