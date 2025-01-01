#!/usr/BIN/python3.10
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen,Kill,Spawn
from functools import partial
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import random

turtle_counter = 1
class TurtlesManager(Node):
    def __init__(self):
        super().__init__("Turtles_manager") #the name of the node
        self.get_logger().info("Welcome to the game!")
        self.create_timer(2.0, self.turtle_spawner) # Pass the function reference, not call it.
        self.alive_turtles_publisher = self.create_publisher(TurtleArray,"alive_turtles",10)
        self.catch_server = self.create_service(CatchTurtle,"catch_turtle",self.callback_catch_turtle)
        self.alive_turtles = []
    
    def callback_catch_turtle(self,request,respone):       
        if request.name is None:
            respone.success = False
            self.get_logger().info("waiting for a turtle name")
            
        elif request.name is not None:
            self.call_kill_turtle(request.name)
            respone.success = True
        return respone
    
    def call_kill_turtle(self,name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server kill")

        request = Kill.Request()
        request.name = name
        print("i am goingto kill:",name)

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_turtle,name=name))  # this is like spining

    def callback_call_kill_turtle(self, future,name):
        try:
            self.alive_turtles.pop(0)######how to kill the specific turtle name????????? currently kill the latest one  
            respone = future.result()
            self.clear_path()
            #self.get_logger().info("The turtle: " +response.name + " has been killed!")
            self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))
    
    def clear_path(self):
        self.call_clear_path()
    
    def call_clear_path(self):
        client = self.create_client(Empty,"clear")
        while not client.wait_for_service(1.0):
                self.get_logger().warn("waiting for server clear")
        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_clear_path))  # this is like spining

    def callback_call_clear_path(self, future):
        try:
            response = future.result()
            self.get_logger().info("CLEARED PATH!")
   
        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))




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