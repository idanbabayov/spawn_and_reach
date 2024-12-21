#!/usr/BIN/python3.10
import rclpy
from rclpy.node import Node
from sympy.abc import theta
from turtlesim.srv import SetPen,Kill,Spawn
from functools import partial
import random


turtle_counter = 1
turtle = []
class TurtlesManager(Node):
    def __init__(self):
        super().__init__("Turtles_manager") #the name of the node
        self.get_logger().info("Welcome to the game!")
        self.create_timer(4.0, self.turtle_spawner) # Pass the function reference, not call it.

    def turtle_spawner(self):
        new_turtle = self.random_turtle()
        self.call_spawn_turtle(new_turtle)


    def random_turtle(self):
        global turtle_counter
        global turtle
        turtle.clear()
        turtle_counter +=1

        x = random.uniform(0 , 10)
        y = random.uniform(0 , 10)
        theta = 0.0
        name = "turtle_" + str(turtle_counter)

        turtle.append(x)
        turtle.append(y)
        turtle.append(theta)
        turtle.append(name)
        print("the turtle is",turtle)
        return turtle


    def call_spawn_turtle(self,turtle_to_spawn):
        client = self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server spawn")
        request = Spawn.Request()
        request.x = turtle_to_spawn[0]
        request.y = turtle_to_spawn[1]
        request.theta = turtle_to_spawn[2]
        request.name = turtle_to_spawn[3]

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_turtle))  # this is like spining

    def callback_call_spawn_turtle(self, future):
        try:
            response = future.result()
            self.get_logger().info("The turtle: "+response.name + "has been spawned")
        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args = args) #starts the ros2 communication. a must in every project
    node = TurtlesManager()
    rclpy.spin(node) #keeps the node alive to the enviroment
    rclpy.shutdown()#turn off comuunication at the end of the program and kills the nodes

if __name__ == "__main__":
    main()