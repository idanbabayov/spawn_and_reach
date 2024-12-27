#!/usr/BIN/python3.10
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen,Kill,Spawn
from functools import partial
from geometry_msgs.msg import Pose2D
import random
#################test_second_commit##########

turtle_counter = 1
turtle = []
class TurtlesManager(Node):
    def __init__(self):
        super().__init__("Turtles_manager") #the name of the node
        self.get_logger().info("Welcome to the game!")
        self.create_timer(4.0, self.turtle_spawner) # Pass the function reference, not call it.
        self.turtle_location_publisher = self.create_publisher(Pose2D,"turtle_position",10)

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
        future.add_done_callback(partial(self.callback_call_spawn_turtle,turtle_to_spawn = turtle_to_spawn))  # this is like spining

    def callback_call_spawn_turtle(self, future, turtle_to_spawn):
        try:
            response = future.result()
            self.get_logger().info("The turtle: " +response.name + " has been spawned")

            msg = Pose2D()
            msg.x = turtle_to_spawn[0]
            msg.y = turtle_to_spawn[1]
            msg.theta = turtle_to_spawn[2]

            self.turtle_location_publisher.publish(msg) #publish the location of the turtle to the navigator node

            

        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args = args) #starts the ros2 communication. a must in every project
    node = TurtlesManager()
    rclpy.spin(node) #keeps the node alive to the enviroment
    rclpy.shutdown()#turn off comuunication at the end of the program and kills the nodes

if __name__ == "__main__":
    main()