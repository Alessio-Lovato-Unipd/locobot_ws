#!/usr/bin/env python3

"""
@file
This file contains the node that convert a wrapped joystick status (see 'talker_PS3.py' or 'talker_360.py' file) to a Twist_unstamped message via the Joystick2CmdVelNode class.
The node subscribes to the /joystatus topic and publishes on the /morpheus_diff_drive_controller/cmd_vel_unstamped topic, 
the input topic of the differential drive controller defined in the morpheus_gazebo package.
Please refer to 'config.py' file to set the correct mapping of the joystick input.
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

# import configuration variables
from config import movements

##
# @brief This class publishes on the /morpheus_diff_drive_controller/cmd_vel_unstamped topic.
#
# It subscribes to the /joystatus topic, which is the output of the talker_PS3 or talker_360 node.
# The publishing message is a Twist_unstamped message, with linear.x and angular.z fields setted by the joystick input.
# The publishing topic is the input of the differential drive controller congifured in
# the morpheus_gazebo package.

class Joystick2CmdVelNode(Node):
    # Constructor
    def __init__(self):
        super().__init__('joystick2cmdvel_node')

        ## Subscription to the /joystatus topic
        self.__subscription = self.create_subscription(
            Float32MultiArray,
            'joystatus',
            self.input_callback,
            10)

        ## Publisher on controller's input topic /cmd_vel_unstamped
        self.__publisher = self.create_publisher(Twist, 'cmdvel_unstamped', 10)

        # Support variables to store data
        ## Linear velocity
        self.__v = 0.0
        ## Angular velocity
        self.__w = 0.0

    ##@brief Callback of the /joystatus topic.
    # 
    # Publishes a Twist_unstamped message on the /cmd_vel_unstamped topic
    # based on the /joystatus input.
    #
    #@param msg Float32MultiArray message from the /joystatus topic
    def input_callback(self, msg):
        # Acquiring data based on the configuration file
        self.__v = msg.data[movements.get("x_movement")]  # 1 move forward, -1 move backward
        self.__w = msg.data[movements.get("z_rotation")]  # 1 counterclockwise rotation, -1 clockwise rotation
                               
        # Exponential (cubic) input distribution
        self.__v = math.pow(self.__v, 3)
        self.__w = math.pow(self.__w, 3)

        # Publish cmd vel message
        out_msg = Twist()
        out_msg.linear.x = self.__v
        out_msg.angular.z = self.__w
        self.__publisher.publish(out_msg)

## Main function
def main(args=None):

    # Initialize ROS2
    rclpy.init(args=args)
    # create node
    joy2cmdvel_node = Joystick2CmdVelNode()

    # Spin
    joy2cmdvel_node.get_logger().info('Starting joy2cmdvel node, shut down with CTRL-C')
    
    # Expecting a keyboard interrupt to shut down
    try:
        rclpy.spin(joy2cmdvel_node)
    except KeyboardInterrupt:
        pass
    
    joy2cmdvel_node.get_logger().info('Keyboard interrupt, shutting down.\n')

    # Shutdown ROS2
    joy2cmdvel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
