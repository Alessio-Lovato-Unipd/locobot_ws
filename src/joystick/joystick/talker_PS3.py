#!/usr/bin/env python3


"""
@file
This file contains the node that convert a generic PS3 controller input to specific Float32MultiArray message.

This node is intended to be used as selector of the /joy topic messages data and to map specific input from the controller into a Float32MultiArray message.
The message is then published to /joystatus topic.
Please refer to 'config.py' file to set the correct mapping of the joystick input.
"""


# Author: Riccardo Dal Borgo
# Modified by: Alessio Lovato


# Based on ros2 joy package, a JOY message is composed by:

#  int32[] axes (values range [-1 1]) that are mapped as follows:
#     [0] Left stick horizontal axis: left = 1, right = -1
#     [1] Left stick vertical axis: up = 1, down = -1
#     [2] L2 button: pressed = -1, released = 1
#     [3] Right stick horizontal axis: left = 1, right = -1
#     [4] Right stick vertical axis: up = 1, down = -1
#     [5] R2 button: pressed = -1, released = 1

#  int32[] buttons (values 0 or 1) that are mapped as follows:
#     [0] Cross
#     [1] Circle
#     [2] Square
#     [3] Triangle
#     [4] L1
#     [5] R1
#     [6] L2 pressed
#     [7] R2 pressed
#     [8] select
#     [9] start
#     [10] power
#     [11] L3
#     [12] R3

import rclpy
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

# import configuration variables
from config import movements

# Creation of dictionaries

## @brief Dictionary of buttons
#
## The key is the button name, the value is the index in the buttons array of the Joy message
## The values of the buttons are 0 or 1
buttons = {
    "cross" : 0,
    "circle" : 1,
    "square" : 2,
    "triangle" : 3,
    "L1" : 4,
    "R1" : 5,
    "L2" : 6,
    "R2" : 7,
    "select" : 8,
    "start" : 9,
    "power" : 10,
    "L3" : 11,
    "R3" : 12
}

## @brief Dictionary of axes
#
## The key is the axis name, the value is the index in the axes array of the Joy message.
## The values of the axes are in the range [-1, 1]
axes = {
    "left_horizontal" : 0,
    "left_vertical" : 1,
    "L2" : 2,
    "right_horizontal" : 3,
    "right_vertical" : 4,
    "R2" : 5,
}


## @brief Node that reads the PS3 joystick status and publishes it to joystatus topic
#
# This class is a ROS2 node that subscribes to the /joy topic and publishes a wrapped message to the /joystatus topic.
# The message is a Float32MultiArray message, with the values of the axes and buttons mapped to the movements defined in the config.py file.
class TalkerPS3(Node) :
    ## Constructor
    def __init__(self):
        super().__init__("talker_PS3")
        ## Publisher to joystatus topic of message type Float32MultiArray
        self.__publisher = self.create_publisher(Float32MultiArray, 'joystatus', 10)
        ## Subscription to /joy topic
        self.__subscription = self.create_subscription(Joy, "joy", self.joyread_callback, 10)
        ## Controller connection status
        self.__controller_connected = False
        ## Controller safety activation
        self.__controller_safety = False



    ## @brief Callback of /joy topic.
    #
    # Publish a message to /joystatus topic only if the controller is connected and the sticks have been moved in all the direction and then released.
    # After the required movements have been detected, a 5 seconds delay is set to allow the user to release the sticks safely.
    #
    # @param data data received from /joy topic
    # @return None
    def joyread_callback(self, data):
        # Remove noise from axes
        for i in range(0,len(data.axes)):
            if (data.axes[i] < 0.1 and data.axes[i] > -0.1):
                data.axes[i] = 0.0

        # Check message validity
        if ((not self.__controller_connected)):
            # N.B. If the controller is not connected or the sticks haven't been moved,
            #      all the axes values are 1
            if (data.axes[axes.get("left_horizontal")] == 0 and 
                    data.axes[axes.get("left_vertical")] == 0 and
                    data.axes[axes.get("right_horizontal")] == 0 and
                    data.axes[axes.get("right_vertical")] == 0):
                # set 5 seconds delay to check the new message and allow a safe start
                if (not self.__controller_safety):
                    self.__controller_safety = True
                    self.get_logger().info('Waiting 5 seconds...Please release the sticks')
                    time.sleep(5)
                    return

                # Controller connected
                self.__controller_connected = True
                self.get_logger().info('Controller connected')
                return

            elif (self.__controller_safety):
                self.get_logger().info('Waiting 5 seconds...Please release the sticks')
                time.sleep(5)
                return

            else:
                # Controller not connected, do not publish
                self.get_logger().info('Controller not activated! Please turn it on, move the sticks and then release them to activate the controller.')
                return
        
            
        #Declaration type
        status = Float32MultiArray()
        status.data = [0.0] * len(movements) 


        # Match the buttons and axes to the movements

        # Forward/Backward
        status.data[movements.get("x_movement")] = data.axes[axes.get("left_vertical")]
        # Turn Left/Right
        status.data[movements.get("z_rotation")] = data.axes[axes.get("right_horizontal")]

        #Add other movements here

        # Publish message
        self.__publisher.publish(status)

    
    ## @brief Check if the sticks have been released.
    #
    # The sticks are considered released if the absolute value of the axes values is less than a margin.
    # The default margin is 0.2 but it can set in a range [0, 1].
    #
    # @param axis_values array of the axes values
    # @param margin margin of the check
    # @return True if the sticks have been released, False otherwise
    def __sticks_released(self, axis_values, margin=0.2):
        return (((axis_values[axes.get("left_horizontal")] < margin) and axis_values[axes.get("left_horizontal")] > -margin) and
                ((axis_values[axes.get("left_vertical")] < margin) and axis_values[axes.get("left_vertical")] > -margin) and
                ((axis_values[axes.get("right_horizontal")] < margin) and axis_values[axes.get("right_horizontal")] > -margin) and
                ((axis_values[axes.get("right_vertical")] < margin) and axis_values[axes.get("right_vertical")] > -margin))



## @brief Main function
#
# Initialize ROS2 and start the TalkerPS3 node. Shuts down with CTRL-C.
def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    # Create the node
    node = TalkerPS3()
    node.get_logger().info('Starting PS3 joystick node, shut down with CTRL-C')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Shutting down...')
    finally:
        node.destroy_node()
        #Shutdown
        rclpy.shutdown()
        



if __name__ == '__main__':
    main()
