#!/usr/bin/env python3

"""
@file

This file provides the sequence definition of the data stored in the /joystatus topic message.

The message is a Float32MultiArray message, and the sequence is defined in 'movements' variable.
"""


## @brief Maps the rover's movements to the indexes of the Float32MultiArray message used in the /joystatus topic.
movements = {
    "x_movement" : 0,
    "z_rotation" : 1
    # Other movements can be added here
}
