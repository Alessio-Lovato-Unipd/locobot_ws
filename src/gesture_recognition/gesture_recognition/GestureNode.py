#!/usr/bin/env python3
"""
@file GestureNode.py
@package gesture_recognition
@brief This module contains the GestureRecognizer class that recognizes gestures using the MediaPipe library and sends the corresponding state to the robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import signal
import sys

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import os
from ament_index_python import get_package_share_directory
from locobot_control_interfaces.srv import ControlStates
import rcl_interfaces.msg


"""
@brief Class to recognize gestures using the MediaPipe library and send the corresponding state to the robot.
@details The gestures recognized are:
    - 'Closed_Fist': Sets the state to CLOSE_GRIPPER.
    - 'Pointing_Up': Sets the state to OPEN_GRIPPER.
    - 'Thumb_Up': Sets the state to NAVIGATION.
    - 'Thumb_Down': Sets the state to INTERACTION.
    - 'Open_Palm': Sets the state to IDLE.
    - 'ILoveYou': Sets the state to ABORT.

The recognized gestures are published using a service client. The service name is given as a parameter.
The recognized gesture is also printed in the console.

@param camera_topic The topic of the camera image.
@param service_name The name of the service to connect with. If null, the service client won't be used.
@param minimum_score The minimum score to consider a gesture (default is 0.6). Must be between 0 and 1.
@param rotate_image Flag to decide if the image from the camera should be flipped horizontally before being analyzed. If true (default), gestures must be performed towards down.

@note The gesture recognizer model is loaded from the gesture_recognizer_model folder in the package and it is the 
    'HandGestureClassifier' model with float 16 quantization type. It is the default one.
"""
class GestureRecognizer(Node):
    def __init__(self):
        super().__init__('gesture_recognizer')
        # Declare parameters with description
        from rcl_interfaces.msg import ParameterDescriptor
        description = ParameterDescriptor(description='Topic of the camera image')
        self.declare_parameter('camera_topic', '/camera/image_raw', description)
        description = ParameterDescriptor(description='Name of the service to connect with')
        self.declare_parameter('service_name', '', description)
        description = ParameterDescriptor(description="Rotate the image from the camera. Default True.")
        self.declare_parameter('rotate_image', True, description)

        # Variable to save the rotation parameter
        self.rotate_image = self.get_parameter('rotate_image').value

        # Load the gesture recognizer model
        model_path = os.path.join(get_package_share_directory('gesture_recognition'), 'gesture_recognizer_model', 'gesture_recognizer.task')
        base_options = python.BaseOptions(model_asset_path=model_path,)
        options = vision.GestureRecognizerOptions(base_options=base_options)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        # Initialize the last gesture to None
        self.last_gesture = 'None' 

        # Get minimum score to consider a gesture
        description = rcl_interfaces.msg.ParameterDescriptor(description='Minimum score to consider a gesture')
        description.floating_point_range.append(rcl_interfaces.msg.FloatingPointRange(from_value=0.0, to_value=1.0))
        self.declare_parameter('minimum_score', 0.6, description)
        self.minimum_score = self.get_parameter('minimum_score').value

        # Create a subscriber to the camera image
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_converter_callback,
            10)
        self.br = CvBridge()

        # Create a service client to control the state machine
        self.use_client = self.get_parameter('service_name').value != ''
        if self.use_client:
            self.client = self.create_client(ControlStates, self.get_parameter('service_name').value)
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = ControlStates.Request()

    """
    @brief Sends the state corresponding to the given gesture to the state machine.
    @param gesture The gesture string. 
    @note Possible values are:
        - 'Closed_Fist': Sets the state to CLOSE_GRIPPER.
        - 'Pointing_Up': Sets the state to OPEN_GRIPPER.
        - 'Thumb_Up': Sets the state to NAVIGATION.
        - 'Thumb_Down': Sets the state to INTERACTION.
        - 'Open_Palm': Sets the state to IDLE.
        - 'ILoveYou': Sets the state to ABORT.
    @return None
    """
    def send_state(self, gesture):
        # Convert the string to the corresponding integer from the ControlStates message
        if gesture == 'Closed_Fist':
            self.req.state = ControlStates.Request.CLOSE_GRIPPER
        elif gesture == 'Pointing_Up':
            self.req.state = ControlStates.Request.OPEN_GRIPPER
        elif gesture == 'Thumb_Up':
            self.req.state = ControlStates.Request.NAVIGATION
        elif gesture == 'Thumb_Down':
            self.req.state = ControlStates.Request.INTERACTION
        elif gesture == 'Open_Palm':
            self.req.state = ControlStates.Request.IDLE
        elif gesture == 'ILoveYou':
            self.req.state = ControlStates.Request.ABORT
        else:
            return
        # Send the new state to the state machine
        self.future = self.client.call_async(self.req)
    
    """
    @brief Callback function to convert ROS image messages to OpenCV images, process them using MediaPipe, and recognize gestures.
    @param data The ROS image message to be converted and processed.
    @return None
    """
    def image_converter_callback(self, data):
        # Convert the image message to a cv2 image
        current_frame = self.br.imgmsg_to_cv2(data)
        # Convert the image to RGB
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        # Rotate the image to be able to to the gestures pointing towards the floor.
        # This is only done to enhance ergonomics
        if self.rotate_image:
            current_frame = cv2.rotate(current_frame, cv2.ROTATE_180)
        # Convert to MediaPipe image format
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=current_frame)
        # Process the image
        results = self.recognizer.recognize(mp_image)
        # Print the results
        if not results.gestures:
            return
        else:
            gesture = results.gestures[0][0]
            if not gesture.category_name == 'None' and gesture.score > self.minimum_score:
                # Prevent a gesture to be sent multiple times, except for the Victory and Closed_Fist gestures that can be repeated.
                # Note that those gesture can be repeated since if they're sent during the wrong state, they will be ignored.
                if self.last_gesture != gesture.category_name or gesture.category_name == 'Pointing_Up' or gesture.category_name == 'Closed_Fist':
                    if self.use_client:
                        self.send_state(gesture.category_name)
                    self.last_gesture = gesture.category_name
                    rclpy.logging.get_logger('gesture_recognizer').info(f"Gesture: {gesture.category_name} with confidence {gesture.score}")





def main(args=None):
    rclpy.init(args=args)
    image_subscriber = GestureRecognizer()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            image_subscriber.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()