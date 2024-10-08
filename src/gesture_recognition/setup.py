from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gesture_recognition'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'gesture_recognizer_model'), 
        ['gesture_recognizer_model/gesture_recognizer.task']),  # Include the model file
    ],
    install_requires=['setuptools',
                      'opencv-python',
                      'mediapipe',
                      'cv_bridge',
                      'realsense2_camera',
                      'simulation_interfaces'],
    zip_safe=True,
    maintainer='alessio',
    maintainer_email='alessio.lovato.1@studenti.unipd.it',
    description='This package contains the gesture recognition node that recognizes gestures using the MediaPipe library and sends the corresponding state to the robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognizer_node = gesture_recognition.GestureNode:main',
        ],
    },
)
