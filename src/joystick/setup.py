from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joystick'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Copy of shared configuration file
        (os.path.join('lib', package_name), glob(os.path.join('joystick', 'config.py')))
    ],
    install_requires=['setuptools', 'ament_index_python', 'rclpy', 'std_msgs', 'geometry_msgs'],
    zip_safe=True,
    python_requires='>=3.6',
    maintainer='alessio',
    maintainer_email='alessio.lovato.1@studenti.unipd.it',
    description='Package that contains all the nodes to extract information from joystick status and control a differential drive controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'talker_360 = joystick.talker_360:main',
            'talker_PS3 = joystick.talker_PS3:main',
            'joy2cmdvel = joystick.joy2cmdvel:main'
        ]
    }
)