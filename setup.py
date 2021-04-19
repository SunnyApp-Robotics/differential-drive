import os
from glob import glob
from setuptools import setup

package_name = 'differential_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('launch/*')),
        (os.path.join('share', package_name, 'launch'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Duarte',
    maintainer_email='electronica@sunnyapp.com',
    description='The differential_drive package Provides some basic tools for interfacing a differential-drive robot with the ROS navigation stack.  The intent is to make this independent of specific robot implementation.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_joystick = differential_drive.virtual_joystick:main',
            'kinematics = differential_drive.kinematics:main',
            'odometry_encoders = differential_drive.odometry_encoders:main'
        ],
    },
)
