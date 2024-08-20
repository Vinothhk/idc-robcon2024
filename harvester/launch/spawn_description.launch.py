#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    position =  [2.0, 2.0, 0.4]  #[0.4, 1.5, 1.5]
    orientation = [0.0, 0.0, 3.14]
    robot_base_name = "HARVESTER BOT"

    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    return LaunchDescription(
        [
            spawn_robot,
        ]
    )