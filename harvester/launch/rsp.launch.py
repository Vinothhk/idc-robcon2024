import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    urdf_file = 'urdf/bot_n.urdf'
    package_desc = 'harvester'
    print("URDF LOADING..")
    robot_desc_path = os.path.join(get_package_share_directory(package_desc), urdf_file)
    robot_controllers=os.path.join(get_package_share_directory("harvester"),"config","robocontroller.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_desc_path,robot_controllers],
        output="both",
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty = True,
        parameters=[{ 'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output = "screen"
    )
    
    rviz_config_dir = os.path.join(get_package_share_directory(package_desc), 'rviz','robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["roboarm_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["roboarm_joint_controller", "--controller-manager", "/controller_manager"],
    )
    
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    return LaunchDescription(
        [
            ros2_control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawner
        ]
    )