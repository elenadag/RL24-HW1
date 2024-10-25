from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
 

def generate_launch_description():


    arm_urdf_path = get_package_share_directory("arm_description")
    rviz_config_file = os.path.join(arm_urdf_path, "config", "arm.rviz")
    robot_arm_description = os.path.join(arm_urdf_path, "urdf", "arm.urdf")


    declared_arguments = [] 
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", #this will be the name of the argument  
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "rviz", "arm.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )

    # urdf_example_lesson_xacro = os.path.join(links_urdf_path, "urdf", "links.urdf.xacro")


    with open(robot_arm_description, 'r') as infp:
        arm_desc = infp.read()

    robot_description_links = {"robot_description": arm_desc}

    #r_d_x = {"robot_description":Command(['xacro ', urdf_example_lesson_xacro])}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )  

    robot_state_publisher_node_links = Node(
        package="robot_state_publisher", #ros2 run robot_state_publisher robot_state_publisher
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_links,
                    {"use_sim_time": True},
            ],
        remappings=[('/robot_description', '/robot_description')]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)
    
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'iiwa',
                   '-allow_renaming', 'true',],
    )

    ign = [gazebo_ignition, gz_spawn_entity]


    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node_links,
        rviz_node
        *ign
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start) 