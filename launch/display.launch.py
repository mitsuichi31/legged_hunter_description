import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('legged_hunter_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'hunter.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Since the original was just a URDF, we can likely just read it directly if xacro isn't needed,
    # but using xacro command is safer if they add macros later.
    # However, for pure URDF like the one we saw, simple file reading is often enough.
    # Let's stick to reading the file content for simplicity if it's just urdf,
    # OR use the 'urdf' package conventions.
    
    # Actually, verify if the file is URDF or XACRO. 
    # The file extension is .urdf. 
    # Let's assume standard robot_state_publisher usage.

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
