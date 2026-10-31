"""RViz 跟着实物动：robot_state_publisher + RViz + USB CDC 关节桥。

URDF 和 RViz 配置从 ros2_gong 的 `robotarm` 包读（只读，不改它）；
/joint_states 由本目录的 rviz_joint_states.py 从 igh6_jxb 的串口流翻译过来。

用法:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/ros2_gong/install/setup.bash      # 为了找到 robotarm 的 URDF
    ros2 launch ~/Desktop/igh6_jxb/host/display.launch.py
    ros2 launch ... display.launch.py rviz:=false      # 只看话题
    ros2 launch ... display.launch.py port:=/dev/ttyACM1
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    share = FindPackageShare('robotarm')
    urdf = PathJoinSubstitution([share, 'urdf', 'robotarm.urdf'])
    rviz_cfg = PathJoinSubstitution([share, 'rviz', 'display.rviz'])
    rviz = LaunchConfiguration('rviz')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'port', default_value='', description='留空则自动找 0483:5740'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str),
                'publish_frequency': 50.0,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz),
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', os.path.join(HERE, 'rviz_joint_states.py'),
                 '--port', port],
            output='screen',
        ),
    ])
