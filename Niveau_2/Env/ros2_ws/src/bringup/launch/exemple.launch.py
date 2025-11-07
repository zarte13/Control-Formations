# mission/launch/bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mavros",
            executable="mavros_node",
            output="screen",
            parameters=[
                {
                    "fcu_url": "tcp://192.168.0.35:5762",
                    "tgt_system": 1,
                    "tgt_component": 1,
                }
            ],
        ),
        Node(
            package='solution_du_siecle',
            executable='ardupilotcontroller',   # mission/colin_drone_node.py : main()
            name='ardupilot_controller',
            output='screen',
        ),
        Node(
            package='mission_control',
            executable='balloon',         # mission_control/ballon_pub.py : main()
            name='ballon',
            output='screen',
        ),
        Node(
            package='mission_control',
            executable='monitor',            # mission_control/monitor.py : main()
            name='monitor',
            output='screen',
        ),
    ])
