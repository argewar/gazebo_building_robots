import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Câmera (Gazebo -> ROS 2)
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # Lidar (Gazebo -> ROS 2)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Odometria (Gazebo -> ROS 2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # Comando de Velocidade (ROS 2 -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        # Remapeia os tópicos no lado do ROS 2 para seguir as convenções
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge_node
    ])