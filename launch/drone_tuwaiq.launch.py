from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ns = '/drone'  # Define namespace
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash', '-c', 'cd /home/user/shared_volume/PX4-Autopilot && make px4_sitl gz_x500_lidar_camera'],
            name='px4_sitl',
            output='screen'
        ),
        
        # Start micro-XRCE-DDS agent
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                    name='micro_xrce_agent',
                    output='screen'
                )
            ]
        ),
        
        # ROS-GZ Bridge with delay
        TimerAction(
            period=4.0,
            actions=[
                Node(
            package='ros_gz_bridge',
            name='ros_bridge_node',
            executable='parameter_bridge',
            arguments=[
                # Existing topics 
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/world/default/model/x500_lidar_camera_0/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/default/model/x500_lidar_camera_0/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double',
                '/gimbal/cmd_roll@std_msgs/msg/Float64]gz.msgs.Double',
                '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
                '/imu_gimbal@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/world/default/model/x500_lidar_camera_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/world/default/model/x500_lidar_camera_0/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                
                    # All sensor and control topics
                    ],
                    remappings=[
                        ('/world/default/model/x500_lidar_camera_0/link/pitch_link/sensor/camera/image', ns + '/gimbal_camera'),
                        ('/world/default/model/x500_lidar_camera_0/link/pitch_link/sensor/camera/camera_info', ns + '/gimbal_camera_info'),
                        ('/gimbal/cmd_yaw', ns + '/gimbal/cmd_yaw'),
                        ('/gimbal/cmd_roll', ns + '/gimbal/cmd_roll'),
                        ('/gimbal/cmd_pitch', ns + '/gimbal/cmd_pitch'),
                        ('/imu_gimbal', ns + '/imu_gimbal'),
                        ('/scan', ns + '/scan'),
                        ('/scan/points', ns + '/scan/points'),
                        ('/world/default/model/x500_lidar_camera_0/link/base_link/sensor/imu_sensor/imu', ns + '/imu'),
                        ('/world/default/model/x500_lidar_camera_0/link/base_link/sensor/air_pressure_sensor/air_pressure', ns + '/air_pressure'),
                        ('/navsat', ns + '/gps'),
                    ]
                ),
            ]
        ),
        
        # Start YOLO detection with delay
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('yolo_bringup'), 
                                   'launch', 'yolo.launch.py')
                    ]),
                    launch_arguments={
                        'input_image_topic': '/drone/gimbal_camera',
                        'model': 'yolov8n.pt',  # Using nano model for better performance
                        'device': 'cuda:0',  # Using GPU acceleration
                        'threshold': '0.3',  # Lower threshold to detect more objects
                        'use_tracking': 'False',
                        'use_3d': 'False',
                        'use_debug': 'True',  # Enable debug to see detection images
                        'namespace': 'yolo',  # Ensure correct namespace
                        'enable': 'True'  # Explicitly enable YOLO
                    }.items()
                )
            ]
        ),
        
        # Start gimbal YOLO node with delay
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='drone_tuwaiq',
                    executable='gimbal_yolo_node.py',
                    name='gimbal_yolo_node',
                    output='screen'
                )
            ]
        ),
        
        # Note: After PX4 shows 'Ready for takeoff!', run this command in PX4 console:
        # uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
    ])