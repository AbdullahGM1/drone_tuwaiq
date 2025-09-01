from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ns = '/drone'  # Define namespace
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash', '-c', 'cd /home/user/shared_volume/PX4-Autopilot && make px4_sitl gz_x500_lidar_camera'],
            name='px4_sitl',
            output='screen'
        ),
        # ROS-GZ Bridge 
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
                
                # Remapping 
                '--ros-args', '-r', '/world/default/model/x500_lidar_camera_0/link/pitch_link/sensor/camera/image:=' + ns + '/gimbal_camera',
                '--ros-args', '-r', '/world/default/model/x500_lidar_camera_0/link/pitch_link/sensor/camera/camera_info:=' + ns + '/gimbal_camera_info',
                '--ros-args', '-r', '/gimbal/cmd_yaw:=' + ns + '/gimbal/cmd_yaw',
                '--ros-args', '-r', '/gimbal/cmd_roll:=' + ns + '/gimbal/cmd_roll',
                '--ros-args', '-r', '/gimbal/cmd_pitch:=' + ns + '/gimbal/cmd_pitch',
                '--ros-args', '-r', '/imu_gimbal:=' + ns + '/imu_gimbal',
                '--ros-args', '-r', '/scan:=' + ns + '/scan',
                '--ros-args', '-r', '/scan/points:=' + ns + '/scan/points',
                
                # Sensors Remapping 
                '--ros-args', '-r', '/world/default/model/x500_lidar_camera_0/link/base_link/sensor/imu_sensor/imu:=' + ns + '/imu',
                '--ros-args', '-r', '/world/default/model/x500_lidar_camera_0/link/base_link/sensor/air_pressure_sensor/air_pressure:=' + ns + '/air_pressure',
                '--ros-args', '-r', '/navsat:=' + ns + '/gps',
            ],
        ),
    ])