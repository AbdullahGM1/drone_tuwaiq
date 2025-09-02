from setuptools import find_packages, setup

package_name = 'drone_tuwaiq'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drone_tuwaiq.launch.py']),
        ('lib/' + package_name, ['drone_tuwaiq/drone_controller.py', 'drone_tuwaiq/command_interface.py', 'drone_tuwaiq/gimbal_yolo_node.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AbdullahGM1',
    maintainer_email='agm.musalami@gmail.com',
    description='Drone control package for Tuwaiq Academy',
    license='MIT',
    test_suite='pytest',
    entry_points={
        'console_scripts': [
            'drone_controller = drone_tuwaiq.drone_controller:main',
            'drone_command = drone_tuwaiq.command_interface:main',
            'gimbal_yolo_node = drone_tuwaiq.gimbal_yolo_node:main',
        ],
    },
)