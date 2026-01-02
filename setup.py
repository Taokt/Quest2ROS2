from setuptools import setup

package_name = 'q2r2_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jialong Li, Zhenguo Wang, Tianci Wang',
    maintainer_email='jialong.li@cs.lth.se',
    description='A framework for using VR controllers to remotely control robot system through ROS 2 and ROS–TCP communication.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2quest = q2r2_bringup.ros2quest:main',
            'SimulationInput = q2r2_bringup.SimulationInput:main',
            'CheckTCPconnection = q2r2_bringup.CheckTCPconnection:main',
            'left_arm_controller = q2r2_bringup.left_arm_controller:main',
            'right_arm_controller = q2r2_bringup.right_arm_controller:main',
        ],
    },
)
