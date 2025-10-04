from setuptools import setup

package_name = 'q2r_bringup'

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
    description='A framework for using Meta Quest 2/3 VR controllers quest2ros to remotely control a KUKA dual-arm robot through ROS 2 and ROS–TCP communication.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2quest = q2r_bringup.ros2quest:main',
            'SImulationInpt = q2r_bringup.SImulationInpt:main',
            'CheckTCPconnection = q2r_bringup.CheckTCPconnection:main',
            'set_target_point = q2r_bringup.set_target_point:main',
            'go_to_xyz = q2r_bringup.go_to_xyz:main',
            'send_goal_model = q2r_bringup.send_goal_model:main',
            'send_goal = q2r_bringup.send_goal:main',
            'fk_client = q2r_bringup.fk_client:main',
            'Q2R_control_goal_left = q2r_bringup.Q2R_control_goal_left:main',
            'Q2R_control_goal_right = q2r_bringup.Q2R_control_goal_right:main',
            'left_arm_controller = q2r_bringup.left_arm_controller:main',
            'right_arm_controller = q2r_bringup.right_arm_controller:main',
        ],
    },
)
