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
    maintainer='The Authors',
    maintainer_email='anon.author@anon.org',
    description='A framework for using VR controllers to remotely control robot system through ROS 2 and ROS–TCP communication.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2quest = q2r_bringup.ros2quest:main',
            'simulationInput = q2r_bringup.simulationInput:main',
            'CheckTCPconnection = q2r_bringup.CheckTCPconnection:main',
            'left_arm_controller = q2r_bringup.left_arm_controller:main',
            'right_arm_controller = q2r_bringup.right_arm_controller:main',
        ],
    },
)
