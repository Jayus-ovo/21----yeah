from setuptools import find_packages, setup

package_name = 'chassis_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chassis.launch.py']),
        ('share/' + package_name + '/launch', ['launch/medical_competition.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Dev',
    maintainer_email='dev@team.local',
    description='Chassis execution module for vehicle control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_follower = chassis_executor.track_follower:main',
            'obstacle_avoider = chassis_executor.obstacle_avoider:main',
            'bravo_route_conductor = chassis_executor.route_conductor:main',
            'announcement_manager = chassis_executor.announcement_manager:main',
            'positioning_controller = chassis_executor.positioning_controller:main',
        ],
    },
)
