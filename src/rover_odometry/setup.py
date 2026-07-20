from setuptools import find_packages, setup

package_name = 'rover_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # wheel/vehicle parameters
        ('share/' + package_name + '/config', [
            'config/rover_odometry.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vind',
    maintainer_email='vind@todo.todo',
    description=(
        'Differential wheel odometry from VESC UAVCAN ESC status '
        '(/fmu/out/esc_status) for the Vind-Roz rover'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = rover_odometry.wheel_odometry_node:main',
        ],
    },
)
