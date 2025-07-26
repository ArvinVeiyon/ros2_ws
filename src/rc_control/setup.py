from setuptools import find_packages, setup

package_name = 'rc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # our config folder
        ('share/' + package_name + '/config',
         ['config/rc_mapping.yaml']),
    ],
    install_requires=[
        'setuptools',
        'ruamel.yaml',   # needed by the calibrator
    ],
    zip_safe=True,
    maintainer='vind',
    maintainer_email='vind@todo.todo',
    description=(
        'ROS 2 package for RC‑based camera switching, '
        'shutdown/reboot, and channel calibration utility'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_sw_node       = rc_control.camera_sw_node:main',
            'shutdown_reboot_node = rc_control.shutdown_reboot_node:main',
            'channel_calibrator   = rc_control.scripts.channel_calibrator:main',
        ],
    },
)
