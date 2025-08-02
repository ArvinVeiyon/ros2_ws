from setuptools import find_packages, setup

package_name = 'rc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # install the single master config
        ('share/' + package_name + '/config', [
            'config/rc_mapping.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'ruamel.yaml',   # used by calibration and sync tools
    ],
    zip_safe=True,
    maintainer='vind',
    maintainer_email='vind@todo.todo',
    description=(
        'ROS 2 package for RC‑based camera switching, '
        'shutdown/reboot, merged RC control, and calibration utilities'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # merged RC‑control node
            'rc_control_node      = rc_control.rc_control_node:main',
            # calibration & tooling
            'channel_calibrator   = rc_control.scripts.channel_calibrator:main',
            'rc_tool              = rc_control.scripts.rc_tool:main',
            'sync_params          = rc_control.scripts.sync_params:main',
        ],
    },
)
