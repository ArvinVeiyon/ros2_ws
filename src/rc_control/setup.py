from setuptools import find_packages, setup

package_name = 'rc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vind',
    maintainer_email='vind@todo.todo',
    description='ROS 2 package for RC-based camera switching',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_sw_node = rc_control.camera_sw_node:main',
            'shutdown_reboot_node = rc_control.shutdown_reboot_node:main',
        ],
    },
)
