from setuptools import find_packages, setup

package_name = 'vision_streaming'

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
    description='ROS 2 package for vision streaming with FFmpeg and camera switching',
    license='Apache License 2.0',  # Example license, update as needed
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_streaming_node = vision_streaming.vision_streaming_node:main',  # Entry point for the node
        ],
    },
)
