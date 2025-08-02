from setuptools import find_packages, setup

package_name = 'optical_flow'

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
    description='ROS 2 package for optical flow calculation and publishing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optical_flow_node = optical_flow.optical_flow_node:main'
        ],
    },
)
