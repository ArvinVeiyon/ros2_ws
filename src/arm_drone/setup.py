from setuptools import setup

package_name = 'arm_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package to arm/disarm PX4 drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_disarm = arm_drone.arm_disarm:main',
        ],
    },
)
