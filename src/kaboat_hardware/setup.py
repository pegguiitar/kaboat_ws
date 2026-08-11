import os
from glob import glob

from setuptools import setup


package_name = 'kaboat_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaboat',
    maintainer_email='taekwon3611@gmail.com',
    description='KABOAT real sensor bringup and health diagnostics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_health_monitor = kaboat_hardware.sensor_health_monitor:main',
            'odom_tf_broadcaster = kaboat_hardware.odom_tf_broadcaster:main',
            'apriltag_odom = kaboat_hardware.apriltag_odom:main',
        ],
    },
)
