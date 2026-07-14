from setuptools import setup

package_name = 'kaboat_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaboat',
    maintainer_email='taekwon3611@gmail.com',
    description='상시 가동 인식 레이어 (obstacle_detector, yolo_detector)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detector = kaboat_perception.obstacle_detector:main',
            'yolo_detector = kaboat_perception.yolo_detector:main',
        ],
    },
)
