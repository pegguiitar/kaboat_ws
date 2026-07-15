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
    description='인식 레이어 (occupancy_grid·buoy_detector 상시 / dock_mark_detector 는 도킹 state 게이팅)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'occupancy_grid = kaboat_perception.occupancy_grid:main',
            'buoy_detector = kaboat_perception.buoy_detector:main',
            'dock_mark_detector = kaboat_perception.dock_mark_detector:main',
        ],
    },
)
