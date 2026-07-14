from setuptools import setup

package_name = 'kaboat_behaviors'

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
    description='미션별 behavior 노드 5개 + 공통 회피-보정 라이브러리',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gate_follower = kaboat_behaviors.gate_follower:main',
            'station_keeper = kaboat_behaviors.station_keeper:main',
            'docking_ctrl = kaboat_behaviors.docking_ctrl:main',
            'search_circler = kaboat_behaviors.search_circler:main',
            'obstacle_planner = kaboat_behaviors.obstacle_planner:main',
        ],
    },
)
