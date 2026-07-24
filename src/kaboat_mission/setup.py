from setuptools import setup

package_name = 'kaboat_mission'

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
    description='mission_manager (behavior 완료 + 목표점 2m 전환 규칙)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_manager = kaboat_mission.mission_manager:main',
        ],
    },
)
