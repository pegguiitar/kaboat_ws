from setuptools import setup

package_name = 'kaboat_control'

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
    description='cmd_mux (state 기반 명령 선택 + 워치독 페일세이프)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_mux = kaboat_control.cmd_mux:main',
        ],
    },
)
