import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'origincar_v2_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='origincar',
    maintainer_email='dev@origincar.local',
    description='V2 protocol bridge for OriginCar Ackermann + RDK X5',
    license='MIT',
    entry_points={
        'console_scripts': [
            'v2_serial_node = origincar_v2_bridge.v2_serial_node:main',
            'nonholonomic_node = origincar_v2_bridge.nonholonomic_node:main',
            'zupt_monitor = origincar_v2_bridge.zupt_monitor:main',
            'waypoint_nav_node = origincar_v2_bridge.waypoint_nav_node:main',
            'qr_announcer_node = origincar_v2_bridge.qr_announcer_node:main',
        ],
    },
)
