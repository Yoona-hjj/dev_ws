from setuptools import find_packages, setup

package_name = 'cockpit_feedback'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='origincar',
    maintainer_email='dev@origincar.local',
    description='Display and scene-recognition feedback nodes for the hybrid navigation stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'race_status_display = cockpit_feedback.race_status_display:main',
            'scene_vlm_client = cockpit_feedback.scene_vlm_client:main',
        ],
    },
)
