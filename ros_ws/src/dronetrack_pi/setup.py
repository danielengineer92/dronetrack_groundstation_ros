import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dronetrack_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config/ is staged from the repo-level configs/ by scripts/setup_pi.sh.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DroneTrack',
    maintainer_email='danielgatesf@gmail.com',
    description='Pi-side detection gate and ground-station watchdog for the split DroneTrack architecture.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detection_gate_node = dronetrack_pi.detection_gate_node:main',
            'ground_station_watchdog_node = dronetrack_pi.ground_station_watchdog_node:main',
        ],
    },
)
