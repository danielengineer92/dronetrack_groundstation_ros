import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dronetrack_groundstation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config/ is staged from the repo-level configs/ by scripts/setup_groundstation.sh.
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DroneTrack',
    maintainer_email='danielgatesf@gmail.com',
    description='Laptop ground-station bringup for the split DroneTrack architecture.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'heartbeat_node = dronetrack_groundstation.heartbeat_node:main',
        ],
    },
)
