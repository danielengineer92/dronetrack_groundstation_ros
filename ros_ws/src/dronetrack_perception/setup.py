import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dronetrack_perception'

# Find model files relative to repo root (two levels up from this setup.py).
_repo_models = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'models')
_model_files = glob(os.path.join(_repo_models, '*.sdf')) if os.path.isdir(_repo_models) else []

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), _model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DroneTrack',
    maintainer_email='danielgatesf@gmail.com',
    description='Laptop-side YOLO inference for the split DroneTrack architecture.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = dronetrack_perception.yolo_node:main',
            'gz_cam_republisher = dronetrack_perception.gz_cam_republisher:main',
            'target_mover_node = dronetrack_perception.target_mover_node:main',
        ],
    },
)
