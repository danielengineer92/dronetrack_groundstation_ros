from setuptools import find_packages, setup

package_name = 'dronetrack_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
