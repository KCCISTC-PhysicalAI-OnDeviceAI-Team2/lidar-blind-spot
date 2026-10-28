import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'blindspot_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sunghun Jung',
    maintainer_email='283126466+ning00120-pixel@users.noreply.github.com',
    description='Cluster LiDAR scan points into floor obstacles',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_cluster_node = blindspot_lidar.lidar_cluster_node:main',
        ],
    },
)
