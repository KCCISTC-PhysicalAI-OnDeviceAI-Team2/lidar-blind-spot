import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'blindspot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sikim',
    maintainer_email='101053374+sikim000130@users.noreply.github.com',
    description='TODO: Package description',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = blindspot_perception.detector_node:main',
        ],
    },
)
