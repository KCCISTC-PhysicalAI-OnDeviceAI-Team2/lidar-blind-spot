import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'blindspot_calib'

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
        (os.path.join('share', package_name, 'calib'), glob('calib/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sunghun Jung',
    maintainer_email='283126466+ning00120-pixel@users.noreply.github.com',
    description='Floor homography calibration and camera-to-floor conversion',
    license='AGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'extract_frame = blindspot_calib.extract_frame:main',
            'pick_points = blindspot_calib.pick_points:main',
            'compute_homography = blindspot_calib.compute_homography:main',
            'camera_to_floor_node = blindspot_calib.camera_to_floor_node:main',
            'measure_points = blindspot_calib.measure_points:main',
        ],
    },
)
