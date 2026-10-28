from glob import glob

from setuptools import find_packages, setup

package_name = 'blindspot_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksj',
    maintainer_email='323469589+ksj-0609@users.noreply.github.com',
    description='Draws detection boxes on camera images',
    license='AGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'overlay_node = blindspot_viz.overlay_node:main',
        ],
    },
)
