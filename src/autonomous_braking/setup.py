from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autonomous_braking'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'sprites'),
            glob(os.path.join('sprites', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.edu',
    description='Autonomous Emergency Braking System - ADAS Final Project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation = autonomous_braking.simulation:main',
            'visualizer = autonomous_braking.visualizer:main',
        ],
    },
)
