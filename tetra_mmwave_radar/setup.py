import os
from glob import glob

from setuptools import setup

package_name = 'tetra_mmwave_radar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johny',
    maintainer_email='johny@hyulimatech.co.kr',
    description='Experimental driver for the DFRobot SEN0395 24GHz mmWave presence radar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mmwave_radar_node = tetra_mmwave_radar.mmwave_radar_node:main',
        ],
    },
)
