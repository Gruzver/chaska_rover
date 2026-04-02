from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gr',
    maintainer_email='gruzver.phocco@pucp.edu.pe',
    description='Swerve / differential / Ackermann drive controller for Chaska rover',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'swerve_node = rover_controller.swerve_node:main',
            'joy_mode_switcher = rover_controller.joy_mode_switcher:main',
        ],
    },
)
