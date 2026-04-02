from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'chaska_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gr',
    maintainer_email='gr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_velocity_node = chaska_arm_controller.joint_velocity_node:main',
            'joystick_controller = chaska_arm_controller.joystick_controller_node:main',
            'move_rmd_motor = chaska_arm_controller.move_rmd_motor:main',
            'move_nemas = chaska_arm_controller.move_nemas:main',
            'joystick_teleoperado = chaska_arm_controller.joystick_teleoperado:main',
            'move_effector_esp32 = chaska_arm_controller.move_effector_esp32:main',
            'control_total = chaska_arm_controller.control_total:main',
        ],
    },
)
