import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pandora_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo@todo.todo',
    description='The pandora_control package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'h_control = pandora_control.h_control:main',
            'ik_server = pandora_control.ik_server:main',
            'set_point = pandora_control.set_point:main',
            'stability_set_point = pandora_control.stability_set_point:main',
            'static_stability = pandora_control.static_stability:main',
            'support_polygon = pandora_control.support_polygon:main',
            'wheel_contacts = pandora_control.wheel_contacts:main',
            'wheel_control = pandora_control.wheel_control:main',
            'world_broadcaster = pandora_control.world_broadcaster:main',
        ],
    },
)
