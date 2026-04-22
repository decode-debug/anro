"""Plik setup.py dla naszego pakietu lab2."""
import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'lab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikolaj',
    maintainer_email='mikolaj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_controller = lab2.turtle_controller:main',
            'dobot_move = lab2.dobot_move:main',
                'dobot_tower = lab2.dobot_tower:main',
            'dobot_print_position = lab2.dobot_print_position:main',
        ],
    },
)
