import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'spot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alaa-radhwan',
    maintainer_email='alaa.radhwanmustafa@gmail.com',
    description='Keyboard and PS5 gamepad teleoperation publishing /cmd_vel for Spot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ps5_teleop = spot_teleop.ps5_teleop:main",
        ],
    },
)
