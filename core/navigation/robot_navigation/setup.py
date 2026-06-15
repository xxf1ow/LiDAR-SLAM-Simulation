import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxf1ow',
    maintainer_email='20twenty.degree@gmail.com',
    description='5e Nav2 最小打通：先验图 2D 化、twist_stamper、nav2 参数与 bringup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pcd_to_occupancy = robot_navigation.pcd_to_occupancy:main',
            'twist_stamper = robot_navigation.twist_stamper:main',
        ],
    },
)
