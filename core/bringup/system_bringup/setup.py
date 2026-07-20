import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'system_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxf1ow',
    maintainer_email='20twenty.degree@gmail.com',
    description='全模块启动 + 启动前跨模块一致性闸门。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'consistency_check = system_bringup.consistency_check:main',
        ],
    },
)
