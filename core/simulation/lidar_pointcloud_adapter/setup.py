from setuptools import find_packages, setup

package_name = 'lidar_pointcloud_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxf1ow',
    maintainer_email='20twenty.degree@gmail.com',
    description='Gz gpu_lidar 组织化点云 → Velodyne 风格(加 ring/time)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'adapter_node = lidar_pointcloud_adapter.adapter_node:main',
        ],
    },
)
