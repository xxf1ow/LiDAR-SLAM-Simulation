from setuptools import setup

package_name = 'sim_obstacles'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dynamic_obstacles.launch.py']),
        ('share/' + package_name + '/config', ['config/obstacles.yaml']),
        ('share/' + package_name + '/models', ['models/obstacle.sdf.in']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xxf1ow',
    maintainer_email='20twenty.degree@gmail.com',
    description='仿真专用动态障碍物（sim-only）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_driver = sim_obstacles.obstacle_driver:main',
        ],
    },
)
