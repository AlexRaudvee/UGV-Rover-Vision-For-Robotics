from glob import glob

from setuptools import find_packages, setup

package_name = 'l2s3_curling'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Lab 2 Session 3: curling target planning for the UGV Rover.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'path_planner = l2s3_curling.astar_node:main',
            'obstacle_map = l2s3_curling.obstacle_map_node:main',
            'path_follower_preview = l2s3_curling.path_follow_preview:main',
            'path_follower = l2s3_curling.path_follower_node:main',
        ],
    },
)
