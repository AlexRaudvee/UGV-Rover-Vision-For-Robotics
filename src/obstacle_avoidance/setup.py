"""Install the avoidance node, its launch file and its settings."""

from glob import glob
from setuptools import find_packages, setup

setup(
    name='obstacle_avoidance', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/obstacle_avoidance']),
        ('share/obstacle_avoidance', ['package.xml', 'README.md']),
        ('share/obstacle_avoidance/launch', glob('launch/*.launch.py')),
        ('share/obstacle_avoidance/config', glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Soham Chaudhuri', maintainer_email='sohamchaudhuri25@gmail.com',
    description='Drive around black obstacles and return to the original path.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': ['avoidance = obstacle_avoidance.node:main']},
)
