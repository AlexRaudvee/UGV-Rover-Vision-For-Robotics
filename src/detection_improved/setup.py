from glob import glob

from setuptools import find_packages, setup

package_name = 'detection_improved'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='nikola.petrov@student.uva.nl',
    description='Improved depth and LiDAR detectors with their own histograms and gaps, '
                'as an alternative to object_detection, obstacle_detection and polar_hist.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'depth_detector = detection_improved.detector:main',
        'lidar_detector = detection_improved.detector:main_lidar',
        'histogram = detection_improved.histogram_node:main',
    ]},
)
