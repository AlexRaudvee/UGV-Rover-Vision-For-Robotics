from glob import glob

from setuptools import find_packages, setup

package_name = 'marker_localization'

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
    maintainer_email='jetson@todo.todo',
    description='Lab 2: AprilTag/ArUco marker detection and localization for the UGV Rover.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'detector = marker_localization.detector_node:main',
            'scanner = marker_localization.scanner_node:main',
            'evaluate_detection = marker_localization.evaluate:console_main',
        ],
    },
)
