from glob import glob
from setuptools import find_packages, setup

setup(name='obstacle_avoidance_adv',version='0.1.0',packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages',['resource/obstacle_avoidance_adv']),
                  ('share/obstacle_avoidance_adv',['package.xml','README.md']),
                  ('share/obstacle_avoidance_adv/config',glob('config/*.yaml')),
                  ('share/obstacle_avoidance_adv/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,maintainer='jetson',
      maintainer_email='nikola.petrov@student.uva.nl',description='LiDAR-first quarter-turn obstacle bypass with gyro feedback',
      license='Apache-2.0',entry_points={'console_scripts':['controller = obstacle_avoidance_adv.node:main']})
