"""Install histogram nodes, launches, and Vizanti layout."""

from glob import glob
from setuptools import find_packages, setup

setup(
    name='polar_hist', version='0.2.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/polar_hist']),
        ('share/polar_hist', ['package.xml', 'README.md', 'LICENSE']),
        ('share/polar_hist/launch', glob('launch/*.launch.py')),
        ('share/polar_hist/config', glob('config/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='jetson', maintainer_email='nikola.petrov@student.uva.nl',
    description='Live polar histograms and measured 35 cm passage candidates.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': ['histogram = polar_hist.node:main']},
)
