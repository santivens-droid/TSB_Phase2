from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tsb_recruitment_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Técnico Solar Boat',
    maintainer_email='tecnico.solarboat@gmail.com',
    description='HoloOcean interface package for TSB Recruitment Project',
    license='TODO',
    entry_points={
        'console_scripts': [
            'tsb_recruitment_sim_node = tsb_recruitment_sim.tsb_recruitment_sim_node:main'
        ],
    },
)
