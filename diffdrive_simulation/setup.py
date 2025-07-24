from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diffdrive_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='valencimm',
    maintainer_email='miguel.valencia@uq.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'simulation_node = diffdrive_simulation.simulation_node:main',
            'noisy_odom_node = diffdrive_simulation.noisy_odom_node:main'
        ],
    },
)
