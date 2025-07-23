from setuptools import find_packages, setup

package_name = 'diffdrive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pygame', 'numpy'],
    zip_safe=True,
    maintainer='Miguel Marco Valencia',
    maintainer_email='miguel.valencia@uq.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'controller_node = diffdrive_controller.controller_node:main',
        ],
    },
)