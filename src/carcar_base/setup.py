from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'carcar_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carcar maintainers',
    maintainer_email='maintainer@example.com',
    description='ROS 2 hardware boundary for the Carcar mobile base.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosmaster_node = carcar_base.rosmaster_node:main',
            'two_motor_test_node = carcar_base.two_motor_test_node:main',
        ],
    },
)
