from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'moveon_control'

srv_files = glob('srv/*.srv')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), srv_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='durmaz',
    maintainer_email='b190101023@subu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering = moveon_control.steering:main',
            'anglesensor = moveon_control.anglesensor:main',
            'motor = moveon_control.motor:main',
            'steer_angle = moveon_control.steer_angle:main',
            'robot_control_node = moveon_control.duz:main',
            'robot_control = moveon_control.robot_control:main'
        ],
    },
)
