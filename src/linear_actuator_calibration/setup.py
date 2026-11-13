from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'linear_actuator_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ingolf',
    maintainer_email='christian.clt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_talker = linear_actuator_calibration.joint_state_publisher:main',
            'calibration_sequence = linear_actuator_calibration.calibration_sequence:main',
        ],
    },
)
