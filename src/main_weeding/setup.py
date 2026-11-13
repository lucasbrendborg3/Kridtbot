from setuptools import setup

package_name = 'main_weeding'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunnar',
    maintainer_email='lucas@brendborg.dk',
    description='Stop-and-go weeding mission for kridtbot',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mission = main_weeding.mission:main',
        ],
    },
)
