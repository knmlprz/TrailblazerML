from setuptools import find_packages, setup
from glob import glob
package_name = 'rover_teleop_twist_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walkowiczf',
    maintainer_email='fillio00@wp.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_node = rover_teleop_twist_joy.teleop_twist_node:main'
        ],
    },
)
