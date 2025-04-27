from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trailblazer_detections'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # DODAJEMY MODELE
        (os.path.join('share', package_name, 'models'), glob('trailblazer_detections/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walkowiczf',
    maintainer_email='fillio00@wp.pl',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = trailblazer_detections.yolo_node:main'
        ],
    },
)
