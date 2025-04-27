from setuptools import find_packages, setup

package_name = 'trailblazer_detections'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/trailblazer_detections'
        ]),
        ('share/trailblazer_detections', [
            'package.xml'
        ]),
        ('share/trailblazer_detections/launch', [
            'launch/yolo_node.launch.py'
        ]),
        ('share/trailblazer_detections/models', [
            'trailblazer_detections/models/urc2024.pt'
        ]),
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
