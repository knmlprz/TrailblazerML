from setuptools import setup

package_name = 'trailblazer_detections'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=['launch.yolo_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name
        ]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/yolo_node.launch.py'
        ]),
        ('share/' + package_name + '/models', [
            'models/urc2024.pt'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walkowiczf',
    maintainer_email='fillio00@wp.pl',
    description='YOLO object detection node for ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = launch.yolo_node:main'
        ],
    },
)
