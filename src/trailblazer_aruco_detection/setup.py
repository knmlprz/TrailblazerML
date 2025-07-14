from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trailblazer_aruco_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@example.com',
    description='Aruco detection with DepthAI camera',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
