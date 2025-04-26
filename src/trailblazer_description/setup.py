from setuptools import find_packages, setup
<<<<<<< HEAD
from glob import glob
=======
import glob
>>>>>>> main

package_name = 'trailblazer_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
<<<<<<< HEAD
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/urdf/include', glob('urdf/include/*.xacro')),
        ('share/' + package_name + '/meshes', glob('meshes/*.STL')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
=======
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/description', glob.glob('description/*.xacro')),
        ('share/' + package_name + '/meshes', glob.glob('meshes/*.stl')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
>>>>>>> main
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walkowiczf',
    maintainer_email='fillio00@wp.pl',
    description='TODO: Package description',
<<<<<<< HEAD
    entry_points={
        'console_scripts': [
        ],
=======
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
>>>>>>> main
    },
)
