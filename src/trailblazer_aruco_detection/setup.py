from setuptools import setup

package_name = 'trailblazer_aruco_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/aruco_pipeline_launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Twoje Imię',
    maintainer_email='twoj@email.com',
    description='Aruco + camera launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

