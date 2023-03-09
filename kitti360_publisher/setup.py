import os
from glob import glob
from setuptools import setup

package_name = 'kitti360_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='glob',
    maintainer_email='pgrigor22@gmail.com',
    description='ROS 2 publisher of Kitti-360 dataset.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = kitti360_publisher.kitti360_publisher_node:main',
            'labels_node = kitti360_publisher.labels:main'
        ],
    },
)