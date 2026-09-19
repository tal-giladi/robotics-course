from glob import glob

from setuptools import find_packages, setup

package_name = 'karmel_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics course',
    maintainer_email='course@example.com',
    description='Vision in ROS 2 and detection-to-map-position for lessons 13.15-13.16',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_camera = karmel_vision.fake_camera:main',
            'detector_node = karmel_vision.detector_node:main',
            'latency_probe = karmel_vision.latency_probe:main',
            'detection_3d_node = karmel_vision.detection_3d_node:main',
            'object_map_node = karmel_vision.object_map_node:main',
        ],
    },
)
