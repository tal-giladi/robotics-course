from glob import glob

from setuptools import find_packages, setup

package_name = 'course_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics course',
    maintainer_email='course@example.com',
    description='Your own wheel odometry node and an odometry comparison tool for lesson 09.07',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry = course_odometry.wheel_odometry_node:main',
            'odom_compare = course_odometry.odom_compare_node:main',
            'fake_wheels = course_odometry.fake_wheels_node:main',
        ],
    },
)
