from glob import glob

from setuptools import find_packages, setup

package_name = 'course_params_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics course',
    maintainer_email='course@example.com',
    description='Lesson 04.09: declaring, describing, loading (YAML) and validating ROS 2 parameters.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'battery_health_params = course_params_examples.battery_health_params:main',
        ],
    },
)
