from glob import glob

from setuptools import find_packages, setup

package_name = 'course_launch_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics course',
    maintainer_email='course@example.com',
    description='Lesson 04.10: Python launch files with arguments, includes, remapping, namespaces and conditions.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
