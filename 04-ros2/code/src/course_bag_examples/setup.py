from setuptools import find_packages, setup

package_name = 'course_bag_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics course',
    maintainer_email='course@example.com',
    description='Lesson 04.14: reading rosbag2 (MCAP) files from Python for replay tests.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bag_replay_check = course_bag_examples.bag_replay_check:main',
        ],
    },
)
