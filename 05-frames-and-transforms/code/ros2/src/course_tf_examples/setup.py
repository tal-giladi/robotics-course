from glob import glob

from setuptools import find_packages, setup

package_name = 'course_tf_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics course',
    maintainer_email='course@example.com',
    description='TF2, URDF, RViz and TF debugging examples for lessons 05.07-05.11',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'karmel_static_tf = course_tf_examples.karmel_static_tf:main',
            'fake_odometry = course_tf_examples.fake_odometry:main',
            'fake_localization = course_tf_examples.fake_localization:main',
            'bottle_listener = course_tf_examples.bottle_listener:main',
            'time_travel = course_tf_examples.time_travel:main',
            'wait_for_transform = course_tf_examples.bottle_listener:wait_for_transform_demo',
            'urdf_chain = course_tf_examples.urdf_math:main',
            'bottle_detector = course_tf_examples.bottle_detector:main',
            'bottle_to_map = course_tf_examples.bottle_to_map:main',
            'fake_scan = course_tf_examples.fake_scan:main',
        ],
    },
)
