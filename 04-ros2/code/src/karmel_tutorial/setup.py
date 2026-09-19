from setuptools import find_packages, setup

package_name = 'karmel_tutorial'

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
    maintainer='karmel course',
    maintainer_email='student@example.com',
    description='The battery + drive example built up across ROS 2 lessons 04.04-04.08.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # 04.04 topics
            'battery_sim = karmel_tutorial.battery_sim:main',
            'battery_monitor = karmel_tutorial.battery_monitor:main',
            # 04.06 custom message (+ 04.07 service)
            'battery_health = karmel_tutorial.battery_health:main',
            # 04.07 services
            'set_threshold_client = karmel_tutorial.set_threshold_client:main',
            'deadlock_demo = karmel_tutorial.deadlock_demo:main',
            # 04.08 actions
            'drive_distance_server = karmel_tutorial.drive_distance_server:main',
            'drive_distance_client = karmel_tutorial.drive_distance_client:main',
        ],
    },
)
