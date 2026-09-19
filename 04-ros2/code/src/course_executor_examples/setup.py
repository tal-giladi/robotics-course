from setuptools import find_packages, setup

package_name = 'course_executor_examples'

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
    description='Lesson 04.12: executors, callback groups, timers and the synchronous-service-call deadlock.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'battery_status_server = course_executor_examples.battery_status_server:main',
            'supervisor = course_executor_examples.supervisor:main',
            'timer_starvation = course_executor_examples.timer_starvation:main',
        ],
    },
)
