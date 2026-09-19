from setuptools import find_packages, setup

package_name = 'course_qos_examples'

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
    description='Lesson 04.11: QoS reliability, durability, history and incompatibility demos.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voltage_publisher_qos = course_qos_examples.voltage_publisher_qos:main',
            'voltage_listener_qos = course_qos_examples.voltage_listener_qos:main',
            'battery_spec_publisher = course_qos_examples.battery_spec_publisher:main',
            'slow_health_listener = course_qos_examples.slow_health_listener:main',
        ],
    },
)
