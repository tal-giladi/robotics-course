from pathlib import Path

from setuptools import find_packages, setup

package_name = "so101_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [str(p) for p in Path("launch").glob("*.launch.py")]),
        (f"share/{package_name}/config", [str(p) for p in Path("config").glob("*.yaml")]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics course",
    maintainer_email="course@example.com",
    description="SO-101 bringup: controllers, launch files and the Python servo driver.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_bridge = so101_bringup.joint_bridge:main",
        ],
    },
)
