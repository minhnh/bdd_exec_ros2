from glob import glob
from os.path import join as os_join
from setuptools import find_packages, setup

package_name = "bdd_exec_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os_join("share", "ament_index", "resource_index", "packages"),
            [os_join("resource", package_name)],
        ),
        (os_join("share", package_name), ["package.xml"]),
        (os_join("share", package_name, "config"), glob(os_join("config", "*.yaml"))),
        (os_join("share", package_name, "launch"), glob(os_join("launch", "*"))),
        (os_join("share", package_name, "models"), glob(os_join("models", "*"))),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools", "pyside6"],
    zip_safe=True,
    maintainer="Minh Nguyen",
    maintainer_email="1168534+minhnh@users.noreply.github.com",
    description="Execution setup for bdd-dsl with ROS2",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "bdd_coordination_node = bdd_exec_ros2.executables.bdd_coordination_node:main",
            "mockup_behaviour_node = bdd_exec_ros2.executables.mockup_behaviour_node:main",
            "visualizer = bdd_exec_ros2.executables.visualizer:main",
        ],
    },
)
