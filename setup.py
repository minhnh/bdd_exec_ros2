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
        (os_join("share", package_name, "launch"), glob(os_join("launch", "*.*"))),
        (os_join("share", package_name, "models"), glob(os_join("models", "*.*"))),
        (
            os_join("share", package_name, "models", "robbdd"),
            glob(os_join("models", "robbdd", "*.*")),
        ),
    ],
    package_data={
        package_name: [
            "py.typed",
            "web/index.html",
            "web/styles.css",
            "web/app.mjs",
            "web/timeline.mjs",
        ]
    },
    install_requires=[
        "setuptools",
        "aiohttp",
        "pyside6",
        "numpy",
        "scipy>=1.16",
        # Requires source install from GitHub
        "rdf_utils",
        "bdd_dsl",
        "scene-dsl",
        "robbdd",
    ],
    zip_safe=False,
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
            "sim_interface_test = bdd_exec_ros2.executables.sim_interface_test:main",
            "mockup_behaviour_node = bdd_exec_ros2.executables.mockup_behaviour_node:main",
            "visualizer = bdd_exec_ros2.executables.visualizer:main",
            "web_visualizer = bdd_exec_ros2.executables.web_visualizer:main",
        ],
    },
)
