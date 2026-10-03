from glob import glob

from setuptools import find_packages, setup


package_name = "arp_assignment"


setup(
    name=package_name,

    version="0.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/"
                + package_name
            ],
        ),

        (
            "share/"
            + package_name,

            ["package.xml"],
        ),

        (
            "share/"
            + package_name
            + "/launch",

            glob("launch/*.launch.py"),
        ),

        (
            "share/"
            + package_name
            + "/rviz",

            glob("rviz/*.rviz"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="vikas",

    maintainer_email="vikas@example.com",

    description=(
        "ARP Assignment 1 - "
        "Backward Value Iteration"
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest"
    ],

    entry_points={
        "console_scripts": [
            (
                "navigation_node = "
                "arp_assignment.navigation_node:main"
            ),
        ],
    },
)