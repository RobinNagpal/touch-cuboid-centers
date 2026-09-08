from glob import glob

from setuptools import find_packages, setup

package_name = "cuboid_cell"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robin Nagpal",
    maintainer_email="robinnagpal.tiet@gmail.com",
    description="Find the biggest face of each cuboid on a table and touch its centre.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "touch_cuboids = cuboid_cell.main:main",
        ],
    },
)
