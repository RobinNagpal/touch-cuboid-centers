from glob import glob

from setuptools import find_packages, setup

package_name = "work_cell"

# Each subject folder carries its own description and configuration next to the
# code that uses it, so the install list is grouped the same way.
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/arm", glob("work_cell/arm/*.xacro") + glob("work_cell/arm/*.yaml")),
        ("share/" + package_name + "/arm/camera", glob("work_cell/arm/camera/*.xacro")),
        ("share/" + package_name + "/table", glob("work_cell/table/*.sdf")),
        ("share/" + package_name + "/cuboids", glob("work_cell/cuboids/*.sdf")),
        (
            "share/" + package_name + "/world",
            glob("work_cell/world/*.sdf")
            + glob("work_cell/world/*.yaml")
            + glob("work_cell/world/*.xml")
            + glob("work_cell/world/*.rviz"),
        ),
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
            "touch_cuboids = work_cell.main:main",
        ],
    },
)
