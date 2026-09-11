from glob import glob

from setuptools import find_packages, setup

package_name = "table_assembly"

# The robot model and the simulated world carry their own description and
# configuration next to the code that uses them, so the install list is grouped
# the same way.
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/arm",
            glob("table_assembly/arm/*.xacro") + glob("table_assembly/arm/*.yaml"),
        ),
        ("share/" + package_name + "/arm/camera", glob("table_assembly/arm/camera/*.xacro")),
        (
            "share/" + package_name + "/world",
            glob("table_assembly/world/*.sdf")
            + glob("table_assembly/world/*.yaml")
            + glob("table_assembly/world/*.xml")
            + glob("table_assembly/world/*.rviz"),
        ),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robin Nagpal",
    maintainer_email="robinnagpal.tiet@gmail.com",
    description="Measure a table top, stand four legs where it needs them, and put the top on them.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "assemble_table = table_assembly.main:main",
        ],
    },
)
