import glob

from setuptools import find_packages, setup

package_name = "iras_viz"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/table_scene.py"],
        ),
        (
            "share/" + package_name + "/data/apriltag",
            glob.glob("data/apriltag/*.png"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jjewett",
    maintainer_email="jewettje@oregonstate.edu",
    description="Visualization tools for the IRAS project",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "table_scene = iras_viz.table_scene_node:main",
        ],
    },
)
