from setuptools import find_packages, setup
import os
from glob import glob

package_name = "warehouse_sim"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
]

config_dir = ""

for dirpath, dirnames, filenames in os.walk(config_dir):
    for f in filenames:
        full_path = os.path.join(dirpath, f)
        install_path = os.path.join("share", package_name, dirpath)
        data_files.append((install_path, [full_path]))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Will Solow",
    maintainer_email="soloww@oregonstate.edu",
    description="Warehouse package for kinova arm and jackal testing",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_relay = warehouse_sim.tf_relay:main",
        ],
    },
)
