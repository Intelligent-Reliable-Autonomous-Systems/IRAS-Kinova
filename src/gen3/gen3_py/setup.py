import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gen3_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "robot"), glob("robot/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="will-solow",
    maintainer_email="soloww@oregonstate.edu",
    description="Python package for Kinova arm",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "ee_pub = gen3_py.ee_publisher:main",
            "robot_info=gen3_py.robot_info:main",
            "body_pose=gen3_py.link_pose_publisher:main",
            "jacobian_pub=gen3_py.jacobian_pub:main",
            "vel_integrator=gen3_py.velocity_integrator:main",
        ],
    },
)
