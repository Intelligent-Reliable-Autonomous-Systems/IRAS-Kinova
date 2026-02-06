from setuptools import find_packages, setup

package_name = "iras_realsense"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/test.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/camera_info.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jjewett",
    maintainer_email="jewettje@oregonstate.edu",
    description="An external realsense RGB-D camera for detecting objects in a manipulation workspace",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
