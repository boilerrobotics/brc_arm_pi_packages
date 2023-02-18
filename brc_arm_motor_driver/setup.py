from setuptools import setup
import os
from glob import glob

package_name = "brc_arm_motor_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andy",
    maintainer_email="ding258@purdue.edu",
    description="Controls motor drivers on BRC robot arm",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "brc_arm_motor_driver = brc_arm_motor_driver.brc_arm_motor_driver:main"
        ],
    },
)
