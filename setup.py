import os
from glob import glob

from setuptools import find_packages, setup


package_name = "hand_nucleo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="pib",
    maintainer_email="team@pib.rocks",
    description="ROS2 bridge for the PIB hand Nucleo VCP serial protocol",
    license="Agpl3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hand_nucleo_bridge = hand_nucleo.bridge:main",
        ],
    },
)
