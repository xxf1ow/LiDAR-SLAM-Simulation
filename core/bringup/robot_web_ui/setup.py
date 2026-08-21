import os

from setuptools import find_packages, setup


package_name = "robot_web_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "web"),
            ["robot_web_ui/web/index.html", "robot_web_ui/web/map_view.js"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xxf1ow",
    maintainer_email="20twenty.degree@gmail.com",
    description="Neutral mobile robot Web controls",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robot_web_ui = robot_web_ui.web_ui_node:main",
        ],
    },
)
