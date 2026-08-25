from setuptools import find_packages, setup


package_name = "auto_aim_tools"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hj",
    maintainer_email="hj@todo.todo",
    description="Read-only ROS 2 input preflight diagnostics for auto aim.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros_input_preflight = auto_aim_tools.ros_input_preflight:main",
        ],
    },
)
