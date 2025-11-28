from setuptools import setup

package_name = "rosbag_frame_loader"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Laurens",
    maintainer_email="laurens.michielsen9@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "db3_frame_loader = rosbag_frame_loader.db3_frame_loader_node:main",
        ],
    },
)

