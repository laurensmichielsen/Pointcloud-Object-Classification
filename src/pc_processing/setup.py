from setuptools import find_packages, setup

package_name = "pc_processing"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        # runtime Python dependencies - prefer installing these with rosdep or pip in venv
        "numpy",
        "scikit-learn",
        "open3d",
    ],
    zip_safe=True,
    maintainer="Laurens Michielsen",
    maintainer_email="laurens.michielsen9@gmail.com",
    description="Point cloud preprocessing (filtering, ground removal, clustering, cone reconstruction).",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        # Uncomment / edit the next line to create a console script for your node
        "console_scripts": ["preprocessing_node = pc_processing.pre_processing_node:main"],
    },
    python_requires=">=3.8",
)
