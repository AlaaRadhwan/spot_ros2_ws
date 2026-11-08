from setuptools import find_packages, setup

package_name = "spot_eval"

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
    maintainer="alaa-radhwan",
    maintainer_email="alaa.radhwanmustafa@gmail.com",
    description="Automated evaluation tools for the Spot V3 Gazebo deployment.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gazebo_eval = spot_eval.gazebo_eval:main",
        ],
    },
)
