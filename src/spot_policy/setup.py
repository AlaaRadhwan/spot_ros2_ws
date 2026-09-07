from glob import glob
import os

from setuptools import find_packages, setup


package_name = "spot_policy"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        exclude=["test"],
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "models",
            ),
            glob("models/*"),
        ),
    ],
    install_requires=[
        "setuptools",
        "numpy",
    ],
    zip_safe=True,
    maintainer="alaa-radhwan",
    maintainer_email="todo@example.com",
    description="ROS 2 deployment node for the Spot RL policy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "observation_monitor = "
                "spot_policy.observation_monitor:main"
            ),
            "policy_dry_run = spot_policy.policy_dry_run:main",
            "policy_controller = spot_policy.policy_controller:main",
        ],
    },
)