import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rescue_swarm_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NIDAR RescueSwarm Team",
    maintainer_email="dev@nidar-rescueswarm.org",
    description="ROS 2 integration bridge and autonomy coordination package for NIDAR RescueSwarm 6-drone PX4 SITL + Gazebo simulation.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager_node = rescue_swarm_bridge.mission_manager_node:main",
            "vehicle_adapter_node = rescue_swarm_bridge.vehicle_adapter_node:main",
            "perception_bridge_node = rescue_swarm_bridge.perception_bridge_node:main",
            "simulated_delivery_node = rescue_swarm_bridge.simulated_delivery_node:main",
            "gcs_bridge_node = rescue_swarm_bridge.gcs_bridge_node:main",
        ],
    },
)
