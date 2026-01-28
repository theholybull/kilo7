from setuptools import setup
import os
from glob import glob

package_name = "kilo_core"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "systemd"), glob("systemd/*.service") + glob("systemd/*.env")),
        (os.path.join("share", package_name, "hook"), glob("hooks/*.dsv")),
    ],
    install_requires=["setuptools", "paho-mqtt", "pyyaml"],
    zip_safe=True,
    maintainer="theholybull",
    maintainer_email="",
    description="KILO7 core backend nodes (MQTT bridge, safety gate, control, relay kill)",
    license="",
    entry_points={
        "console_scripts": [
            "mqtt_bridge = kilo_core.mqtt_bridge:main",
            "safety_gate = kilo_core.safety_gate:main",
            "control_pwm = kilo_core.control_pwm:main",
            "relay_kill = kilo_core.relay_kill:main",
            "safety_model = kilo_core.safety_model:main",
            "perception_summary = kilo_core.perception_summary:main",
        ],
    },
)
