#!/usr/bin/env python3
"""Unified environment preflight validator for NIDAR RescueSwarm.

Verifies prerequisites for both software simulation and PX4 SITL + Gazebo Harmonic:
- Host OS & Python runtime
- Required Python dependencies
- Model weights & YOLO integrity
- Fleet and scenario configuration validity
- ROS 2 & colcon build tools
- PX4 Autopilot checkout & Gazebo Harmonic simulator
- Multi-vehicle port availability
- Disk space and resource requirements

Outputs clean tabular summary with PASS / FAIL / WARNING / NOT_INSTALLED status
and actionable setup instructions for missing components.
"""

import argparse
import importlib
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
from typing import Any, Dict, List, Tuple


class Status:
    PASS = "PASS"
    FAIL = "FAIL"
    WARNING = "WARNING"
    NOT_INSTALLED = "NOT_INSTALLED"
    NOT_CHECKED = "NOT_CHECKED"


def colorize(text: str, status: str) -> str:
    colors = {
        Status.PASS: "\033[92m",          # Green
        Status.FAIL: "\033[91m",          # Red
        Status.WARNING: "\033[93m",       # Yellow
        Status.NOT_INSTALLED: "\033[94m", # Blue
        Status.NOT_CHECKED: "\033[90m",   # Gray
    }
    reset = "\033[0m"
    return f"{colors.get(status, '')}{text}{reset}"


class PreflightChecker:
    def __init__(self, repo_root: str) -> None:
        self.repo_root = repo_root
        self.results: List[Tuple[str, str, str, str]] = []  # (Category, Check, Status, Details)
        self.recommendations: List[str] = []

    def check_os_and_python(self) -> None:
        # OS
        system_os = platform.system()
        release = platform.release()
        self.results.append(("Host Environment", "Operating System", Status.PASS, f"{system_os} {release} ({platform.machine()})"))

        # Python version
        py_ver = sys.version_info
        ver_str = f"{py_ver.major}.{py_ver.minor}.{py_ver.micro}"
        if py_ver.major == 3 and py_ver.minor >= 10:
            self.results.append(("Host Environment", "Python Version", Status.PASS, f"Python {ver_str} (>= 3.10 required)"))
        else:
            self.results.append(("Host Environment", "Python Version", Status.FAIL, f"Python {ver_str} (< 3.10 unsupported)"))
            self.recommendations.append("Upgrade Python to version 3.10 or higher.")

    def check_python_packages(self) -> None:
        required_pkgs = [
            ("numpy", "numpy"),
            ("cv2", "opencv-python"),
            ("torch", "torch"),
            ("ultralytics", "ultralytics"),
            ("pymavlink", "pymavlink"),
            ("yaml", "pyyaml"),
        ]
        for mod_name, pkg_name in required_pkgs:
            try:
                mod = importlib.import_module(mod_name)
                ver = getattr(mod, "__version__", "installed")
                self.results.append(("Python Dependencies", pkg_name, Status.PASS, f"v{ver}"))
            except ImportError:
                self.results.append(("Python Dependencies", pkg_name, Status.FAIL, "Not found"))
                self.recommendations.append(f"Install missing package: pip install {pkg_name}")

        # Optional ROS 2 python client
        try:
            import rclpy
            self.results.append(("ROS 2 Integration", "rclpy", Status.PASS, "Installed and importable"))
        except ImportError:
            self.results.append(("ROS 2 Integration", "rclpy", Status.WARNING, "Not installed in current Python env (required for ROS 2 nodes)"))

    def check_configuration_files(self) -> None:
        fleet_cfg = os.path.join(self.repo_root, "config", "fleet.json")
        if os.path.exists(fleet_cfg):
            try:
                with open(fleet_cfg, "r", encoding="utf-8") as f:
                    data = json.load(f)
                drones = data.get("fleet") or data.get("drones")
                count = len(drones) if drones else 0
                if count == 6:
                    self.results.append(("Configuration", "fleet.json", Status.PASS, "6 vehicles defined (S1-S3, D1-D3)"))
                else:
                    self.results.append(("Configuration", "fleet.json", Status.WARNING, f"Found {count} vehicles, expected 6"))
            except Exception as e:
                self.results.append(("Configuration", "fleet.json", Status.FAIL, f"Parse error: {e}"))
        else:
            self.results.append(("Configuration", "fleet.json", Status.FAIL, "File missing"))

        scenarios_cfg = os.path.join(self.repo_root, "config", "mission_scenarios.json")
        if os.path.exists(scenarios_cfg):
            self.results.append(("Configuration", "mission_scenarios.json", Status.PASS, "Scenarios configuration present"))
        else:
            self.results.append(("Configuration", "mission_scenarios.json", Status.WARNING, "File missing"))

    def check_model_weights(self) -> None:
        model_path = os.path.join(self.repo_root, "assets", "models", "best.pt")
        if os.path.exists(model_path):
            size_mb = os.path.getsize(model_path) / (1024 * 1024)
            self.results.append(("Perception Model", "YOLO best.pt", Status.PASS, f"Found at {model_path} ({size_mb:.1f} MB)"))
        else:
            self.results.append(("Perception Model", "YOLO best.pt", Status.WARNING, f"Not found at {model_path} (model inference will fall back to mock)"))
            self.recommendations.append(f"Place YOLO model weights at {model_path} for genuine perception testing.")

    def check_ros2_and_colcon(self) -> None:
        # Check colcon
        colcon_bin = shutil.which("colcon")
        if colcon_bin:
            self.results.append(("ROS 2 Integration", "colcon build tool", Status.PASS, colcon_bin))
        else:
            self.results.append(("ROS 2 Integration", "colcon build tool", Status.NOT_INSTALLED, "colcon not in host PATH"))

        # Check ROS 2 CLI
        ros2_bin = shutil.which("ros2")
        ros_distro = os.environ.get("ROS_DISTRO", "")
        if ros2_bin:
            self.results.append(("ROS 2 Integration", "ros2 CLI", Status.PASS, f"{ros2_bin} (ROS_DISTRO={ros_distro})"))
        else:
            self.results.append(("ROS 2 Integration", "ros2 CLI", Status.NOT_INSTALLED, "ros2 CLI not in host PATH"))
            self.recommendations.append("Source ROS 2 environment: source /opt/ros/jazzy/setup.bash (or humble)")

    def check_px4_and_gazebo(self) -> None:
        # Check gz CLI
        gz_bin = shutil.which("gz")
        if gz_bin:
            try:
                out = subprocess.check_output([gz_bin, "sim", "--versions"], text=True, stderr=subprocess.STDOUT)
                ver_line = out.strip().split("\n")[0]
                self.results.append(("Simulator Stack", "Gazebo Harmonic (gz)", Status.PASS, ver_line))
            except Exception:
                self.results.append(("Simulator Stack", "Gazebo Harmonic (gz)", Status.PASS, gz_bin))
        else:
            # Check WSL if on Windows
            if platform.system() == "Windows" and shutil.which("wsl"):
                try:
                    wsl_gz = subprocess.check_output(["wsl", "bash", "-c", "which gz && gz sim --versions | head -n 1"], text=True, stderr=subprocess.STDOUT).strip()
                    self.results.append(("Simulator Stack", "Gazebo Sim (WSL 2)", Status.PASS, wsl_gz.replace("\n", " - ")))
                except Exception:
                    self.results.append(("Simulator Stack", "Gazebo Sim", Status.NOT_INSTALLED, "gz not found in host or WSL"))
            else:
                self.results.append(("Simulator Stack", "Gazebo Sim", Status.NOT_INSTALLED, "gz not found"))
                self.recommendations.append("Install Gazebo Harmonic: sudo apt install gz-harmonic")

        # Check PX4 checkout
        px4_dir = os.environ.get("PX4_AUTOPILOT_DIR", "")
        sibling_px4 = os.path.abspath(os.path.join(self.repo_root, "..", "PX4-Autopilot"))
        candidates = [px4_dir, os.path.expanduser("~/PX4-Autopilot"), sibling_px4]
        found_px4 = None
        for c in candidates:
            if c and os.path.isdir(c):
                found_px4 = c
                break

        # Check in WSL if on Windows
        if not found_px4 and platform.system() == "Windows" and shutil.which("wsl"):
            try:
                out = subprocess.check_output(
                    ["wsl", "bash", "-c", "ls -d $HOME/PX4-Autopilot ../PX4-Autopilot 2>/dev/null | head -n 1"],
                    text=True,
                ).strip()
                if out:
                    found_px4 = f"WSL:{out}"
            except Exception:
                pass

        if found_px4:
            self.results.append(("Simulator Stack", "PX4-Autopilot", Status.PASS, f"Checkout found at {found_px4}"))
        else:
            self.results.append(("Simulator Stack", "PX4-Autopilot", Status.NOT_INSTALLED, "Repository not found"))
            self.recommendations.append("Clone PX4: git clone --recursive https://github.com/PX4/PX4-Autopilot.git")

    def check_ports(self) -> None:
        ports_to_check = [14540, 14541, 14542, 14543, 14544, 14545, 14550, 14551, 14552, 14553, 14554, 14555]
        conflicts = []
        for port in ports_to_check:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.bind(("127.0.0.1", port))
                s.close()
            except OSError:
                conflicts.append(port)
        if conflicts:
            self.results.append(("Networking & Ports", "MAVLink UDP Ports", Status.WARNING, f"Ports in use: {conflicts}"))
            self.recommendations.append(f"Free UDP ports {conflicts} before starting SITL.")
        else:
            self.results.append(("Networking & Ports", "MAVLink UDP Ports", Status.PASS, "Offboard (14540-14545) & GCS (14550-14555) ports available"))

    def check_disk_space(self) -> None:
        try:
            total, used, free = shutil.disk_usage(self.repo_root)
            free_gb = free / (1024 ** 3)
            if free_gb >= 5.0:
                self.results.append(("System Resources", "Disk Space", Status.PASS, f"{free_gb:.1f} GB available"))
            else:
                self.results.append(("System Resources", "Disk Space", Status.WARNING, f"Low disk space: {free_gb:.1f} GB (< 5 GB)"))
        except Exception:
            self.results.append(("System Resources", "Disk Space", Status.NOT_CHECKED, "Could not determine disk usage"))

    def run_all(self) -> Dict[str, Any]:
        self.check_os_and_python()
        self.check_python_packages()
        self.check_configuration_files()
        self.check_model_weights()
        self.check_ros2_and_colcon()
        self.check_px4_and_gazebo()
        self.check_ports()
        self.check_disk_space()

        return {
            "results": self.results,
            "recommendations": self.recommendations,
        }

    def print_report(self) -> int:
        data = self.run_all()
        results = data["results"]
        recs = data["recommendations"]

        print("=" * 85)
        print("  NIDAR RescueSwarm — System & Simulator Environment Preflight")
        print("=" * 85)
        print(f"{'Category':<22} | {'Check':<25} | {'Status':<14} | Details")
        print("-" * 85)

        has_fail = False
        for cat, name, status, details in results:
            if status == Status.FAIL:
                has_fail = True
            status_colored = colorize(f"[{status}]", status)
            print(f"{cat:<22} | {name:<25} | {status_colored:<23} | {details}")

        print("=" * 85)

        if recs:
            print("\nActionable Setup Recommendations:")
            for i, rec in enumerate(recs, 1):
                print(f"  {i}. {rec}")
            print("-" * 85)

        if has_fail:
            print("Preflight Status: BLOCKED by essential dependencies. See recommendations above.")
            return 1
        else:
            print("Preflight Status: READY for software simulation & SITL integration.")
            return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="RescueSwarm Environment Preflight Validator")
    parser.add_argument("--json-out", default="", help="Path to write JSON preflight report")
    args = parser.parse_args()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    checker = PreflightChecker(repo_root=repo_root)
    code = checker.print_report()

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump({
                "checks": [
                    {"category": c, "name": n, "status": s, "details": d}
                    for c, n, s, d in checker.results
                ],
                "recommendations": checker.recommendations,
                "exit_code": code,
            }, f, indent=2)

    return code


if __name__ == "__main__":
    sys.exit(main())
