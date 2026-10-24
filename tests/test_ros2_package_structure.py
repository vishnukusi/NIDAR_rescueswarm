"""Tests for ROS 2 package structure, XML, AST syntax, and launch files."""

import ast
import os
import unittest
import xml.etree.ElementTree as ET
import yaml


class TestROS2PackageStructure(unittest.TestCase):
    """Verifies that rescue_swarm_bridge satisfies standard ROS 2 package layout and syntax."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        cls.pkg_dir = os.path.join(cls.repo_root, "ros2_ws", "src", "rescue_swarm_bridge")

    def test_package_xml_valid(self):
        xml_path = os.path.join(self.pkg_dir, "package.xml")
        self.assertTrue(os.path.exists(xml_path), f"Missing {xml_path}")
        tree = ET.parse(xml_path)
        root = tree.getroot()
        self.assertEqual(root.tag, "package")
        name = root.find("name")
        self.assertIsNotNone(name)
        self.assertEqual(name.text.strip(), "rescue_swarm_bridge")
        version = root.find("version")
        self.assertIsNotNone(version)
        self.assertEqual(version.text.strip(), "1.0.0")

    def test_setup_py_valid_ast(self):
        setup_path = os.path.join(self.pkg_dir, "setup.py")
        self.assertTrue(os.path.exists(setup_path), f"Missing {setup_path}")
        with open(setup_path, "r", encoding="utf-8") as f:
            code = f.read()
        parsed = ast.parse(code)
        self.assertIsInstance(parsed, ast.Module)

    def test_resource_marker_exists(self):
        marker_path = os.path.join(self.pkg_dir, "resource", "rescue_swarm_bridge")
        self.assertTrue(os.path.exists(marker_path), f"Missing {marker_path}")

    def test_ros2_modules_ast_syntax(self):
        modules = [
            "__init__.py",
            "vehicle_adapter.py",
            "vehicle_adapter_node.py",
            "simulated_delivery_node.py",
            "perception_bridge_node.py",
            "mission_manager_node.py",
            "gcs_bridge_node.py",
        ]
        pkg_code_dir = os.path.join(self.pkg_dir, "rescue_swarm_bridge")
        for mod in modules:
            mod_path = os.path.join(pkg_code_dir, mod)
            self.assertTrue(os.path.exists(mod_path), f"Missing module: {mod_path}")
            with open(mod_path, "r", encoding="utf-8") as f:
                code = f.read()
            parsed = ast.parse(code, filename=mod)
            self.assertIsInstance(parsed, ast.Module)

    def test_launch_files_ast_syntax(self):
        launch_dir = os.path.join(self.pkg_dir, "launch")
        launch_files = ["six_drone_mission.launch.py", "px4_sitl_multi_vehicle.launch.py"]
        for lf in launch_files:
            lf_path = os.path.join(launch_dir, lf)
            self.assertTrue(os.path.exists(lf_path), f"Missing launch file: {lf_path}")
            with open(lf_path, "r", encoding="utf-8") as f:
                code = f.read()
            parsed = ast.parse(code, filename=lf)
            self.assertIsInstance(parsed, ast.Module)

    def test_ros2_params_yaml_valid(self):
        yaml_path = os.path.join(self.pkg_dir, "config", "ros2_params.yaml")
        self.assertTrue(os.path.exists(yaml_path), f"Missing {yaml_path}")
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        self.assertIn("mission_manager_node", data)
        self.assertIn("simulated_delivery_node", data)
        self.assertIn("perception_bridge_node", data)


if __name__ == "__main__":
    unittest.main()
