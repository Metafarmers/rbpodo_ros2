"""RB5 URDF joint-limit regression tests."""

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


_DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
_RB5_XACRO = _DESCRIPTION_ROOT / "robots" / "rb5_850e.urdf.xacro"
_RB5_JOINT_LIMITS = _DESCRIPTION_ROOT / "robots" / "rb5_850e" / "joint.yaml"
_ONE_TURN_JOINTS = ("base", "wrist1", "wrist2", "wrist3")


def _expanded_joint_limits():
    """Return runtime limits emitted by the RB5 xacro model."""

    result = subprocess.run(
        ["xacro", str(_RB5_XACRO)],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = ET.fromstring(result.stdout)
    limits = {}
    for joint_name in _ONE_TURN_JOINTS:
        joint = robot.find(f"./joint[@name='{joint_name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        limits[joint_name] = (
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
    return limits


def test_rb5_runtime_limits_include_exact_absolute_one_turn_targets():
    """Generated runtime URDF must accept 0 and both absolute ±2π goals."""

    for lower, upper in _expanded_joint_limits().values():
        assert lower == -math.tau
        assert upper == math.tau
        assert lower <= 0.0 <= upper
        assert lower <= -math.tau <= upper
        assert lower <= math.tau <= upper


def test_rb5_one_turn_joint_yaml_limits_keep_exact_absolute_endpoints():
    """The xacro source must not round a valid ±360° target out of range."""

    joint_config = yaml.safe_load(_RB5_JOINT_LIMITS.read_text(encoding="utf-8"))

    for joint_name in _ONE_TURN_JOINTS:
        limit = joint_config[joint_name]["limit"]
        assert limit["lower"] == -math.tau
        assert limit["upper"] == math.tau


def test_rb5_shoulder_and_elbow_limits_remain_half_turn_ranges():
    """Only the RB5 joints specified as ±360° gain the exact one-turn limit."""

    joint_config = yaml.safe_load(_RB5_JOINT_LIMITS.read_text(encoding="utf-8"))

    for joint_name in ("shoulder", "elbow"):
        limit = joint_config[joint_name]["limit"]
        assert limit["lower"] == -3.14
        assert limit["upper"] == 3.14
