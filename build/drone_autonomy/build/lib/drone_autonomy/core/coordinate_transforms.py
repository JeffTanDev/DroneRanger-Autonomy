"""
Coordinate transformation utilities for drone navigation.

Handles conversions between different coordinate systems:
- ENU (East-North-Up) - Used by Gazebo and ROS
- NED (North-East-Down) - Used by PX4 and ArduPilot
"""

import math
from typing import Tuple
import numpy as np


def enu_to_ned(x_enu: float, y_enu: float, z_enu: float) -> Tuple[float, float, float]:
    """
    Convert ENU coordinates to NED coordinates.

    ENU: x=East, y=North, z=Up
    NED: x=North, y=East, z=Down

    Args:
        x_enu: East coordinate in ENU
        y_enu: North coordinate in ENU
        z_enu: Up coordinate in ENU

    Returns:
        Tuple of (x_ned, y_ned, z_ned)
    """
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu
    return (x_ned, y_ned, z_ned)


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float) -> Tuple[float, float, float]:
    """
    Convert NED coordinates to ENU coordinates.

    Args:
        x_ned: North coordinate in NED
        y_ned: East coordinate in NED
        z_ned: Down coordinate in NED

    Returns:
        Tuple of (x_enu, y_enu, z_enu)
    """
    x_enu = y_ned
    y_enu = x_ned
    z_enu = -z_ned
    return (x_enu, y_enu, z_enu)


def distance_3d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
    """Calculate 3D Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


def distance_2d(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate 2D Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw angle to quaternion (x, y, z, w)."""
    half_yaw = yaw / 2.0
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle from quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))