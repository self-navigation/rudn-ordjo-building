"""Spawn physics surface patches into the Gazebo world.

The `patches` launch argument is a JSON array of objects with fields x, y, z
(world frame), width (local X, m), length (local Y, m), profile, and optional
yaw (rad, default 0) and name. Profiles are defined in surface_patches.py.

Example:
    patches:='[{"x": 25, "y": 8, "z": 0, "width": 3, "length": 2, "profile": "slippery"}]'

Coordinates are world-frame; the building floor itself is spawned at (23, 5).
"""

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Profiles and the SDF builder now live in an importable module so the RL
# corrector's Gazebo bridge can spawn the same patches at runtime (DRY).
from rudn_ordjo_building.surface_patches import (
    PROFILES,
    build_patch_sdf as _build_patch_sdf,
)


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def _spawn_patches(context):
    raw = LaunchConfiguration("patches").perform(context).strip()
    if not raw or raw == "[]":
        return []

    try:
        patch_list = json.loads(raw)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in 'patches' argument: {e}") from e

    nodes = []
    for idx, patch in enumerate(patch_list):
        profile_name = patch.get("profile")
        profile = PROFILES.get(profile_name)
        if profile is None:
            raise ValueError(
                f"Patch {idx}: unknown profile '{profile_name}'. "
                f"Available: {list(PROFILES.keys())}"
            )

        sdf = _build_patch_sdf(patch, profile, idx)
        model_name = patch.get("name") or f"surface_patch_{profile_name}_{idx}"
        if 'z' not in patch:
            patch['z'] = 0.001

        nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", model_name,
                    "-string", sdf,
                    "-x", str(patch["x"]),
                    "-y", str(patch["y"]),
                    "-z", str(patch["z"]),
                ],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "patches",
            default_value="[]",
            description=(
                "JSON array of surface patch objects.  "
                "See spawn_surface_patches.launch.py docstring for schema."
            ),
        ),
        OpaqueFunction(function=_spawn_patches),
    ])
