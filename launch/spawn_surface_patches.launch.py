"""
Custom physics surface patches for the Gazebo simulation environment.

Patches are passed as a JSON array via the `patches` launch argument.
Each object in the array maps to a SurfacePatch:

    Required fields:
        x, y, z         world-frame position (float)
        width           local-X extent in metres (float)
        length          local-Y extent in metres (float)
        profile         one of the available profile names (string)

    Optional fields:
        yaw             rotation around Z in radians (float, default 0)
        name            Gazebo model name (string, auto-generated if omitted)

Available profiles:
    "slippery"      mu=0.2  — wet floor, moderate slide  [sky blue]
    "icy"           mu=0.05 — near-zero grip              [pale ice blue]
    "rough"         mu=2.5  — high grip                   [dark brown]
    "directional_x" grips in local X, slides in local Y  [blue/grey Y-stripes]
    "directional_y" slides in local X, grips in local Y  [blue/grey X-stripes]

Example argument value:
    patches:='[
        {"x": 25, "y": 8,  "z": 0, "width": 3, "length": 2, "profile": "slippery"},
        {"x": 30, "y": 5,  "z": 0, "width": 2, "length": 4, "profile": "icy"},
        {"x": 20, "y": 10, "z": 0, "width": 4, "length": 4, "profile": "directional_x"}
    ]'

Coordinate system: world frame matches the Gazebo world.  The building floor is
spawned at x=23 y=5 (see gz_sim.launch.py), so patch coordinates should be
given relative to the world origin.
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
