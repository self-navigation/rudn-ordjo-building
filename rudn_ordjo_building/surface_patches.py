"""Physics surface-patch profiles and SDF generation.

Extracted from spawn_surface_patches.launch.py so the SDF builder is importable
without pulling in `launch` -- the launch file re-imports these names, and the RL
corrector's Gazebo bridge spawns the same patches at runtime for domain
randomization. This module is the single source of truth for the friction
profiles; do not duplicate the mu/slip numbers elsewhere.
"""

from dataclasses import dataclass
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# Physics profiles
# ---------------------------------------------------------------------------

@dataclass
class PhysicsProfile:
    mu: float
    mu2: float
    slip1: float
    slip2: float
    color_rgba: Tuple[float, float, float, float]
    fdir1: Optional[Tuple[float, float, float]] = None
    # When set, the visual is replaced with stripes running along this axis
    # ("x" = slip is in x, stripes elongated in x; "y" = slip is in y)
    slip_axis: Optional[str] = None


PROFILES: dict = {
    "slippery": PhysicsProfile(
        mu=0.2,
        mu2=0.2,
        slip1=0.3,
        slip2=0.3,
        color_rgba=(0.25, 0.60, 1.00, 0.75),   # sky blue
    ),
    "icy": PhysicsProfile(
        mu=0.05,
        mu2=0.05,
        slip1=0.85,
        slip2=0.85,
        color_rgba=(0.88, 0.95, 1.00, 0.88),   # pale ice white-blue
    ),
    "rough": PhysicsProfile(
        mu=2.5,
        mu2=2.5,
        slip1=0.0,
        slip2=0.0,
        color_rgba=(0.42, 0.26, 0.12, 1.00),   # dark brown
    ),
    # Grips in local X, slides in local Y.  Stripes run in Y (slip direction).
    "directional_x": PhysicsProfile(
        mu=1.0,
        mu2=0.15,
        slip1=0.0,
        slip2=0.55,
        color_rgba=(0.10, 0.35, 0.90, 0.90),
        fdir1=(1.0, 0.0, 0.0),
        slip_axis="y",
    ),
    # Grips in local Y, slides in local X.  Stripes run in X (slip direction).
    "directional_y": PhysicsProfile(
        mu=0.15,
        mu2=1.0,
        slip1=0.55,
        slip2=0.0,
        color_rgba=(0.10, 0.35, 0.90, 0.90),
        fdir1=(1.0, 0.0, 0.0),
        slip_axis="x",
    ),
}


# ---------------------------------------------------------------------------
# SDF generation
# ---------------------------------------------------------------------------

_THICKNESS = 0.02   # box height in metres; top face sits flush with z=patch.z
_STRIPE_COUNT = 24   # number of stripes on directional surfaces


def _fmt_color(rgba: Tuple[float, float, float, float]) -> str:
    r, g, b, a = rgba
    return f"{r} {g} {b} {a}"


def _stripe_visuals(width: float, length: float, slip_axis: str, c1: tuple[float, float, float, float], c2: tuple[float, float, float, float]) -> str:
    """Return XML for alternating stripe <visual> elements."""
    parts = []

    if slip_axis == "y":
        sw = width / _STRIPE_COUNT
        for i in range(_STRIPE_COUNT):
            ox = -width / 2 + (i + 0.5) * sw
            color = _fmt_color(c1 if i % 2 == 0 else c2)
            parts.append(
                f"<visual name=\"stripe_{i}\">"
                f"<pose>{ox:.4f} 0 0 0 0 0</pose>"
                f"<geometry><box>"
                f"<size>{sw:.4f} {length:.4f} {_THICKNESS:.4f}</size>"
                f"</box></geometry>"
                f"<material>"
                f"<ambient>{color}</ambient>"
                f"<diffuse>{color}</diffuse>"
                f"</material>"
                f"</visual>"
            )
    else:
        sl = length / _STRIPE_COUNT
        for i in range(_STRIPE_COUNT):
            oy = -length / 2 + (i + 0.5) * sl
            color = _fmt_color(c1 if i % 2 == 0 else c2)
            parts.append(
                f"<visual name=\"stripe_{i}\">"
                f"<pose>0 {oy:.4f} 0 0 0 0</pose>"
                f"<geometry><box>"
                f"<size>{width:.4f} {sl:.4f} {_THICKNESS:.4f}</size>"
                f"</box></geometry>"
                f"<material>"
                f"<ambient>{color}</ambient>"
                f"<diffuse>{color}</diffuse>"
                f"</material>"
                f"</visual>"
            )

    return "\n        ".join(parts)


def build_patch_sdf(patch: dict, profile: PhysicsProfile, idx: int) -> str:
    name = patch.get("name") or f"surface_patch_{idx}"
    width = patch["width"]
    length = patch["length"]
    yaw = patch.get("yaw", 0.0)

    if profile.slip_axis:
        blue = (0.10, 0.35, 0.90, 0.90)
        brown = (0.431, 0.271, 0, 0.75)
        visual_xml = _stripe_visuals(width, length, profile.slip_axis, blue, brown)
    else:
        color = _fmt_color(profile.color_rgba)
        visual_xml = (
            f"<visual name=\"visual\">"
            f"<geometry><box>"
            f"<size>{width:.4f} {length:.4f} {_THICKNESS:.4f}</size>"
            f"</box></geometry>"
            f"<material>"
            f"<ambient>{color}</ambient>"
            f"<diffuse>{color}</diffuse>"
            f"</material>"
            f"</visual>"
        )

    fdir_xml = ""
    if profile.fdir1:
        fx, fy, fz = profile.fdir1
        fdir_xml = f"<fdir1>{fx} {fy} {fz}</fdir1>"

    return (
        '<?xml version="1.0" ?>'
        '<sdf version="1.9">'
        f'<model name="{name}">'
        "<static>true</static>"
        "<link name=\"link\">"
        f"<pose>0 0 {_THICKNESS / 2:.4f} 0 0 {yaw:.6f}</pose>"
        f"{visual_xml}"
        "<collision name=\"collision\">"
        "<geometry><box>"
        f"<size>{width:.4f} {length:.4f} {_THICKNESS:.4f}</size>"
        "</box></geometry>"
        "<surface><friction><ode>"
        f"<mu>{profile.mu}</mu>"
        f"<mu2>{profile.mu2}</mu2>"
        f"{fdir_xml}"
        f"<slip1>{profile.slip1}</slip1>"
        f"<slip2>{profile.slip2}</slip2>"
        "</ode></friction></surface>"
        "</collision>"
        "</link>"
        "</model>"
        "</sdf>"
    )
