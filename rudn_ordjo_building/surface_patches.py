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


# REALISM NOTE (2026-08-04). The original four profiles were picked by eye on a
# laptop to make the robot visibly slip, and never checked against measured
# rubber-on-surface coefficients. Reference values: dry concrete 0.7-1.0, wet
# concrete 0.5-0.7, dry linoleum ~0.4-0.5, wet tile ~0.3, oily floor ~0.2, tyre
# on ice 0.1-0.15. The deployment target is rubber tyres on university linoleum
# and tile, so `linoleum` and `wet_tile` below are the COMMON case and `icy`
# (0.05, polished black ice) is a deliberately adversarial rarity -- it is not an
# indoor floor and must be described as a worst case in any write-up.
# See DEFAULT_PATCH_WEIGHTS for how often each is drawn.
PROFILES: dict = {
    # Dry linoleum / vinyl -- the actual deployment surface.
    "linoleum": PhysicsProfile(
        mu=0.45,
        mu2=0.45,
        slip1=0.0,
        slip2=0.0,
        color_rgba=(0.85, 0.80, 0.65, 0.65),   # pale tan
    ),
    # Wet or polished tile: still walkable, noticeably slick.
    "wet_tile": PhysicsProfile(
        mu=0.30,
        mu2=0.30,
        slip1=0.1,
        slip2=0.1,
        color_rgba=(0.55, 0.75, 0.80, 0.70),   # pale teal
    ),
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
    # Same mu as "icy", zero force-dependent slip. Exists to answer whether
    # slip1/slip2 do anything at all: they live in a <friction><ode> block, but
    # gz-sim loads the DARTSIM plugin (the world's <physics type="ode"> is
    # Gazebo Classic syntax that gz-sim ignores), and DART is not known to read
    # ODE's force-dependent-slip parameters. Drive across this and "icy" in one
    # run: identical behaviour means mu is the only knob that was ever
    # connected. Magenta so the two are distinguishable on screen.
    "icy_noslip": PhysicsProfile(
        mu=0.05,
        mu2=0.05,
        slip1=0.0,
        slip2=0.0,
        color_rgba=(1.00, 0.30, 0.90, 0.88),
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


# How often each profile is drawn when a sampler randomizes patches.
#
# The old sampler drew UNIFORMLY from
# ["slippery", "icy", "directional_x", "directional_y"], so HALF of every patch
# set was at or below tyre-on-ice friction -- an adversarial worst case being
# presented as a floor. Gains tuned against that are tuned for black ice and are
# over-aggressive on the surface the robot actually runs on.
#
# This distribution keeps the hard cases (they are what the corrector exists to
# handle, and the advisor wants an ice comparison) but makes them the minority:
# ~60% at mu >= 0.30, 10% ice. Changing these numbers changes the plant, so any
# comparison across the change must be re-baselined -- do not mix results.
DEFAULT_PATCH_WEIGHTS: dict = {
    "linoleum": 0.30,
    "wet_tile": 0.25,
    "slippery": 0.20,
    "directional_x": 0.08,
    "directional_y": 0.07,
    "icy": 0.10,
}


# ---------------------------------------------------------------------------
# SDF generation
# ---------------------------------------------------------------------------

_THICKNESS = 0.02   # box height in metres; top face sits flush with z=patch.z
# The link is SUNK by its own half-height so the top face lands on z=patch.z
# instead of a full thickness above it. It used to be raised by +THICKNESS/2,
# which put the top face ~2.1 cm above the floor (spawn z is 0.001) and made
# every "friction patch" a 2 cm curb: the robot drove into the leading edge and
# stopped dead, wheels turning. That reads exactly like zero traction in the
# logs, and it is not -- it is a step. Patches must be surfaces, not obstacles.
#
# patch.z defaults to 0.001, so the top face ends up 1 mm proud of the ground
# rather than exactly coincident with it. That is deliberate: two collision
# surfaces at identical height make contact selection ambiguous, and the robot
# can end up resting on the floor plane with the patch's friction never applied.
# 1 mm is below anything a 82.5 mm wheel notices.
_SINK = -_THICKNESS / 2
# The VISUAL is lifted higher than the collision. At 1 mm the patch surface and
# the floor are close enough that the renderer z-fights and the patch appears to
# clip through the ground. A visual carries no contact geometry, so it can sit
# as high as it likes without the robot ever touching it -- the physics still
# uses the 1 mm collision offset above. Keep these two independent: raising the
# COLLISION to fix a rendering artefact is how the 2 cm curb happened.
_VISUAL_LIFT = 0.006
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
                f"<pose>{ox:.4f} 0 {_SINK + _VISUAL_LIFT:.4f} 0 0 0</pose>"
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
                f"<pose>0 {oy:.4f} {_SINK + _VISUAL_LIFT:.4f} 0 0 0</pose>"
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
            f"<pose>0 0 {_SINK + _VISUAL_LIFT:.4f} 0 0 0</pose>"
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
        f"<pose>0 0 0 0 0 {yaw:.6f}</pose>"
        f"{visual_xml}"
        "<collision name=\"collision\">"
        f"<pose>0 0 {_SINK:.4f} 0 0 0</pose>"
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
