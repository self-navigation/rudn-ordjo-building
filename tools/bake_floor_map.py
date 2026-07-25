#!/usr/bin/env python3
"""Bake a floor of the RUDN building into a ROS occupancy-grid map (PNG + YAML).

WHY THIS EXISTS
---------------
The runtime map normally comes from rtabmap, but SLAM is a poor fixture for
testing the *corrector*: it needs the robot to drive before a map exists, it
fails to initialise on featureless walls, and it produces a slightly different
map every run -- so two corrector runs are never comparable. This tool bakes the
map ahead of time from the same meshes Gazebo collides against, giving a
deterministic fixture that is byte-identical across runs.

It also removes SLAM's *sensor* dependency: with a baked map, nothing consumes
the lidar or the cameras, so the sim can run with SIM_SENSORS=false and a much
better realtime factor.

This is an OFFLINE tool. It needs trimesh, which is deliberately NOT a runtime
dependency of the workspace -- run it in a venv:

    python3 -m venv /tmp/meshvenv
    /tmp/meshvenv/bin/pip install trimesh numpy pillow
    /tmp/meshvenv/bin/python tools/bake_floor_map.py

The output is checked in, so a normal user never runs this.

HOW THE GEOMETRY LINES UP
-------------------------
Three facts have to agree with the sim, and all three are read off the launch
files rather than guessed:

1. The GLBs are Y-up. model_template.sdf gives the link `pose` a +90 deg roll,
   so a mesh vertex (x, y, z) lands at world (x, -z, y).  See MESH_TO_WORLD.
2. gz_sim.launch.py spawns the floor at (23, 5, 0) -- chosen so that the robot's
   (0, 0) spawn falls inside the building.  See FLOOR_ORIGIN.
3. spawn_floor.launch.py drops the `center` part on floors >= 4 and the `right`
   part on floors >= 6.  See floor_parts().

The robot does NOT drive on the mesh: ordjo_world.world has a flat ground plane
at z=0 and the robot is spawned above it and dropped. So the meshes contribute
walls only, and a horizontal slice at lidar height is exactly what a perfect
sensor would see.

If any of those three change, this file is what goes stale -- there is no
runtime check that can catch it.
"""

from __future__ import annotations

import argparse
import os
from collections import deque

import numpy as np

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MESH_DIR = os.path.join(PKG, "models", "meshes")
DEFAULT_OUT = os.path.join(PKG, "maps")

# Where the consuming workspace spawns this floor. This package does not choose
# it -- agx_navigation's gz_sim.launch.py passes x=23, y=5 to spawn_floor.launch.py
# so that the robot's (0, 0) spawn lands inside the building. Override with
# --floor-origin if your workspace spawns it elsewhere, or the baked map will be
# offset from the sim's actual walls.
DEFAULT_FLOOR_ORIGIN = (23.0, 5.0)

# Standard ROS map_server greyscale conventions.
PX_FREE = 254
PX_UNKNOWN = 205
PX_OCCUPIED = 0


def floor_parts(floor: int) -> list[str]:
    """Mirror spawn_floor.launch.py's exclude_part() rules."""
    parts = ["left"]
    if floor < 4:
        parts.append("center")
    if floor < 6:
        parts.append("right")
    return parts


def mesh_to_world(vertices: np.ndarray, floor_origin) -> np.ndarray:
    """Apply the SDF link pose (roll +90 deg) and the floor spawn offset."""
    return np.column_stack(
        [vertices[:, 0], -vertices[:, 2], vertices[:, 1]]
    ) + np.array([floor_origin[0], floor_origin[1], 0.0])


def slice_segments(floor: int, heights: np.ndarray, floor_origin) -> list[np.ndarray]:
    """Cut each floor part at every height and return XY segments.

    Uses intersections.mesh_plane rather than Trimesh.section: section() assembles
    the crossings into connected Path2D entities, which drags in scipy's graph
    traversal, and we only ever rasterize the raw segments anyway.
    """
    import trimesh

    out = []
    for part in floor_parts(floor):
        path = os.path.join(MESH_DIR, f"floor_{floor}_{part}.glb")
        mesh = trimesh.load(path, force="mesh")
        mesh.vertices = mesh_to_world(mesh.vertices, floor_origin)
        for z in heights:
            lines = trimesh.intersections.mesh_plane(
                mesh, plane_origin=[0.0, 0.0, float(z)], plane_normal=[0.0, 0.0, 1.0]
            )
            if len(lines):
                out.append(lines[:, :, :2])
    return out


def rasterize(segments, origin, shape, res) -> np.ndarray:
    """Stamp segments into a boolean occupancy grid (row 0 = y_min)."""
    height, width = shape
    grid = np.zeros(shape, dtype=bool)
    ox, oy = origin
    for arr in segments:
        for p0, p1 in arr:
            # Sample at half-cell spacing: dense enough that a segment never
            # skips a cell, which would punch a hole a planner could route
            # through.
            n = max(2, int(np.hypot(*(p1 - p0)) / (res * 0.5)) + 1)
            t = np.linspace(0.0, 1.0, n)[:, None]
            pts = p0 + t * (p1 - p0)
            cols = ((pts[:, 0] - ox) / res).astype(int)
            rows = ((pts[:, 1] - oy) / res).astype(int)
            ok = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
            grid[rows[ok], cols[ok]] = True
    return grid


def flood_free(occupied: np.ndarray, seed_rc) -> np.ndarray:
    """4-connected flood fill of free space from the seed.

    Everything the robot cannot reach -- outside the building, sealed rooms --
    stays UNKNOWN rather than free. That matters: the vector field runs Fast
    Marching over free space, and an unbounded free region outside the walls
    would let the front escape around the building and produce a nonsense
    'optimal' path.
    """
    height, width = occupied.shape
    free = np.zeros_like(occupied)
    r0, c0 = seed_rc
    if not (0 <= r0 < height and 0 <= c0 < width):
        raise SystemExit(f"seed {seed_rc} outside the grid {occupied.shape}")
    if occupied[r0, c0]:
        raise SystemExit(
            f"seed cell {seed_rc} is a wall -- pick a --seed inside the building"
        )
    queue = deque([(r0, c0)])
    free[r0, c0] = True
    while queue:
        r, c = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < height and 0 <= cc < width:
                if not free[rr, cc] and not occupied[rr, cc]:
                    free[rr, cc] = True
                    queue.append((rr, cc))
    return free


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--floor", type=int, default=3)
    ap.add_argument("--resolution", type=float, default=0.05, help="metres per cell")
    ap.add_argument("--z-min", type=float, default=0.10,
                    help="bottom of the slice band, world metres")
    ap.add_argument("--z-max", type=float, default=0.70,
                    help="top of the slice band (matches pointcloud_to_laserscan's "
                         "max_height in slam.launch.py)")
    ap.add_argument("--z-step", type=float, default=0.05)
    ap.add_argument("--bounds", type=float, nargs=4,
                    metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                    default=[-30.0, -30.0, 30.0, 30.0],
                    help="world-frame crop. The full floor is ~180x80 m; cropping "
                         "keeps the Fast Marching grid small.")
    ap.add_argument("--seed", type=float, nargs=2, metavar=("X", "Y"),
                    default=[0.0, 0.0],
                    help="a point in free space (the robot spawn) to flood from")
    ap.add_argument("--floor-origin", type=float, nargs=2, metavar=("X", "Y"),
                    default=list(DEFAULT_FLOOR_ORIGIN),
                    help="where the simulation spawns this floor (see above)")
    ap.add_argument("--out-dir", default=DEFAULT_OUT)
    args = ap.parse_args()

    from PIL import Image

    xmin, ymin, xmax, ymax = args.bounds
    res = args.resolution
    width = int(round((xmax - xmin) / res))
    height = int(round((ymax - ymin) / res))

    heights = np.arange(args.z_min, args.z_max + 1e-9, args.z_step)
    print(f"slicing floor {args.floor} at {len(heights)} heights "
          f"({args.z_min}..{args.z_max} m), parts={floor_parts(args.floor)}")
    segments = slice_segments(args.floor, heights, args.floor_origin)
    print(f"  {sum(len(a) for a in segments)} segments")

    occupied = rasterize(segments, (xmin, ymin), (height, width), res)
    print(f"  {occupied.sum()} occupied cells in {width}x{height} grid")

    seed_rc = (int((args.seed[1] - ymin) / res), int((args.seed[0] - xmin) / res))
    free = flood_free(occupied, seed_rc)
    print(f"  {free.sum()} reachable free cells "
          f"({100.0 * free.sum() / free.size:.1f}% of the grid)")

    img = np.full((height, width), PX_UNKNOWN, dtype=np.uint8)
    img[free] = PX_FREE
    img[occupied] = PX_OCCUPIED

    os.makedirs(args.out_dir, exist_ok=True)
    stem = f"floor_{args.floor}"
    png_path = os.path.join(args.out_dir, stem + ".png")
    yaml_path = os.path.join(args.out_dir, stem + ".yaml")

    # ROS maps put row 0 at y_min; PNG row 0 is the top of the image.
    Image.fromarray(np.flipud(img)).save(png_path)

    with open(yaml_path, "w") as fh:
        fh.write(
            "# Generated by rudn_ordjo_building/tools/bake_floor_map.py.\n"
            "# Do not hand-edit this file -- but the PNG next to it IS meant to be\n"
            "# hand-edited (block a doorway, carve a shortcut). It is plain\n"
            "# greyscale:\n"
            f"#   {PX_FREE} = free, {PX_OCCUPIED} = wall, {PX_UNKNOWN} = unknown.\n"
            f"# Rebuild with: --floor {args.floor} --resolution {res}"
            f" --bounds {xmin} {ymin} {xmax} {ymax}"
            f" --floor-origin {args.floor_origin[0]} {args.floor_origin[1]}"
            f" --z-min {args.z_min} --z-max {args.z_max}\n"
            f"image: {stem}.png\n"
            f"resolution: {res}\n"
            f"origin: [{xmin}, {ymin}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
            "mode: trinary\n"
        )

    print(f"wrote {png_path}\n      {yaml_path}")


if __name__ == "__main__":
    main()
