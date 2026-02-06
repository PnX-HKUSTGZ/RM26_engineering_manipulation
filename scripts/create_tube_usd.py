#!/usr/bin/env python3
"""Generate a hollow tube USD mesh for the peg tool."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path


def _ring_points(radius: float, z: float, segments: int) -> list[tuple[float, float, float]]:
    pts = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        pts.append((radius * math.cos(theta), radius * math.sin(theta), z))
    return pts


def build_tube_mesh(outer_radius: float, inner_radius: float, length: float, segments: int):
    half = length * 0.5
    outer_bottom = _ring_points(outer_radius, -half, segments)
    outer_top = _ring_points(outer_radius, half, segments)
    inner_bottom = _ring_points(inner_radius, -half, segments)
    inner_top = _ring_points(inner_radius, half, segments)

    points = outer_bottom + outer_top + inner_bottom + inner_top

    def idx_outer_bottom(i: int) -> int:
        return i

    def idx_outer_top(i: int) -> int:
        return segments + i

    def idx_inner_bottom(i: int) -> int:
        return 2 * segments + i

    def idx_inner_top(i: int) -> int:
        return 3 * segments + i

    face_counts: list[int] = []
    face_indices: list[int] = []

    for i in range(segments):
        j = (i + 1) % segments
        # outer surface (quad)
        face_counts.append(4)
        face_indices.extend([
            idx_outer_bottom(i),
            idx_outer_bottom(j),
            idx_outer_top(j),
            idx_outer_top(i),
        ])
        # inner surface (quad, reversed winding)
        face_counts.append(4)
        face_indices.extend([
            idx_inner_bottom(j),
            idx_inner_bottom(i),
            idx_inner_top(i),
            idx_inner_top(j),
        ])

    return points, face_counts, face_indices


def write_usda(path: Path, points, face_counts, face_indices):
    def fmt_pt(p):
        return f"({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})"

    points_str = ",\n        ".join(fmt_pt(p) for p in points)
    counts_str = ", ".join(str(c) for c in face_counts)
    indices_str = ", ".join(str(i) for i in face_indices)

    content = f"""#usda 1.0
(
    defaultPrim = \"PegTube\"
)

def Xform \"PegTube\"
{{
    def Mesh \"Tube\" (
        prepend apiSchemas = [\"PhysicsCollisionAPI\"]
    )
    {{
        uniform bool physics:collisionEnabled = 1
        uniform token subdivisionScheme = \"none\"
        point3f[] points = [
        {points_str}
        ]
        int[] faceVertexCounts = [{counts_str}]
        int[] faceVertexIndices = [{indices_str}]
    }}
}}
"""
    path.write_text(content)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=None)
    parser.add_argument("--outer_diameter", type=float, default=0.04)
    parser.add_argument("--inner_diameter", type=float, default=0.036)
    parser.add_argument("--length", type=float, default=0.15)
    parser.add_argument("--segments", type=int, default=32)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    default_path = repo_root / "isaacLab" / "manipulation" / "assets" / "usd" / "custom" / "peg_tube.usda"
    out_path = Path(args.output) if args.output else default_path
    out_path.parent.mkdir(parents=True, exist_ok=True)

    points, face_counts, face_indices = build_tube_mesh(
        outer_radius=args.outer_diameter * 0.5,
        inner_radius=args.inner_diameter * 0.5,
        length=args.length,
        segments=args.segments,
    )
    write_usda(out_path, points, face_counts, face_indices)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
