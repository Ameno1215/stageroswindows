"""Flatten a rectangular area of a scanned mesh (RealityScan, etc.).

Principle: a rectangle is defined in space by a position (its center), an
RPY orientation (roll/pitch/yaw in radians) and its dimensions (lx, ly).
The plane of the rectangle is used as the reference:

  - if the mesh is HIGHER than the plane (above it, along the rectangle
    normal) -> the mesh is pressed down onto the plane (whatever sticks
    out is removed);
  - if the mesh is LOWER than the plane -> it is raised up to the plane
    (hollows are filled).

Result: inside the rectangle footprint the surface is perfectly flat and
coplanar with the rectangle — ideal for placing/positioning a robot. The
rest of the mesh is left untouched.

Two entry points:
  - flatten_rectangle: rectangle pose given directly in the mesh frame;
  - flatten_rectangle_in_frame: rectangle pose given in another frame F,
    together with the 4x4 homogeneous transform from F to the mesh frame.

Both accept an optional mesh_rpy (--mesh-rpy on the CLI): an RPY rotation
applied to the whole mesh BEFORE flattening, to straighten a badly
oriented scan; the rotation is kept in the output file.

Supported input/output formats: anything trimesh handles
(STL, OBJ, PLY, GLB, OFF, ...), inferred from the file extension.

Dependencies: pip install trimesh numpy

Command-line example:
    python flatten_mesh.py scan.obj scan_flat.stl \
        --position 0.5 0.2 0.03 --rpy 0 0 0.785 --size 0.06 0.09

Python example:
    from flatten_mesh import flatten_rectangle
    flatten_rectangle("scan.obj", "scan_flat.stl",
                      position=(0.5, 0.2, 0.03),
                      rpy=(0.0, 0.0, 0.785),
                      size=(0.06 0.09))
"""

from __future__ import annotations

import argparse

import numpy as np
import trimesh


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """3x3 rotation matrix from roll/pitch/yaw (radians).

    Standard robotics convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    (extrinsic rotations about the fixed x, then y, then z axes).
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    """Roll/pitch/yaw (radians) from a 3x3 rotation matrix.

    Inverse of rpy_to_matrix (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)).
    """
    rot = np.asarray(rot, dtype=float)
    pitch = np.arcsin(np.clip(-rot[2, 0], -1.0, 1.0))
    if np.isclose(np.abs(rot[2, 0]), 1.0):  # gimbal lock: roll/yaw degenerate
        roll = 0.0
        yaw = np.arctan2(-rot[0, 1], rot[1, 1])
    else:
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
    return float(roll), float(pitch), float(yaw)


def _inside_mask(local_xy: np.ndarray, half_x: float, half_y: float) -> np.ndarray:
    """True for points (expressed in the rectangle frame) that lie within
    the rectangle footprint."""
    return (np.abs(local_xy[:, 0]) <= half_x) & (np.abs(local_xy[:, 1]) <= half_y)


def flatten_rectangle(
    input_path: str,
    output_path: str,
    position,
    rpy,
    size,
    refine_iterations: int = 3,
    mesh_rpy=None,
) -> trimesh.Trimesh:
    """Flatten the mesh onto the plane of a rectangle placed in space.

    Parameters
    ----------
    input_path : path of the input mesh (stl, obj, ply, glb, ...).
    output_path : path of the output mesh (format follows the extension).
    position : (x, y, z) — center of the rectangle in the mesh frame.
    rpy : (roll, pitch, yaw) in radians — orientation of the rectangle.
          The plane normal is the rectangle's local z axis after rotation.
    size : (lx, ly) — rectangle dimensions along its local x and y axes.
    refine_iterations : number of subdivision passes applied to triangles
          straddling the rectangle border, for a clean edge (0 = none).
    mesh_rpy : optional, (roll, pitch, yaw) in radians applied to the MESH
          itself (rotation about the origin of the mesh frame) BEFORE
          flattening. Useful to straighten a badly oriented scan; the
          rotation is kept in the output file. The rectangle position/rpy
          are then expressed in the straightened mesh frame.

    Returns the resulting trimesh.Trimesh (also written to output_path).
    """
    position = np.asarray(position, dtype=float)
    rot = rpy_to_matrix(*rpy)
    half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0

    mesh = trimesh.load(input_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Could not load a mesh from {input_path!r}")

    # 0) Optional rotation of the mesh itself before cleaning
    #    (straightening a badly oriented scan).
    if mesh_rpy is not None and np.any(np.asarray(mesh_rpy, dtype=float) != 0.0):
        mesh_rot = rpy_to_matrix(*mesh_rpy)
        mesh = trimesh.Trimesh(
            vertices=mesh.vertices @ mesh_rot.T, faces=mesh.faces, process=False
        )

    def to_local(vertices: np.ndarray) -> np.ndarray:
        return (vertices - position) @ rot  # same as rot.T @ (v - p)

    # 1) Refine triangles straddling the rectangle border so the
    #    flat / non-flat transition stays sharp even on a coarse mesh.
    for _ in range(max(0, int(refine_iterations))):
        local = to_local(mesh.vertices)
        inside = _inside_mask(local[:, :2], half_x, half_y)
        per_face = inside[mesh.faces]
        crossing = np.where(per_face.any(axis=1) & ~per_face.all(axis=1))[0]
        if len(crossing) == 0:
            break
        vertices, faces = trimesh.remesh.subdivide(
            mesh.vertices, mesh.faces, face_index=crossing
        )[:2]
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)

    # 2) Flatten: every vertex whose projection falls inside the rectangle
    #    is brought onto the plane (local z = 0). Bumps are pressed down,
    #    hollows are raised up.
    local = to_local(mesh.vertices)
    inside = _inside_mask(local[:, :2], half_x, half_y)
    local[inside, 2] = 0.0

    vertices = local @ rot.T + position
    mesh = trimesh.Trimesh(vertices=vertices, faces=mesh.faces, process=False)

    # 3) Cleanup: flattening may squash triangles (vertical walls,
    #    duplicates) -> drop degenerate faces.
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()

    mesh.export(output_path)
    return mesh


def flatten_rectangle_in_frame(
    input_path: str,
    output_path: str,
    position,
    rpy,
    size,
    frame_to_mesh,
    refine_iterations: int = 3,
    mesh_rpy=None,
) -> trimesh.Trimesh:
    """Same as flatten_rectangle, but the rectangle pose is expressed in
    another frame F, and frame_to_mesh is the 4x4 homogeneous transform
    from F to the mesh frame (T_mesh_F).

    Parameters
    ----------
    position, rpy : pose of the rectangle expressed in frame F.
    frame_to_mesh : 4x4 homogeneous matrix such that
          p_mesh = frame_to_mesh @ p_F. The rectangle pose is composed
          with it before flattening; the other parameters behave exactly
          like in flatten_rectangle.
    """
    frame_to_mesh = np.asarray(frame_to_mesh, dtype=float)
    if frame_to_mesh.shape != (4, 4):
        raise ValueError(
            f"frame_to_mesh must be a 4x4 matrix, got shape {frame_to_mesh.shape}"
        )

    # Rectangle pose in frame F as a 4x4, then composed into the mesh frame.
    rect_in_frame = np.eye(4)
    rect_in_frame[:3, :3] = rpy_to_matrix(*rpy)
    rect_in_frame[:3, 3] = np.asarray(position, dtype=float)
    rect_in_mesh = frame_to_mesh @ rect_in_frame

    return flatten_rectangle(
        input_path,
        output_path,
        position=rect_in_mesh[:3, 3],
        rpy=matrix_to_rpy(rect_in_mesh[:3, :3]),
        size=size,
        refine_iterations=refine_iterations,
        mesh_rpy=mesh_rpy,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Flatten a rectangular area of a mesh (fill hollows, "
        "press down bumps) to obtain a flat surface."
    )
    parser.add_argument("input", help="input mesh (stl, obj, ply, glb, ...)")
    parser.add_argument("output", help="output mesh (format follows the extension)")
    parser.add_argument(
        "--position", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"),
        help="center of the rectangle in the mesh frame",
    )
    parser.add_argument(
        "--rpy", nargs=3, type=float, default=(0.0, 0.0, 0.0),
        metavar=("ROLL", "PITCH", "YAW"),
        help="orientation of the rectangle in radians (default: 0 0 0)",
    )
    parser.add_argument(
        "--size", nargs=2, type=float, required=True, metavar=("LX", "LY"),
        help="rectangle dimensions along its local axes",
    )
    parser.add_argument(
        "--refine", type=int, default=3,
        help="subdivision passes at the rectangle border (default: 3)",
    )
    parser.add_argument(
        "--mesh-rpy", nargs=3, type=float, default=None,
        metavar=("ROLL", "PITCH", "YAW"),
        help="RPY rotation (radians) applied to the mesh BEFORE flattening, "
        "to straighten a badly oriented scan (default: none)",
    )
    args = parser.parse_args()

    result = flatten_rectangle(
        args.input, args.output,
        position=args.position, rpy=args.rpy, size=args.size,
        refine_iterations=args.refine, mesh_rpy=args.mesh_rpy,
    )
    print(
        f"OK: {len(result.vertices)} vertices, {len(result.faces)} faces "
        f"-> {args.output}"
    )


if __name__ == "__main__":
    main()
