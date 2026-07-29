"""Flatten rectangular areas of a scanned mesh onto their own plane.

An area is described by a pose (centre plus roll/pitch/yaw) and a size
(lx, ly). The plane of that rectangle is the reference: every vertex whose
projection falls inside the footprint is moved onto it, so the area ends up
perfectly flat and coplanar with the rectangle. The rest of the mesh is left
untouched.

The footprint boundary is obtained by slicing the surface along the four side
planes of the rectangle. Slicing inserts vertices exactly on those planes and
produces two independent copies of the boundary loop, one for the inner region
and one for the outer. Only the inner copy is displaced; the two loops are then
stitched with a strip of triangles, which forms a wall parallel to the
rectangle normal. Each area therefore meets its surroundings with a clean step
instead of a slanted transition, whatever the triangle density of the scan.

Which vertices move is controlled by `remove_side`:

    0   both directions -- bumps are pressed down and hollows are raised up
   -1   clipping only -- material on the -local_z side is brought onto the plane
   +1   clipping only -- material on the +local_z side is brought onto the plane

Poses with a roll near 180 degrees have their local +z axis pointing into the
part, so -1 is the mode that removes material standing out toward the tool.

Supported formats: anything trimesh reads and writes (STL, OBJ, PLY, GLB, ...),
inferred from the file extension.

Dependencies: trimesh, numpy, plus the local module flatten_mesh.
"""

from __future__ import annotations

import numpy as np
import trimesh
from trimesh.intersections import slice_faces_plane


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



def _rect_planes(position, rot, half_x, half_y):
    """Yield the four side planes of the rectangle as (origin, normal) pairs.

    Each normal points toward the interior of the footprint, so keeping the
    half-space a normal points to and intersecting the four results gives the
    footprint itself.
    """
    x_axis, y_axis = rot[:, 0], rot[:, 1]
    return (
        (position + half_x * x_axis, -x_axis),
        (position - half_x * x_axis, x_axis),
        (position + half_y * y_axis, -y_axis),
        (position - half_y * y_axis, y_axis),
    )


def _slice(vertices, faces, origin, normal):
    """Keep the part of a triangle soup lying on the +normal side of a plane.

    Triangles crossing the plane are split, so the returned geometry has
    vertices sitting exactly on it.
    """
    if len(faces) == 0:
        return vertices, faces
    return slice_faces_plane(
        np.asarray(vertices, dtype=np.float64),
        np.asarray(faces, dtype=np.int64),
        np.asarray(normal, dtype=np.float64),
        np.asarray(origin, dtype=np.float64),
    )[:2]


def _split_on_rectangle(mesh, position, rot, half_x, half_y):
    """Partition a mesh into the region inside the footprint and the rest.

    Returns (inside, outside_pieces). Each side plane is applied twice: once
    keeping the interior half-space, once keeping the exterior. What the first
    call keeps is fed to the next plane, what the second call keeps is set
    aside. The pieces reassemble into the original surface with no overlap and
    no gap, and every cut vertex lies exactly on a side plane.

    `inside` is None when the footprint covers no geometry at all.
    """
    in_v, in_f = mesh.vertices, mesh.faces
    outside = []
    for origin, normal in _rect_planes(position, rot, half_x, half_y):
        if len(in_f) == 0:
            return None, outside
        ov, of = _slice(in_v, in_f, origin, -np.asarray(normal, float))
        if len(of):
            outside.append(trimesh.Trimesh(vertices=ov, faces=of, process=False))
        in_v, in_f = _slice(in_v, in_f, origin, normal)
    if len(in_f) == 0:
        return None, outside
    return trimesh.Trimesh(vertices=in_v, faces=in_f, process=False), outside


def _border_edges(mesh, position, rot, half_x, half_y, tol=1e-7):
    """Return the edges forming the cut boundary of an inside region.

    An edge qualifies when it belongs to a single face -- meaning it borders a
    hole rather than sitting between two triangles -- and both of its endpoints
    lie on one of the rectangle side planes. Those are the edges the wall is
    built from.
    """
    local = (mesh.vertices - position) @ rot
    on_border = (np.abs(np.abs(local[:, 0]) - half_x) <= tol) | (
        np.abs(np.abs(local[:, 1]) - half_y) <= tol
    )
    edges = mesh.edges_sorted
    single = trimesh.grouping.group_rows(edges, require_count=1)
    if len(single) == 0:
        return np.zeros((0, 2), dtype=np.int64)
    boundary = edges[single]
    return boundary[on_border[boundary].all(axis=1)]


def _drop_fragments(mesh, min_area):
    """Remove connected components smaller than `min_area`, keeping the largest.

    Cutting a triangle that only grazes a side plane leaves a sliver behind.
    When the cuts of the following planes subdivide the neighbouring edges,
    such a sliver no longer shares a complete edge with anything and ends up as
    a free-floating fragment, which a viewer draws as a speck or a hairline.
    Successive rectangles produce them along their borders, so they read as a
    dotted line. Only components far smaller than a real surface are dropped.
    """
    if min_area <= 0 or len(mesh.faces) == 0:
        return mesh
    comps = trimesh.graph.connected_components(
        mesh.face_adjacency, nodes=np.arange(len(mesh.faces))
    )
    if len(comps) <= 1:
        return mesh
    areas = mesh.area_faces
    totals = [areas[c].sum() for c in comps]
    biggest = int(np.argmax(totals))
    keep = np.zeros(len(mesh.faces), dtype=bool)
    for i, c in enumerate(comps):
        if i == biggest or totals[i] >= min_area:
            keep[c] = True
    if keep.all():
        return mesh
    mesh.update_faces(keep)
    mesh.remove_unreferenced_vertices()
    return mesh


def flatten_rectangle_sharp(
    mesh: trimesh.Trimesh,
    position,
    rot,
    size,
    remove_side: int = 0,
    weld: bool = True,
    min_fragment_area: float = 1e-4,
) -> trimesh.Trimesh:
    """Flatten one rectangular area of `mesh` and return the result.

    Parameters
    ----------
    mesh : surface to work on; it is not modified in place.
    position : (x, y, z) centre of the rectangle, in the mesh frame.
    rot : 3x3 rotation of the rectangle, e.g. rpy_to_matrix(roll, pitch, yaw).
          Its third column is the plane normal.
    size : (lx, ly) dimensions along the rectangle's local x and y axes.
    remove_side : 0 to move every vertex inside the footprint, -1 or +1 to move
          only those on the corresponding side of the plane.
    weld : merge coincident vertices at the end, which sews the wall onto the
          two regions it connects.
    min_fragment_area : area in m2 under which a disconnected component is
          discarded. The largest component is always kept. Set to 0 to keep
          every fragment.
    """
    position = np.asarray(position, dtype=float)
    rot = np.asarray(rot, dtype=float)
    half_x, half_y = float(size[0]) / 2.0, float(size[1]) / 2.0

    inside, outside = _split_on_rectangle(mesh, position, rot, half_x, half_y)
    if inside is None:
        return mesh

    # The wall is spanned between the boundary before and after the move, so
    # both the edge list and the original positions are needed.
    border = _border_edges(inside, position, rot, half_x, half_y)
    before = inside.vertices.copy()

    # Working in the rectangle frame turns "distance to the plane" into the
    # third coordinate, so flattening is just zeroing it.
    local = (inside.vertices - position) @ rot
    if remove_side == 0:
        moved = np.ones(len(local), dtype=bool)
    else:
        s = -1.0 if remove_side < 0 else 1.0
        moved = (s * local[:, 2]) > 0.0
    local[moved, 2] = 0.0
    after = local @ rot.T + position

    inside = trimesh.Trimesh(vertices=after, faces=inside.faces, process=False)

    # Each border edge becomes the quad (old_a, old_b, new_b, new_a), split
    # into two triangles. Edges whose endpoints did not move span nothing and
    # are skipped.
    wall_v, wall_f = [], []
    for a, b in border:
        A0, B0, A1, B1 = before[a], before[b], after[a], after[b]
        if np.allclose(A0, A1, atol=1e-12) and np.allclose(B0, B1, atol=1e-12):
            continue
        i = len(wall_v)
        wall_v.extend([A0, B0, B1, A1])
        wall_f.extend([[i, i + 1, i + 2], [i, i + 2, i + 3]])

    parts = [inside]
    if wall_f:
        parts.append(
            trimesh.Trimesh(
                vertices=np.asarray(wall_v, dtype=float),
                faces=np.asarray(wall_f, dtype=np.int64),
                process=False,
            )
        )
    parts.extend(outside)

    # Zeroing the third coordinate collapses any triangle that was standing
    # perpendicular to the plane, hence the degenerate-face cleanup.
    result = trimesh.util.concatenate(parts)
    result.update_faces(result.nondegenerate_faces())
    result.remove_unreferenced_vertices()
    if weld:
        result.merge_vertices()
    return _drop_fragments(result, min_fragment_area)


def _run_on_file(input_path, output_path, position, rpy, size, remove_side,
                 mesh_rpy, min_fragment_area=1e-4):
    """Load a mesh, flatten one area, write the result back to disk.

    `mesh_rpy` rotates the whole mesh about the origin of its frame before the
    area is processed, which straightens a badly oriented scan; the rotation is
    kept in the exported file, so the pose passed in is understood in the
    straightened frame.
    """
    mesh = trimesh.load(input_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Could not load a mesh from {input_path!r}")
    if mesh_rpy is not None and np.any(np.asarray(mesh_rpy, dtype=float) != 0.0):
        mesh_rot = rpy_to_matrix(*mesh_rpy)
        mesh = trimesh.Trimesh(
            vertices=mesh.vertices @ mesh_rot.T, faces=mesh.faces, process=False
        )
    result = flatten_rectangle_sharp(
        mesh, position, rpy_to_matrix(*rpy), size, remove_side=remove_side,
        min_fragment_area=min_fragment_area,
    )
    result.export(output_path)
    return result


def flatten_rectangle_sharp_in_frame(
    input_path, output_path, position, rpy, size, frame_to_mesh,
    mesh_rpy=None, remove_side=0, min_fragment_area=1e-4,
):
    """Flatten one area whose pose is expressed in an auxiliary frame F.

    `frame_to_mesh` is the 4x4 homogeneous transform such that
    p_mesh = frame_to_mesh @ p_F. The pose is assembled as a 4x4 in F, composed
    with it, and the resulting position and orientation are used in the mesh
    frame.
    """
    frame_to_mesh = np.asarray(frame_to_mesh, dtype=float)
    rect = np.eye(4)
    rect[:3, :3] = rpy_to_matrix(*rpy)
    rect[:3, 3] = np.asarray(position, dtype=float)
    rect = frame_to_mesh @ rect
    return _run_on_file(
        input_path, output_path, rect[:3, 3], matrix_to_rpy(rect[:3, :3]),
        size, remove_side, mesh_rpy, min_fragment_area,
    )


def shave_rectangle_sharp_in_frame(
    input_path, output_path, position, rpy, size, frame_to_mesh,
    mesh_rpy=None, remove_side=-1, min_fragment_area=1e-4,
):
    """Clipping-only counterpart of flatten_rectangle_sharp_in_frame.

    Same arguments, but `remove_side` defaults to -1, so material standing out
    toward the tool is brought down onto the plane while hollows are left as
    they are.
    """
    return flatten_rectangle_sharp_in_frame(
        input_path, output_path, position, rpy, size, frame_to_mesh,
        mesh_rpy=mesh_rpy, remove_side=remove_side,
        min_fragment_area=min_fragment_area,
    )