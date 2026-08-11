r"""Prepare a scanned plate so every reader position sits on a flat surface.

A plate description (json) lists the readers of a test plate and, for each of
them, the positions the robot must reach. This script reads the corresponding
scan and levels one rectangular area per position, so the arm always presents
the card against a plane instead of against the raw scanned relief.

Positions are expressed in the plate frame F and converted to the mesh frame
with the transform `T_mesh_F` defined below.

Areas are processed in two phases:

  Phase 1 levels each area onto its own plane, filling hollows and pressing
  down bumps. Where two footprints overlap, the area processed last sets the
  height of the shared region.

  Phase 2 goes over the same areas in clipping mode: material standing above a
  plane is cut back down to it, and hollows are left alone. The lowest plane
  covering a given point therefore decides its final height, so a position
  higher than its neighbours no longer masks them.

Each call reads a mesh and writes a mesh, so the areas are accumulated by
chaining every call through a single temporary file. The result is moved to
--output once the last position has been handled. The state at the end of
phase 1 can be kept for inspection with --phase1_output.

Arguments
---------
--input        input mesh (stl, obj, ply, glb, ...), e.g. a RealityScan export
--json         plate description file (readers and their positions)
--output       output mesh; the format follows the extension
--clean_space  clearance in meters between a position and the surface levelled
               under it, measured along the rectangle normal (default: 0)
--size         rectangle dimensions LX LY in meters (default: 0.06 0.09)
--phase1_output  where to save the mesh obtained at the end of phase 1;
               omit it and that intermediate mesh is not written

Example (from the ROS folder):
    python .\prepare_mesh\prepare_plate_mesh.py ^
        --input .\plates3\plate3\plaque3new.obj ^
        --json .\plates3\plate3\plate_3_params.json ^
        --output out.obj --clean_space 0.005

Dependencies: trimesh, numpy, plus the local modules sharp_flatten, flatten_mesh
and plate
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh
import math

# The script is meant to be launched from the ROS folder, so its own directory
# and the parent one are put on the import path to reach the local modules.
_HERE = Path(__file__).resolve().parent
for _p in (str(_HERE), str(_HERE.parent)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from plate import load_plate_from_file
from sharp_flatten import (
    flatten_rectangle_sharp_in_frame,
    shave_rectangle_sharp_in_frame,
    rpy_to_matrix,
)

# Pose of the plate frame F in the mesh frame: p_mesh = T_mesh_F @ p_F.
T_mesh_F = np.eye(4)
T_mesh_F[:3, :3] = rpy_to_matrix(0.0, 0.0, 0)
# T_mesh_F[:3, :3] = rpy_to_matrix(0.0, 0.0, 0 + math.pi/2)
T_mesh_F[:3, 3] = [0.6245, -0.25, 0.0]
print(T_mesh_F)
inverse_matrix = np.linalg.inv(T_mesh_F)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Flatten one rectangular area per reader position of a plate "
        "(fill hollows, press down bumps) to obtain flat surfaces."
    )
    parser.add_argument("--input", help="input mesh (stl, obj, ply, glb, ...)")
    parser.add_argument("--json", help="path to json file")
    parser.add_argument("--output", help="output mesh (format follows the extension)")
    parser.add_argument("--clean_space", type=float, default=0.0, help="space margin between the mesh and the position on z axis (in m)")
    parser.add_argument(
        "--size", nargs=2, type=float, default=[0.06, 0.09], metavar=("LX", "LY"),
        help="rectangle dimensions along its local axes",
    )
    parser.add_argument(
        "--phase1_output", default=None,
        help="path where the mesh obtained at the end of phase 1 is saved; "
             "omitted, that intermediate mesh is not written",
    )
    args = parser.parse_args()

    plate = load_plate_from_file(args.json)

    # The temporary file carries the output extension so trimesh keeps
    # exporting in the same format from one call to the next.
    out_ext = Path(args.output).suffix or ".stl"
    tmp_path = str(Path(tempfile.gettempdir()) / f"flatten_chain{out_ext}")

    current_input = args.input   # the very first call reads the original scan
    count = 0

    for phase, flatten_fn in ((1, flatten_rectangle_sharp_in_frame),
                              (2, shave_rectangle_sharp_in_frame)):
        for reader in plate.readers:
            for pos in reader.positions:
                # Clearance is applied along the rectangle normal rather than
                # along z of frame F, so it stays correct on tilted positions.
                rot = rpy_to_matrix(pos.rx, pos.ry, pos.rz)
                local_z = rot[:, 2]
                center = np.array([pos.x, pos.y, pos.z]) + args.clean_space * local_z

                position = (center[0], center[1], center[2])
                rpy = (pos.rx, pos.ry, pos.rz)
                result = flatten_fn(
                    input_path=current_input,
                    output_path=tmp_path,
                    position=position,
                    rpy=rpy,
                    size=args.size,
                    frame_to_mesh=inverse_matrix,
                )
                # From here on every call reads back what the previous one wrote.
                current_input = tmp_path
                if phase == 2:
                    count += 1  # a single phase counts each area once
                print(
                    f"[pass {phase}] OK for {pos.position_label} on "
                    f"{reader.reader_name}: {len(result.vertices)} vertices, "
                    f"{len(result.faces)} faces"
                )

        if phase == 1 and args.phase1_output:
            phase1_path = Path(args.phase1_output)
            phase1_path.parent.mkdir(parents=True, exist_ok=True)
            # The chained file carries the output extension, so a copy is only
            # valid when the requested format matches; otherwise re-export.
            if phase1_path.suffix.lower() == out_ext.lower():
                shutil.copy(tmp_path, str(phase1_path))
            else:
                trimesh.load(tmp_path, force="mesh").export(str(phase1_path))
            print(f"Phase 1 saved -> {phase1_path}")

    if count == 0:
        raise SystemExit("No positions found in the json file, nothing to do.")

    shutil.move(tmp_path, args.output)
    print(f"Done: {count} areas flattened -> {args.output}")


if __name__ == "__main__":
    main()