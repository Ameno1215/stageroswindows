"""Flatten the reader areas of a plate scan for robot positioning.

Reads a plate description (json) listing readers and their positions,
then flattens one rectangle per position on the scanned mesh using
flatten_mesh.flatten_rectangle_in_frame:

  - bumps above each rectangle plane are pressed down;
  - hollows below are raised up;

so every reader position ends up on a perfectly flat surface.

The positions in the json are expressed in the plate frame F; they are
converted to the mesh frame with the hardcoded 4x4 transform T_mesh_F
(translation [0.6245, -0.25, 0], no rotation), inverted before use.
The flatten passes are chained through a temporary mesh file, so all
areas accumulate in a single mesh; the result is only moved to --output
once every position has been processed.

Arguments
---------
--input        input mesh (stl, obj, ply, glb, ...), e.g. a RealityScan export
--json         plate description file (readers and their positions)
--output       output mesh; the format follows the extension
--clean_space  margin in meters subtracted from each position's z (frame F),
               to leave clearance between the flattened surface and the
               position (default: 0)
--size         rectangle dimensions LX LY in meters (default: 0.06 0.09)
--refine       subdivision passes at the rectangle borders (default: 3)

Example (from the ROS folder):
    python .\prepare_mesh\prepare_plate_mesh.py ^
        --input .\plates3\plate3\plaque3new.obj ^
        --json .\plates3\plate3\plate_3_params.json ^
        --output out.obj --clean_space 0.005

Dependencies: trimesh, numpy, plus the local modules flatten_mesh and plate
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
from pathlib import Path

import numpy as np
import sys

_HERE = Path(__file__).resolve().parent
for _p in (str(_HERE), str(_HERE.parent)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from flatten_mesh import flatten_rectangle_in_frame, rpy_to_matrix
from plate import load_plate_from_file



T_mesh_F = np.eye(4)
T_mesh_F[:3, :3] = rpy_to_matrix(0.0, 0.0, 0)
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
        "--refine", type=int, default=3,
        help="subdivision passes at the rectangle border (default: 3)",
    )
    args = parser.parse_args()

    plate = load_plate_from_file(args.json)

    # Temp file with the same extension as the output so trimesh keeps the
    # right export format while chaining the flatten passes.
    out_ext = Path(args.output).suffix or ".stl"
    tmp_path = str(Path(tempfile.gettempdir()) / f"flatten_chain{out_ext}")

    current_input = args.input   # first pass reads the original scan
    count = 0

    for reader in plate.readers:
        for pos in reader.positions:
            
            position=(pos.x, pos.y, pos.z - args.clean_space)
            rpy=(pos.rx, pos.ry, pos.rz)
            # print(rpy)
            result = flatten_rectangle_in_frame(
                input_path=current_input,
                output_path=tmp_path,
                position=position,
                rpy=rpy,
                size=args.size,
                frame_to_mesh=inverse_matrix,
                refine_iterations=args.refine,
            )
            current_input = tmp_path  # next pass continues from the temp mesh
            count += 1
            print(
                f"OK for {pos.position_label} on {reader.reader_name}: "
                f"{len(result.vertices)} vertices, {len(result.faces)} faces"
            )

    if count == 0:
        raise SystemExit("No positions found in the json file, nothing to do.")

    shutil.move(tmp_path, args.output)
    print(f"Done: {count} areas flattened -> {args.output}")


if __name__ == "__main__":
    main()