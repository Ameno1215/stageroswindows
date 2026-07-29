# Creating a Mesh for a Reader Plate

This guide explains how to go from a physical reader plate to a clean, robot-ready mesh: photo capture, photogrammetry in RealityScan, scaling and alignment to the robot frame, and finally re-surfacing the reader areas from the values measured by the robot.

## Step 1 — Photograph the plate

Capture the plate under good, even lighting.

- **Cover glass surfaces with paper** (recommended). Glass reflects and confuses photogrammetry, leaving holes in the mesh. A sheet of paper on top gives a matte, trackable surface.
- Walk **several full loops around the plate**, taking many photos.
- Vary the distance (closer / farther) and the height relative to the ground (lower / higher) between loops so the geometry is covered from many angles.
- **~80–100 photos** is usually a good amount.

## Step 2 — Import into RealityScan

RealityScan is available through the Epic Games launcher.

1. Import all the photos.
2. Select the photos in the menu.
3. Set **Prior pose → Absolute pose → Unknown**.

## Step 3 — First reconstruction

1. In **Alignment**, run **Align Images**.
2. In **Scene 3D / Tools**, run **Set Reconstruction Region** to shrink the volume that gets reconstructed. This cuts computation time.
3. In **Mesh & Color**, run **Normal Detail**, then **Texture**.

## Step 4 — Set the real-world scale

Photogrammetry alone has no absolute scale, so you fix it using a known distance on the plate.

1. In **Alignment**, run **Define Distance**.
2. Pick **two corners along a diagonal** of the plate.
3. Enter the true diagonal length as the value: **0.5830951** (the real plate diagonal, in meters).
4. In the menu, the two control points appear on the two selected points in the photos. **Click all the `+`** to confirm the correspondences.

Then **redo the reconstruction** with the correct scale now applied:

- **Alignment → Align Images**
- **Scene 3D / Tools → Set Reconstruction Region**
- **Mesh & Color → Normal Detail**, then **Texture**

## Step 5 — Place the plate in the robot frame

The plate must sit correctly in the coordinate frame:

- The **corner of the QR code must be at the origin**.
- To check the orientation, **export to `.obj`**, open it in **MeshLab**, and **display the axes**.
- Verify that **X and Y match the robot's frame axes**. If they don't, apply the necessary rotations back in RealityScan and re-export.

## Step 6 — Re-surface the reader areas

The scanned mesh is never perfect, so the actual reading surfaces are rebuilt (flattened) using the values the **robot measured**, provided in the plate's JSON file.

Add these fields to the plate JSON:

```json
"mesh_offset_x": 0.00,
"mesh_offset_y": 0,
"mesh_offset_z": -0.001,
"mesh_rotation_x": 0,
"mesh_rotation_y": 0,
"mesh_rotation_z": 0
```

Then run:

```bash
python .\prepare_mesh\prepare_plate_mesh.py --input .\plates4\plate1\plaque1in.obj --output out.obj --json .\plates4\plate1\plate_1_params.json --clean_space 0.001
```

Adapt the paths to your own files.

### What each argument does

- `--input` — the input mesh exported from RealityScan (e.g. the `.obj` scan). This is the raw, imperfect surface.
- `--output` — the resulting mesh with every reader area flattened. The file format follows the extension (`.obj`, `.stl`, ...).
- `--json` — the plate description file, listing the readers and their measured positions. Each position defines one rectangle that gets flattened onto its own plane.
- `--clean_space` — clearance in meters between a measured position and the surface levelled underneath it, applied along the rectangle's own normal so it stays correct on tilted positions. `0.001` leaves 1 mm of gap; the default is `0`.
- `--size` — dimensions `LX LY` of the flattened rectangle, in meters. Defaults to `0.06 0.09`, which covers a card footprint.
- `--phase1_output` — path where the mesh obtained at the end of phase 1 is saved. Omit it and that intermediate mesh is not written at all. The format follows the extension you give here, so `--phase1_output .\checks\etape1.stl` works even when the final output is an `.obj`, and missing folders are created.

### What the script does

Each position in the JSON defines a rectangle: a centre, an orientation (roll/pitch/yaw) and the `--size` dimensions. Positions are read in the plate frame and converted into the mesh frame using the transform hardcoded at the top of `prepare_plate_mesh.py`:

```python
T_mesh_F[:3, 3] = [0.6245, -0.25, 0.0]
```

If your scan's origin is not where this transform expects it, edit that line — everything downstream depends on it.

Areas are then processed in two phases:

- **Phase 1** levels every area onto its own plane: hollows are filled, bumps are pressed down. Where two rectangles overlap, the one processed last sets the height of the shared region.
- **Phase 2** goes over the same areas in clipping mode: material standing above a plane is cut back down to it, hollows are left alone. This means the **lowest** plane covering a given point decides its final height, so a position higher than its neighbours no longer masks them.

Borders are produced by slicing the mesh exactly along the four sides of each rectangle, then stitching a wall between the levelled region and its surroundings. Each area therefore ends with a clean vertical step rather than a slanted transition, whatever the triangle density of the scan.

### Output files

- `out.obj` — the final mesh, after both phases.
- The phase 1 mesh, only if you asked for it with `--phase1_output`.

When you're setting up a new plate, it's worth running once with `--phase1_output` and comparing the two files: in the phase 1 mesh a tall area may still be covering its neighbours, and in the final one it should be carved back down to them. That comparison is the quickest way to see whether phase 2 did what you expected.

### Checking the result

Open the output in MeshLab and look at the reader areas:

- Each area should be perfectly flat, and offset from its measured position by `--clean_space`.
- Transitions between areas of different heights should be vertical steps, not ramps.
- Where two positions overlap, the lower surface should win.

The console output lists every area as it is processed, with the vertex and face count after each step, which is useful for spotting a position whose rectangle falls outside the scanned geometry (the counts stop changing).