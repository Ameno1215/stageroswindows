#!/usr/bin/env python3
import argparse
import shutil
import subprocess
from pathlib import Path
from urllib.parse import urlparse, unquote
import xml.etree.ElementTree as ET


WORKSPACE_ROOT = Path("/home/antonin/workspace")
ROS_WS_ROOT = WORKSPACE_ROOT / "denso_ros2_ws"
DESCRIPTION_XACRO = ROS_WS_ROOT / "src/denso_robot_drivers_ros2/denso_robot_descriptions/urdf/denso_robot_isaac.urdf.xacro"


def uri_to_path(uri: str) -> Path:
    if uri.startswith("file://"):
        parsed = urlparse(uri)
        if parsed.netloc and parsed.path:
            return Path("//" + parsed.netloc + parsed.path)
        return Path(unquote(parsed.path))
    return Path(uri)


def bundle_relative_path(source_path: Path) -> Path:
    parts = source_path.parts
    if "meshes" in parts:
        return Path("meshes") / source_path.name
    if "tools" in parts:
        tool_index = parts.index("tools")
        return Path(*parts[tool_index:])
    return Path("assets") / source_path.name


def export_bundle(output_dir: Path, model: str, tool: str) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_urdf = output_dir / f"{model}_isaac_raw.urdf"
    bundled_urdf = output_dir / f"{model}_isaac.urdf"

    cmd = [
        "xacro",
        str(DESCRIPTION_XACRO),
        f"model:={model}",
        f"tool:={tool}",
        "sim:=true",
    ]
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    raw_urdf.write_text(result.stdout, encoding="utf-8")

    tree = ET.parse(raw_urdf)
    root = tree.getroot()

    copied = {}
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue

        source_path = uri_to_path(filename)
        if not source_path.exists():
            raise FileNotFoundError(f"Mesh not found: {filename} -> {source_path}")

        rel_path = bundle_relative_path(source_path)
        dest_path = output_dir / rel_path
        dest_path.parent.mkdir(parents=True, exist_ok=True)

        if source_path not in copied:
            shutil.copy2(source_path, dest_path)
            copied[source_path] = dest_path

        mesh.attrib["filename"] = rel_path.as_posix()

    tree.write(bundled_urdf, encoding="utf-8", xml_declaration=True)
    raw_urdf.unlink(missing_ok=True)
    return bundled_urdf


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a Windows-friendly Isaac Sim URDF bundle.")
    parser.add_argument(
        "--output-dir",
        default="/mnt/c/Users/33648/Desktop/STAGE_ROS/isaac_sim/import_bundle",
        help="Destination directory for the bundled URDF and meshes.",
    )
    parser.add_argument("--model", default="vs060", choices=["vs060"], help="Robot model to export.")
    parser.add_argument("--tool", default="effecteur_v2", help="Tool to attach.")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    urdf_path = export_bundle(output_dir, args.model, args.tool)
    print(f"Bundled URDF written to: {urdf_path}")
    print(f"Import this file in Isaac Sim: {urdf_path}")


if __name__ == "__main__":
    main()
