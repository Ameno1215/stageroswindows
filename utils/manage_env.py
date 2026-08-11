from dataclasses import dataclass
from typing import List
import json
from math import pi
from pathlib import Path

@dataclass
class Position:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    @classmethod
    def from_dict(cls, data: dict) -> "Position":
        return cls(
            x=data["x"],
            y=data["y"],
            z=data["z"],
            rx=data["rx"],
            ry=data["ry"],
            rz=data["rz"],
        )
    
@dataclass
class Scale:
    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, data: dict) -> "Scale":
        return cls(
            x=data["x"],
            y=data["y"],
            z=data["z"],
        )
    
@dataclass
class Size:
    x: float
    y: float
    z: float

    @classmethod
    def from_dict(cls, data: dict) -> "Size":
        return cls(
            x=data["x"],
            y=data["y"],
            z=data["z"],
        )
    
@dataclass
class Mesh:
    id: str
    path: str
    position: Position
    scale: Scale    
    a: float
    r: float
    g: float
    b: float

@dataclass
class Box:
    id: str
    position: Position
    size: Size
    enable_collision: bool    
    a: float
    r: float
    g: float
    b: float

@dataclass
class Fence:
    front: float
    back: float
    left: float
    right: float
    top: float
    bottom: float


@dataclass
class Environnement:
    boxes: List[Box]
    meshes: List[Mesh]
    fence: Fence

def to_path_real(input):
    base_path = Path.cwd()
    abs_path_to_stl = base_path / input

    posix_path = abs_path_to_stl.as_posix()

    if posix_path.lower().startswith("c:/"):
        wsl_path = "/mnt/c/" + posix_path[3:]
    else:
        wsl_path = posix_path

    return f"file://{wsl_path}"


def env_from_json(json_data: dict) -> Environnement:
    boxes = []
    meshes = []
    fence = None

    for mesh_data in json_data.get("mesh_list", []):
        position = Position.from_dict(mesh_data["position"])
        if mesh_data["angle_format"] ==  "DEG":
            position.rx *= pi/180
            position.ry *= pi/180
            position.rz *= pi/180
        scale = Scale.from_dict(mesh_data["scale"])

        mesh = Mesh(
            id=mesh_data["id"],
            path=to_path_real(mesh_data["path"]),
            scale=scale,
            position=position,
            a=mesh_data["a"],
            r=mesh_data["r"],
            g=mesh_data["g"],
            b=mesh_data["b"],
        )

        meshes.append(mesh)

    for boxes_data in json_data.get("box_list", []):
        position = Position.from_dict(boxes_data["position"])
        if boxes_data["angle_format"] ==  "DEG":
            position.rx *= pi/180
            position.ry *= pi/180
            position.rz *= pi/180
        size = Size.from_dict(boxes_data["size"])

        box = Box(
            id=boxes_data["id"],
            size=size,
            position=position,
            enable_collision=boxes_data["enable_collision"],
            a=boxes_data["a"],
            r=boxes_data["r"],
            g=boxes_data["g"],
            b=boxes_data["b"]
        )

        boxes.append(box)

    fence_data = json_data["fence"]
    fence = Fence(
        front=fence_data["front"],
        back=fence_data["back"],
        left=fence_data["left"],
        right=fence_data["right"],
        top=fence_data["top"],
        bottom=fence_data["bottom"],
    )
    
    return Environnement(
        boxes=boxes,
        meshes=meshes,
        fence=fence
    )

def load_env_from_file(file_path: str) -> Environnement:
    with open(file_path, 'r') as f:
        json_data = json.load(f)
    return env_from_json(json_data)