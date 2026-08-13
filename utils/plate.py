from dataclasses import dataclass
from typing import List
import json
from math import pi


@dataclass
class Position:
    position_label: str
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    @classmethod
    def from_dict(cls, data: dict) -> "Position":
        return cls(
            position_label=data["position_label"],
            x=data["x"] / 10000,
            y=data["y"] / 10000,
            z=data["z"] / 10000,
            rx=data["rx"] / 10*pi/180,
            ry=data["ry"] / 10*pi/180,
            rz=data["rz"] / 10*pi/180,
        )


@dataclass
class Reader:
    reader_name: str
    reader_brand: str
    reader_model: str
    baudrate: int
    reader_market: str
    reader_number: str
    minimum_height: float
    maximum_height: float
    include_in_test: bool
    positions: List[Position]

@dataclass
class Phone:
    commercial_name: str
    model_number: str
    os_version: str
    kernel_version: int
    adb_device_name: str
    phone_number: str
    minimum_height: float
    maximum_height: float
    include_in_test: bool
    position: Position

@dataclass
class PlateReader:
    plate_number: int
    mesh_path: str
    mesh_rotation_x: float # must be in m
    mesh_rotation_y: float # must be in m
    mesh_rotation_z: float # must be in m
    mesh_offset_x: float # must be in deg
    mesh_offset_y: float # must be in deg
    mesh_offset_z: float # must be in deg
    readers: List[Reader]

@dataclass
class PlatePhone:
    plate_number: str
    mesh_path: str
    mesh_rotation_x: float # must be in m
    mesh_rotation_y: float # must be in m
    mesh_rotation_z: float # must be in m
    mesh_offset_x: float # must be in deg
    mesh_offset_y: float # must be in deg
    mesh_offset_z: float # must be in deg
    phones: List[Phone]

def reader_plate_from_json(json_data: dict) -> PlateReader:
    readers = []

    for reader_data in json_data.get("list_readers", []):
        positions = [
            Position.from_dict(pos)
            for pos in reader_data.get("list_position", [])
        ]

        reader = Reader(
            reader_name=reader_data["reader_name"],
            reader_brand=reader_data["reader_brand"],
            reader_model=reader_data["reader_model"],
            baudrate=reader_data["baudrate"],
            reader_market=reader_data["reader_market"],
            reader_number=reader_data["reader_number"],
            minimum_height=reader_data["minimum_height"]/1000, 
            maximum_height=reader_data["maximum_height"]/1000,
            include_in_test=reader_data["include_in_test"],
            positions=positions,
        )

        readers.append(reader)

    return PlateReader(
        plate_number=json_data["plate_number"],
        mesh_path=json_data["mesh_path"],
        mesh_rotation_x=json_data["mesh_rotation_x"],
        mesh_rotation_y=json_data["mesh_rotation_y"],
        mesh_rotation_z=json_data["mesh_rotation_z"],
        mesh_offset_x=json_data["mesh_offset_x"],
        mesh_offset_y=json_data["mesh_offset_y"],
        mesh_offset_z=json_data["mesh_offset_z"],
        readers=readers
    )

def phone_plate_from_json(json_data: dict) -> PlatePhone:
    phones = []

    for phone_data in json_data.get("list_phones", []):
        position = Position.from_dict(phone_data["position"])

        phone = Phone(
            commercial_name=phone_data["commercial_name"],
            model_number=phone_data["model_number"],
            os_version=phone_data["os_version"],
            kernel_version=phone_data["kernel_version"],
            adb_device_name=phone_data["adb_device_name"],
            phone_number=phone_data["phone_number"],
            minimum_height=phone_data["minimum_height"],
            maximum_height=phone_data["maximum_height"],
            include_in_test=phone_data["include_in_test"],
            position=position,
        )

        phones.append(phone)

    return PlatePhone(
        plate_number=json_data["plate_number"],
        mesh_path=json_data["mesh_path"],
        mesh_rotation_x=json_data["mesh_rotation_x"],
        mesh_rotation_y=json_data["mesh_rotation_y"],
        mesh_rotation_z=json_data["mesh_rotation_z"],
        mesh_offset_x=json_data["mesh_offset_x"],
        mesh_offset_y=json_data["mesh_offset_y"],
        mesh_offset_z=json_data["mesh_offset_z"],
        phones=phones
    )


def load_reader_plate_from_file(file_path: str) -> PlateReader:
    with open(file_path, 'r') as f:
        json_data = json.load(f)
    return reader_plate_from_json(json_data)


def load_phone_plate_from_file(file_path: str) -> PlatePhone:
    with open(file_path, 'r') as f:
        json_data = json.load(f)
    return phone_plate_from_json(json_data)