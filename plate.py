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
class Plate:
    plate_number: int
    readers: List[Reader]


def plate_from_json(json_data: dict) -> Plate:
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

    return Plate(
        plate_number=json_data["plate_number"],
        readers=readers
    )

def load_plate_from_file(file_path: str) -> Plate:
    with open(file_path, 'r') as f:
        json_data = json.load(f)
    return plate_from_json(json_data)