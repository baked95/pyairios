"""Data model for the node data fetching functions."""

from dataclasses import dataclass

from pyairios.properties import AiriosBaseProperty
from pyairios.registers import Result

type AiriosDeviceData = dict[AiriosBaseProperty, Result]


@dataclass
class AiriosData:
    """Data from bridge and all bound nodes."""

    bridge_key: int
    nodes: dict[int, AiriosDeviceData]
