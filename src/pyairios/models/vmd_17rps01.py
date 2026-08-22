"""Airios VMD-17RPS01 controller implementation (Vasco).

The process block of this controller is shorter than the VMD-02RPS78 one and laid out at
different addresses. Everything from 41024 upwards answers IllegalDataAddress, so there is no
CO2, no fan RPM and no bypass mode/status register on this unit; the bypass position lives at
41022 instead of 41016.

Register map reported by @PiotrOrman against a Vasco X350E, verified with 24 h of logging plus
a speed sweep:

  41003        current ventilation speed (see the note on the read scale below)
  41005/41006  supply / exhaust fan percentage
  41007/41009  float sensor slots, not fitted on this unit
  41011        temperature, home side      -> exhaust (before the heat exchanger)
  41013        temperature, outdoor side   -> inlet (before the heat exchanger)
  41015        temperature, home side      -> outlet (after the heat exchanger)
  41017        temperature, outdoor side   -> supply (after the heat exchanger)
  41019-41021  zero, no optional sensor fitted
  41022        bypass position, 0 or 100
  41023        zero, no optional sensor fitted
  41500        requested ventilation speed, write only

Registers 41000 and 41002 exist but answer Modbus exception 5 (acknowledge): they are never
populated over RF on this unit, so they are left out of the map.

Three things in this file are still hypotheses and are marked as such below: the read scale of
41003, the assignment of the four fitted temperature sensors, and whether the status registers
(address + 10000) exist.
"""

from __future__ import annotations

import logging
import math

from pyairios.client import AsyncAiriosModbusClient
from pyairios.constants import (
    AiriosDeviceType,
    ProductId,
    VMDBypassPosition,
    VMDRequestedVentilationSpeed,
    VMDSensorStatus,
    VMDTemperature,
    VMDVentilationSpeed,
)
from pyairios.node import AiriosNode
from pyairios.properties import AiriosVMDProperty as vp
from pyairios.registers import (
    FloatRegister,
    RegisterAccess,
    RegisterBase,
    Result,
    U16Register,
)

LOGGER = logging.getLogger(__name__)


def pr_id() -> ProductId:
    """
    Get product_id for model VMD_17RPS01.
    Named as is to discern from product_id register.
    """
    return ProductId.VMD_17RPS01


def pr_type() -> AiriosDeviceType:
    """
    Get the device type.
    """
    return AiriosDeviceType.CONTROLLER


def pr_description() -> list[str]:
    """
    Get description of product(s) using VMD_17RPS01.
    Human-readable text, used in e.g. HomeAssistant Binding UI.
    :return: string or tuple of strings, starting with manufacturer
    """
    return ["Vasco X350E"]


def pr_instantiate(device_id: int, client: AsyncAiriosModbusClient) -> VMD17RPS01:
    """Get a new device instance. Used by the device factory to instantiate by product ID."""
    return VMD17RPS01(device_id, client)


def _temperature_adapter(value: float) -> VMDTemperature:
    if math.isnan(value):
        status = VMDSensorStatus.UNAVAILABLE
    elif value < -273.0:
        status = VMDSensorStatus.ERROR
    else:
        status = VMDSensorStatus.OK
        value = round(value, 2)
    return VMDTemperature(value, status)


def _bypass_position_adapter(value) -> VMDBypassPosition:
    error = value > 120
    return VMDBypassPosition(value, error)


class VMD17RPS01(AiriosNode):
    """Represents a VMD-17RPS01 controller node."""

    def __init__(self, device_id: int, client: AsyncAiriosModbusClient) -> None:
        """Initialize the VMD-17RPS01 controller node instance."""
        super().__init__(device_id, client)

        # NOTE: declared without RegisterAccess.STATUS. The status of a value is read from
        # address + 10000, and whether this unit implements those registers has not been
        # tested. Adding the flag before checking would make every fetch() fail the way the
        # serial registers did on the Ethernet bridge.
        vmd_registers: list[RegisterBase] = [
            # HYPOTHESIS: 41003 uses the standard VMDVentilationSpeed read scale, the same
            # one 41000 uses on the VMD-02RPS78. Off/low/mid/high (0/1/2/3) match it exactly,
            # but a boost request reads back as 5, which is not a member (boost is 23 there).
            # If instead this register is a compact 0-5 scale of its own, it needs an adapter.
            # A read of 41003 after requesting AWAY settles it: 21 means the enum, 1 means the
            # compact scale.
            U16Register(
                vp.CURRENT_VENTILATION_SPEED,
                41003,
                RegisterAccess.READ,
                result_type=VMDVentilationSpeed,
            ),
            U16Register(vp.FAN_SPEED_SUPPLY, 41005, RegisterAccess.READ),
            U16Register(vp.FAN_SPEED_EXHAUST, 41006, RegisterAccess.READ),
            # HYPOTHESIS: the four fitted sensors follow the same order as the VMD-02RPS78
            # (exhaust, inlet, outlet, supply). It matches the reported behaviour: 41013 and
            # 41017 converge when the bypass is open, which is what inlet and supply do, while
            # 41011 and 41015 carry home-side air. To confirm, compare 41011 against the room
            # temperature and 41013 against the outdoor temperature.
            FloatRegister(
                vp.TEMPERATURE_EXHAUST,
                41011,
                RegisterAccess.READ,
                result_adapter=_temperature_adapter,
            ),
            FloatRegister(
                vp.TEMPERATURE_INLET,
                41013,
                RegisterAccess.READ,
                result_adapter=_temperature_adapter,
            ),
            FloatRegister(
                vp.TEMPERATURE_OUTLET,
                41015,
                RegisterAccess.READ,
                result_adapter=_temperature_adapter,
            ),
            FloatRegister(
                vp.TEMPERATURE_SUPPLY,
                41017,
                RegisterAccess.READ,
                result_adapter=_temperature_adapter,
            ),
            U16Register(
                vp.BYPASS_POSITION,
                41022,
                RegisterAccess.READ,
                result_adapter=_bypass_position_adapter,
            ),
            U16Register(
                vp.REQUESTED_VENTILATION_SPEED,
                41500,
                RegisterAccess.WRITE,
                result_type=VMDRequestedVentilationSpeed,
            ),
        ]
        self._add_registers(vmd_registers)

    def pr_id(self) -> ProductId:
        """Return the product ID."""
        return pr_id()

    def pr_type(self) -> AiriosDeviceType:
        """Return the product type."""
        return pr_type()

    def pr_description(self) -> list[str]:
        """Return the product description."""
        return pr_description()

    def __str__(self) -> str:
        return f"VMD-17RPS01@{self.device_id}"

    async def ventilation_speed(self) -> Result[VMDVentilationSpeed]:
        """Get the current ventilation speed.

        Register 41500 is write only on this unit, so the actual state comes from 41003.
        """
        return await self.client.get_register(
            self.regmap[vp.CURRENT_VENTILATION_SPEED], self.device_id
        )

    async def set_ventilation_speed(self, speed: VMDRequestedVentilationSpeed) -> bool:
        """Set the ventilation speed.

        The write and read scales differ, as on the VMD-02RPS78: writing LOW (2) reads back
        from 41003 as VMDVentilationSpeed.LOW (1).
        """
        return await self.client.set_register(
            self.regmap[vp.REQUESTED_VENTILATION_SPEED], speed, self.device_id
        )

    async def bypass_position(self) -> Result[VMDBypassPosition]:
        """Get the bypass position, 0 (closed) or 100 (open)."""
        return await self.client.get_register(self.regmap[vp.BYPASS_POSITION], self.device_id)

    async def supply_fan_speed(self) -> Result[int]:
        """Get the supply fan percentage."""
        return await self.client.get_register(self.regmap[vp.FAN_SPEED_SUPPLY], self.device_id)

    async def exhaust_fan_speed(self) -> Result[int]:
        """Get the exhaust fan percentage."""
        return await self.client.get_register(self.regmap[vp.FAN_SPEED_EXHAUST], self.device_id)

    async def exhaust_air_temperature(self) -> Result[VMDTemperature]:
        """Get the temperature of the air extracted from the home, before the exchanger."""
        return await self.client.get_register(self.regmap[vp.TEMPERATURE_EXHAUST], self.device_id)

    async def outdoor_air_temperature(self) -> Result[VMDTemperature]:
        """Get the temperature of the air taken from outside, before the exchanger."""
        return await self.client.get_register(self.regmap[vp.TEMPERATURE_INLET], self.device_id)

    async def indoor_air_temperature(self) -> Result[VMDTemperature]:
        """Get the temperature of the air discharged outside, after the exchanger."""
        return await self.client.get_register(self.regmap[vp.TEMPERATURE_OUTLET], self.device_id)

    async def supply_air_temperature(self) -> Result[VMDTemperature]:
        """Get the temperature of the air supplied to the home, after the exchanger."""
        return await self.client.get_register(self.regmap[vp.TEMPERATURE_SUPPLY], self.device_id)
