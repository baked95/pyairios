"""Airios VMD-17RPS01 controller implementation (Vasco).

The process block of this controller is shorter than the VMD-02RPS78 one and laid out at
different addresses. Everything from 41024 upwards answers IllegalDataAddress, so there is no
CO2, no fan RPM and no bypass mode/status register on this unit; the bypass position lives at
41022 instead of 41016.

Register map reported and verified on real hardware by @PiotrOrman against a Vasco X350E,
cross-checked against the Vasco Climate Control app:

  41003        current ventilation speed, own 0-5 scale (see _ventilation_speed_adapter)
  41005/41006  supply / exhaust fan percentage
  41007/41009  float sensor slots, not fitted on this unit
  41011        temperature, exhaust (extracted from the home, before the exchanger)
  41013        temperature, inlet (taken from outside, before the exchanger)
  41015        temperature, outlet (discharged outside, after the exchanger)
  41017        temperature, supply (delivered to the home, after the exchanger)
  41019-41021  zero, no optional sensor fitted
  41022        bypass position, 0 or 100
  41023        zero, no optional sensor fitted
  41500        requested ventilation speed, write only
  42000/42007  high preset, exhaust and supply (which is which is not established)
  42003-42006  low and mid presets, supply and exhaust
  42008        free ventilation (bypass) room threshold, float
  42010        frost protection / electric pre-heater activation, float
  42012        pre-heater setpoint (minimum supply temperature), float

The temperature assignment was verified against independent sensors: with the bypass open the
inlet/supply pair sits together and the exhaust/outlet pair sits together, and 41013 tracked an
independent outdoor sensor within 0.3 K.

NO AWAY PRESET. On the VMD-02RPS78 registers 42001/42002 hold the away fan speeds; on this unit
they answer IllegalDataAddress. That is why requesting AWAY and requesting LOW are
indistinguishable here — both drive the fans to the low preset — and why the read scale maps 1
to LOW rather than AWAY.

Registers 41000 and 41002 exist but answer Modbus exception 5 (acknowledge): they are never
populated over RF on this unit, so they are left out of the map. Writes must use FC16; FC06 is
rejected, as on the bridge.

Every preset stores supply and exhaust separately, so a unit can deliberately be run
unbalanced. That is the structure to expect, and it is what the scan shows: 30/30 for low,
60/60 for mid and 325/325 for high. Low and mid sit at the same addresses the VMD-02RPS78 uses
for them; high does not, and its pair is 42000 and 42007.

UNITS, still open. The Vasco app expresses low and mid as a percentage of the high preset
rather than of the fan maximum, and high as an absolute airflow in m3/h, which is why 42000 and
42007 carry no max_value of 100. The addresses and the pairings are solid; the direction within
the high pair, and whether the percentage base is really the high preset, are pending a
write-then-readback test.

The 42xxx registers are declared WITHOUT RegisterAccess.STATUS: 51003 and 51011 were verified
to respond (both 0x1107), so the 41xxx status registers are safe, but nothing in the 52xxx
range has been probed yet.
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


# Register 41003 uses a compact scale of its own, not VMDVentilationSpeed. Verified by
# writing each preset to 41500 and reading 41003 back on real hardware:
#     write 0 Off -> 0     write 1 Away -> 1     write 2 Low  -> 1
#     write 3 Mid -> 2     write 4 High -> 3     write 7 Boost -> 5
# Away and Low are indistinguishable because this unit has no away preset (see the note on
# registers 42001/42002 above), so 1 maps to LOW.
_VENTILATION_SPEED_CODES = {
    0: VMDVentilationSpeed.OFF,
    1: VMDVentilationSpeed.LOW,
    2: VMDVentilationSpeed.MID,
    3: VMDVentilationSpeed.HIGH,
    5: VMDVentilationSpeed.BOOST,
}


def _ventilation_speed_adapter(value: int) -> VMDVentilationSpeed:
    try:
        return _VENTILATION_SPEED_CODES[int(value)]
    except KeyError as ex:
        raise ValueError(f"Unknown VMD-17RPS01 ventilation speed code {value}") from ex


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
                RegisterAccess.READ | RegisterAccess.STATUS,
                result_adapter=_ventilation_speed_adapter,
            ),
            U16Register(
                vp.FAN_SPEED_SUPPLY, 41005, RegisterAccess.READ | RegisterAccess.STATUS
            ),
            U16Register(
                vp.FAN_SPEED_EXHAUST, 41006, RegisterAccess.READ | RegisterAccess.STATUS
            ),
            # HYPOTHESIS: the four fitted sensors follow the same order as the VMD-02RPS78
            # (exhaust, inlet, outlet, supply). It matches the reported behaviour: 41013 and
            # 41017 converge when the bypass is open, which is what inlet and supply do, while
            # 41011 and 41015 carry home-side air. To confirm, compare 41011 against the room
            # temperature and 41013 against the outdoor temperature.
            FloatRegister(
                vp.TEMPERATURE_EXHAUST,
                41011,
                RegisterAccess.READ | RegisterAccess.STATUS,
                result_adapter=_temperature_adapter,
            ),
            FloatRegister(
                vp.TEMPERATURE_INLET,
                41013,
                RegisterAccess.READ | RegisterAccess.STATUS,
                result_adapter=_temperature_adapter,
            ),
            FloatRegister(
                vp.TEMPERATURE_OUTLET,
                41015,
                RegisterAccess.READ | RegisterAccess.STATUS,
                result_adapter=_temperature_adapter,
            ),
            FloatRegister(
                vp.TEMPERATURE_SUPPLY,
                41017,
                RegisterAccess.READ | RegisterAccess.STATUS,
                result_adapter=_temperature_adapter,
            ),
            U16Register(
                vp.BYPASS_POSITION,
                41022,
                RegisterAccess.READ | RegisterAccess.STATUS,
                result_adapter=_bypass_position_adapter,
            ),
            U16Register(
                vp.REQUESTED_VENTILATION_SPEED,
                41500,
                RegisterAccess.WRITE,
                result_type=VMDRequestedVentilationSpeed,
            ),
            # Configuration block. Same addresses the VMD-02RPS78 uses for the low and mid
            # presets; 42001/42002 (away) do not exist here. See the note on units above.
            U16Register(
                vp.FAN_SPEED_LOW_SUPPLY, 42003, RegisterAccess.READ | RegisterAccess.WRITE
            ),
            U16Register(
                vp.FAN_SPEED_LOW_EXHAUST, 42004, RegisterAccess.READ | RegisterAccess.WRITE
            ),
            U16Register(
                vp.FAN_SPEED_MID_SUPPLY, 42005, RegisterAccess.READ | RegisterAccess.WRITE
            ),
            U16Register(
                vp.FAN_SPEED_MID_EXHAUST, 42006, RegisterAccess.READ | RegisterAccess.WRITE
            ),
            # High is a balanced pair too, like low and mid: 42000 and 42007 both read 325.
            # Every preset holds a separate supply and exhaust value precisely so the unit can
            # be run unbalanced on purpose, so a preset with a single register would be the odd
            # one out. Which of the two is supply and which is exhaust is NOT established yet:
            # a write-then-readback test (write one, watch 41005 vs 41006) decides it.
            # Absolute airflow in m3/h on this unit, not a percentage, so no max_value of 100.
            U16Register(
                vp.FAN_SPEED_HIGH_EXHAUST,
                42000,
                RegisterAccess.READ | RegisterAccess.WRITE,
                max_value=2000,
            ),
            U16Register(
                vp.FAN_SPEED_HIGH_SUPPLY,
                42007,
                RegisterAccess.READ | RegisterAccess.WRITE,
                max_value=2000,
            ),
            # Room temperature above which the bypass opens. Verified causally: writing 30
            # closed the damper within 5 s with the room at 24.3 C, and 41022 followed.
            FloatRegister(
                vp.FREE_VENTILATION_HEATING_SETPOINT,
                42008,
                RegisterAccess.READ | RegisterAccess.WRITE,
            ),
            FloatRegister(
                vp.FROST_PROTECTION_PREHEATER_SETPOINT,
                42010,
                RegisterAccess.READ | RegisterAccess.WRITE,
            ),
            FloatRegister(
                vp.PREHEATER_SETPOINT,
                42012,
                RegisterAccess.READ | RegisterAccess.WRITE,
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

    async def free_ventilation_setpoint(self) -> Result[float]:
        """Get the room temperature above which the bypass opens (register 42008)."""
        return await self.client.get_register(
            self.regmap[vp.FREE_VENTILATION_HEATING_SETPOINT], self.device_id
        )

    async def set_free_ventilation_setpoint(self, value: float) -> bool:
        """Set the room temperature above which the bypass opens."""
        return await self.client.set_register(
            self.regmap[vp.FREE_VENTILATION_HEATING_SETPOINT], value, self.device_id
        )

    async def preheater_setpoint(self) -> Result[float]:
        """Get the minimum supply temperature the pre-heater maintains (register 42012)."""
        return await self.client.get_register(self.regmap[vp.PREHEATER_SETPOINT], self.device_id)

    async def set_preheater_setpoint(self, value: float) -> bool:
        """Set the minimum supply temperature the pre-heater maintains."""
        return await self.client.set_register(
            self.regmap[vp.PREHEATER_SETPOINT], value, self.device_id
        )

    async def frost_protection_setpoint(self) -> Result[float]:
        """Get the outdoor temperature that activates the pre-heater (register 42010)."""
        return await self.client.get_register(
            self.regmap[vp.FROST_PROTECTION_PREHEATER_SETPOINT], self.device_id
        )

    async def set_frost_protection_setpoint(self, value: float) -> bool:
        """Set the outdoor temperature that activates the pre-heater."""
        return await self.client.set_register(
            self.regmap[vp.FROST_PROTECTION_PREHEATER_SETPOINT], value, self.device_id
        )

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
