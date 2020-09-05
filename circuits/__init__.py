"""circuits package"""
# pylint: disable=no-name-in-module
from unyt import (
    unyt_quantity,
    V,
    A,
    Ω,
    F,
    H,
    Hz,
    s,
    kg,
    m,
)
from circuits.common import AmbientTemperature
from circuits.components import (
    PortDirection,
    Port,
    Pin,
    PowerTap,
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
)
from circuits.circuit import Circuit
from circuits.netlist import Netlist
from circuits.version import __version__

__all__ = [
    "unyt_quantity",
    "AmbientTemperature",
    "Ta",
    "PortDirection",
    "Port",
    "Pin",
    "PowerTap",
    "pwrtaps",
    "signalports",
    "Resistor",
    "Capacitor",
    "Inductor",
    "VoltageSource",
    "Circuit",
    "read_netlist",
    "V",
    "A",
    "Ω",
    "F",
    "H",
    "Hz",
    "s",
    "kg",
    "m",
]

# pylint: disable=invalid-name
Ta = AmbientTemperature(*(unyt_quantity(t, "degC") for t in [23, -40, 70]))
pwrtaps = {"gnd": PowerTap("gnd"), "+5V": PowerTap("+5V")}
signalports = {
    "in": Port("in", PortDirection.IN),
    "out": Port("out", PortDirection.OUT),
}
read_netlist = Netlist()


def __dir__():
    return __all__
