"""circuits package"""
# pylint: disable=no-name-in-module
from unyt import (
    unyt_quantity,
    unyt_array,
    V,
    A,
    Ω,
    F,
    H,
    Hz,
    s,
    kg,
    m,
    dB,
    matplotlib_support,
    degC,
    K,
    delta_degC,
)
from circuits.common import (
    AmbientTemperature,
    DeviceTemperature,
    temperature_difference,
)
from circuits.components import (
    PortDirection,
    Port,
    Pin,
    PowerTap,
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
    Opamp,
    ResistorNetwork,
)
from circuits.circuit import Circuit
from circuits.netlist import Netlist
from circuits.version import __version__

__all__ = [
    "unyt_quantity",
    "AmbientTemperature",
    "temperature_difference",
    "Ta",
    "T_device",
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
    "degC",
    "K",
    "delta_degC",
    "Opamp",
    "ResistorNetwork",
]

# pylint: disable=invalid-name
Ta = AmbientTemperature(*unyt_array([23, -40, 70], "degC"))
T_device = DeviceTemperature(*unyt_array([38, -25, 85], "degC"))
pwrtaps = {"gnd": PowerTap("gnd"), "+5V": PowerTap("+5V")}
signalports = {
    "in": Port("in", PortDirection.IN),
    "out": Port("out", PortDirection.OUT),
}
read_netlist = Netlist()
matplotlib_support()
matplotlib_support.label_style = "/"


def __dir__():
    return __all__
