"""Netlist"""
import io
import pathlib
import re
from circuits import (
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
    Circuit,
    Opamp,
)
from circuits.common import singleton

SUFFIXES = {
    "f": "-15",
    "p": "-12",
    "n": "-9",
    "u": "-6",
    "m": "-3",
    "k": "3",
    "meg": "6",
    "g": "9",
    "t": "12",
}

VALUE_PATTERN = r"([+-]?[0-9]+\.?[0-9]*)(e[+-]?[0-9]+|meg|f|p|n|u|m|k|g|t)?"


def scale_value(value):
    """Convert value to float"""
    try:
        n, s = re.match("^" + VALUE_PATTERN + "$", value, re.IGNORECASE).groups()
    except AttributeError:
        raise ValueError(f"{value} is not a valid value") from None
    s = SUFFIXES.get(s, 0)
    return float(f"{n}e{s}")


COMPONENTS = {
    "R": Resistor,
    "C": Capacitor,
    "L": Inductor,
    "V": VoltageSource,
    "opamp": Opamp,
}

TOKEN_TYPES = {
    "refdes": "^([RCLV]|XU)[0-9]+$",
    "node": r"^[\w]+$",
    "value": VALUE_PATTERN,
    "key_value": r"^[\w]+=" + VALUE_PATTERN + "$",
}


@singleton
class Netlist:
    """Convert netlist to Circuit"""

    def __init__(self):
        self.netlist = []
        self.components = {}
        self.nodes = {}
        self.circuit = Circuit()

    def initialize_attributes(self):
        """Initialize attributes"""
        self.netlist = []
        self.components = {}
        self.nodes = {}
        self.circuit = Circuit()

    @staticmethod
    def validate_token(value, token_type):
        """Validate token"""
        ret_value = value if re.match(TOKEN_TYPES[token_type], value) else None
        if ret_value is None:
            raise ValueError(f"invalid '{value}'")
        return ret_value

    def parse(self):
        """Parse the netlist"""
        for line in self.netlist:
            if line.startswith("*") or line.startswith("."):
                continue
            line = line.split(";")[0]
            tokens = line.split(" ")
            refdes = self.validate_token(tokens[0], "refdes")
            component_type = refdes[0]
            if component_type in ["R", "C", "L"]:
                node1 = self.validate_token(tokens[1], "node")
                node2 = self.validate_token(tokens[2], "node")
                value = scale_value(self.validate_token(tokens[3], "value"))
                self.components[refdes] = COMPONENTS[component_type](refdes, value)
                for token in tokens[4:]:
                    self.validate_token(token, "key_value")
                    param, value = token.split("=")
                    setattr(self.components[refdes], param.lower(), scale_value(value))
                self.nodes[refdes] = (node1, node2)
            elif component_type == "V":
                node1 = self.validate_token(tokens[1], "node")
                node2 = self.validate_token(tokens[2], "node")
                i, value = (
                    (4, tokens[4]) if tokens[3] in ["DC", "AC"] else (3, tokens[3])
                )
                value = scale_value(self.validate_token(value, "value"))
                self.components[refdes] = COMPONENTS[component_type](refdes, value)
                for token in tokens[i + 1 :]:
                    self.validate_token(token, "key_value")
                    param, value = token.split("=")
                    setattr(self.components[refdes], param.lower(), scale_value(value))
                self.nodes[refdes] = (node1, node2)
            elif component_type.startswith("X"):
                nodes = []
                i = 0
                for i, token in enumerate(tokens[1:]):
                    if token in ["opamp"]:
                        break
                    nodes.append(self.validate_token(token, "node"))
                else:
                    raise ValueError(f"'{line}' unrecognized")
                refdes = refdes[1:]
                params = {}
                for token in tokens[i + 2 :]:
                    self.validate_token(token, "key_value")
                    param, value = token.split("=")
                    params[param] = value
                aol = params.pop("aol", 1e5)
                self.components[refdes] = COMPONENTS["opamp"](refdes, aol)
                self.nodes[refdes] = nodes
            else:
                raise ValueError(f"'{line}' unrecognized")

    def makecircuit(self):
        """Make circuit"""
        for refdes in self.components:
            component = self.components[refdes]
            for i, node in enumerate(self.nodes[refdes]):
                self.circuit.connect(component.pin(i + 1), node_number=int(node))
        return self.circuit

    def __call__(self, file):
        self.initialize_attributes()
        if isinstance(file, io.TextIOBase):
            lines = file.readlines()
        elif pathlib.Path(file).is_file():
            with open(file, "rt", encoding="utf-8-sig") as f:
                lines = f.readlines()
        else:
            lines = file.split("\n")
        for line in lines:
            line = line.strip()
            if line != "":
                self.netlist.append(line.strip())
        self.parse()
        return self.makecircuit()
