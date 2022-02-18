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
    ResistorNetwork,
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
    None: 0,
}

VALUE_PATTERN = r"([+-]?[0-9]+\.?[0-9]*)(e[+-]?[0-9]+|meg|f|p|n|u|m|k|g|t)?"


def scale_value(value):
    """Convert value to float"""
    try:
        n, s = re.match("^" + VALUE_PATTERN + "$", value, re.IGNORECASE).groups()
    except AttributeError:
        raise ValueError(f"{value} is not a valid value") from None
    try:
        sn = SUFFIXES[s]
    except KeyError:
        return float(f"{n}{s}")
    return float(f"{n}e{sn}")


COMPONENTS = {
    "R": Resistor,
    "C": Capacitor,
    "L": Inductor,
    "V": VoltageSource,
    "opamp": Opamp,
    "RN": ResistorNetwork,
    "RP": ResistorNetwork,
}

TOKEN_TYPES = {
    "refdes": r"^([RCLV]|XU|RN|RP)([0-9]+|[\w]+)\.?([0-9]+)?$",
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
        passive_networks = {}
        for line in self.netlist:
            if line.startswith("*") or line.startswith("."):
                continue
            line = line.split(";")[0]
            tokens = line.split(" ")
            refdes = self.validate_token(tokens[0], "refdes")
            match = re.match(TOKEN_TYPES["refdes"], refdes)
            component_type, number, element = match.groups()
            refdes = f"{component_type}{number}"
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
            elif component_type in ["RN", "RP"]:
                if element is None:
                    continue
                node1 = self.validate_token(tokens[1], "node")
                node2 = self.validate_token(tokens[2], "node")
                value = scale_value(self.validate_token(tokens[3], "value"))
                kwargs = []
                for token in tokens[i + 1 :]:
                    self.validate_token(token, "key_value")
                    k, v = token.split("=")
                    v = scale_value(v)
                    kwargs.append((k, v))
                kwargs = dict(kwargs)
                element_data = [element, node1, node2, value, kwargs]
                if refdes not in passive_networks:
                    passive_networks[refdes] = [element_data]
                else:
                    passive_networks[refdes].append(element_data)
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
                aol = params.pop("Aol", 1e5)
                self.components[refdes] = COMPONENTS["opamp"](refdes, aol)
                self.nodes[refdes] = nodes
            else:
                raise ValueError(f"'{line}' unrecognized")
        for refdes, data in passive_networks.items():
            data = sorted(data)
            nodes = []
            values = []
            kwargs = {}
            for row in data:
                nodes.extend(row[1:3])
                values.append(row[3])
                kwargs.update(row[4])
            match = re.match(TOKEN_TYPES["refdes"], refdes)
            component_type, _, _ = match.groups()
            self.components[refdes] = COMPONENTS[component_type](
                refdes, values, **kwargs
            )
            nodes = nodes[::2] + nodes[-1::-2]
            self.nodes[refdes] = nodes

    def makecircuit(self):
        """Make circuit"""
        for refdes in self.components:
            component = self.components[refdes]
            for i, node in enumerate(self.nodes[refdes], start=1):
                if component.parasitic:
                    self.circuit.connect_parasitic(component.pin(i), node=node)
                else:
                    self.circuit.connect(component.pin(i), node_number=node)
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
