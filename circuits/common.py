"""Common definitions"""
from collections import namedtuple
from enum import Enum
from sympy import S
from unyt import delta_degC

AmbientTemperature = namedtuple("AmbientTemperature", field_names=["nom", "min", "max"])
DeviceTemperature = namedtuple("DeviceTemperature", field_names=["cal", "min", "max"])


class PortDirection(Enum):
    """direction enum"""

    IN = 0
    OUT = 1
    INOUT = 2


def singleton(cls):
    """Decorator function to make class 'cls' a singleton"""

    def single_cls(*args, **kwargs):
        if single_cls.instance is None:
            single_cls.instance = cls(*args, **kwargs)
        return single_cls.instance

    single_cls.instance = None
    return single_cls


j = S.ImaginaryUnit  # sqrt(-1)
π = S.Pi


def temperature_difference(start, end):
    """Compute temperature difference `end` - `start`

    Parameters
    ----------
    start : unyt_quantity
        start temperature in degree Celcius
    end : unyt_quantity
        end temperature in degree Celcius

    Returns
    -------
    temperature_difference : unyt_quantity in unit delta_degC
    """
    start = start.in_base().value
    end = end.in_base().value
    return (end - start) * delta_degC
