"""Common definitions"""
from collections import namedtuple
from enum import Enum


AmbientTemperature = namedtuple("AmbientTemperature", field_names=["nom", "min", "max"])


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