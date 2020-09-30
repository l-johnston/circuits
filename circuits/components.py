"""Definition of component types"""
import numbers
from sympy import Symbol, sympify
from unyt import unyt_quantity, K
from circuits.common import PortDirection


class Port:
    """Base class for ports

    Concept:
    - signals flow through ports
    - ports connect to other ports
    - name and direction
    """

    def __init__(self, name, direction=PortDirection.INOUT):
        self._name = name
        self._direction = direction

    @property
    def name(self):
        """Return the port's name"""
        return self._name

    @property
    def direction(self):
        """Return the port's direction"""
        return self._direction.name

    def __repr__(self):
        return f"Port('{self._name}', {self._direction})"

    def __eq__(self, other):
        if isinstance(other, Port):
            if self._name == other._name and self._direction == other._direction:
                return True
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self._name, self._direction))


class Pin:
    """Component pin

    Args:
        name (str): pin name
        number (int): pin number
        direction (PortDirection): signal direction
        owner (Component): component pin belongs to
    """

    def __init__(self, name, number, owner, direction=PortDirection.INOUT):
        self._name = name
        self._number = number
        if not issubclass(type(owner), Component):
            raise TypeError(f"{owner} must be a subclass of Component")
        self._owner = owner
        self._direction = direction

    @property
    def name(self):
        """Return the pin's name"""
        return self._name

    @property
    def number(self):
        """Return the pin number"""
        return self._number

    @property
    def owner(self):
        """Return the pin's owner"""
        return self._owner

    @property
    def direction(self):
        """Return the pin's direction"""
        return self._direction.name

    def __repr__(self):
        return f"{self._owner.name}.pin({self._number})"

    def __eq__(self, other):
        if isinstance(other, Pin):
            if self.__dict__ == other.__dict__:
                return True
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self._name, self._number, self._owner, self._direction))


class PowerTap(Port):
    """Power tap"""

    def __init__(self, name):
        super().__init__(name)

    def __repr__(self):
        return f"PowerTap('{self._name}')"


class Component:
    """Base class for components

    Parameters
    ----------
    name : str
        name of component - follow schematic convention of capital letter
        followed by number such as R1, U1, etc.
    pins :list
        list of Pins
    kwargs
    """

    def __init__(self, name, pins, **kwargs):
        self._name = name
        self._symbol = Symbol(name)
        self._pins = {}
        for pin in pins:
            if isinstance(pin, Pin):
                self._pins[pin.name] = pin
                self._pins[pin.number] = pin
            else:
                raise TypeError(f"{pin} must be a Pin")
        for k, v in kwargs.items():
            setattr(self, k, v)

    @property
    def name(self):
        """Return the component's name"""
        return self._name

    @name.setter
    def name(self, name):
        self._name = name
        self._symbol = Symbol(name)

    @property
    def pins(self):
        """Return the component's pin dict"""
        return self._pins

    def __repr__(self):
        return f"<Component {self._name}>"

    def pin(self, name):
        """Return the Pin for pin name/number from the pins dict"""
        try:
            return self._pins[name]
        except KeyError:
            raise ValueError(f"unknown pin {name}") from None


class PassiveComponent(Component):
    """Class for passive, two-port resistors, capacitors and inductors

    Parameters
    ----------
    name : str, name of passive component
    value : float or unyt_quantity, nominal value
    """

    def __init__(self, name, value):
        pins = [Pin("1", 1, self), Pin("2", 2, self)]
        super().__init__(name, pins)
        self._value = value
        self._tol = 0.0
        self._tc = 0.0 / K
        self._refs = []
        self._laplace_s = Symbol("s")
        self._laplace_admittance = None

    @property
    def value(self):
        """Return value of component"""
        return self._value

    @value.setter
    def value(self, value):
        correct_unit = (self._value / value).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"'{value}' must be in unit '{self._value.units}'")
        self._value = value
        for ref, func in self._refs:
            try:
                ref.value = func(value)
            except NameError:
                pass

    @property
    def tol(self):
        """value (float): tolerance"""
        return self._tol

    @tol.setter
    def tol(self, value):
        self._tol = value

    @property
    def tc(self):
        """value (unyt_quantity or float): temperature coefficient, drift per kelvin"""
        return self._tc

    @tc.setter
    def tc(self, value):
        self._tc = unyt_quantity(value, "1/K")

    @property
    def admittance(self):
        """Return the laplace admittance"""
        return self._laplace_admittance

    def __repr__(self):
        return f"<{self.__class__.__name__}:{self._name},{self._value}>"

    def __add__(self, other):
        if isinstance(other, self.__class__):
            Cls = self.__class__
            value = self.value + other.value
            name = f"{self.name}+{other.name}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: value + other.value))
            other._refs.append((new_component, lambda value: self.value + value))
        elif isinstance(other, unyt_quantity):
            Cls = self.__class__
            value = self.value + other
            name = f"{self.name}+{str(other)}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: value + other))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def __radd__(self, other):
        if isinstance(other, unyt_quantity):
            Cls = self.__class__
            value = other + self.value
            name = f"{str(other)}+{self.name}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: other + value))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def __sub__(self, other):
        if isinstance(other, self.__class__):
            Cls = self.__class__
            value = self.value - other.value
            name = f"{self.name}-{other.name}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: value - other.value))
            other._refs.append((new_component, lambda value: self.value - value))
        elif isinstance(other, unyt_quantity):
            Cls = self.__class__
            value = self.value - other
            name = f"{self.name}-{str(other)}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: value - other))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def __rsub__(self, other):
        if isinstance(other, unyt_quantity):
            Cls = self.__class__
            value = other - self.value
            name = f"{str(other)}-{self.name}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: other - value))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def __mul__(self, other):
        if isinstance(other, numbers.Number):
            Cls = self.__class__
            value = self.value * other
            name = f"{self.name}*{other}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: value * other))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def __rmul__(self, other):
        if isinstance(other, numbers.Number):
            Cls = self.__class__
            value = other * self.value
            name = f"{other}*{self.name}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: other * value))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def __truediv__(self, other):
        if isinstance(other, numbers.Number):
            Cls = self.__class__
            value = self.value / other
            name = f"{self.name}/{other}"
            new_component = Cls(name, value)
            self._refs.append((new_component, lambda value: value / other))
        else:
            raise TypeError(f"{other} not an appropriate type")
        return new_component

    def to(self, unit):
        """Convert component's value to 'unit' expression

        Args:
            unit (str): SI unit expression

        Returns:
            self
        """
        self._value = self._value.to(unit)
        return self


class Resistor(PassiveComponent):
    """Two-port linear resistor

    Args:
        name (str): name such as reference designator
        value (float, unyt_quantity): resistance in unit ohm
    """

    def __init__(self, name, value=unyt_quantity(1, "Ω")):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "Ω")
        correct_unit = (value / unyt_quantity(1, "Ω")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit ohm")
        super().__init__(name, value)
        self._laplace_admittance = 1 / self._symbol


class Capacitor(PassiveComponent):
    """Two-port linear capacitor

    Args:
        name (str): name such as reference designator
        value (unyt_quantity): capacitance in unit farad
    """

    def __init__(self, name, value=unyt_quantity(1, "F")):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "F")
        correct_unit = (value / unyt_quantity(1, "F")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit farad")
        super().__init__(name, value)
        self._laplace_admittance = self._laplace_s * self._symbol


class Inductor(PassiveComponent):
    """Two-port linear inductor

    Args:
        name (str): name such as reference designator
        value (unyt_quantity): inductance in unit henry
    """

    def __init__(self, name, value=unyt_quantity(1, "H")):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "H")
        correct_unit = (value / unyt_quantity(1, "H")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit henry")
        super().__init__(name, value)
        self._laplace_admittance = 1 / (self._laplace_s * self._symbol)


class VoltageSource(Component):
    """A ideal voltage source

    Args:
        name (str): name
        value (unyt_quantity): value in unit volt
    """

    def __init__(self, name, value=unyt_quantity(1, "V")):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "V")
        if not isinstance(value, unyt_quantity):
            raise TypeError(f"{value} must be a unyt_quantity")
        correct_unit = (value / unyt_quantity(1, "V")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit volt")
        pins = [Pin("1", 1, self), Pin("2", 2, self)]
        super().__init__(name, pins)
        self._value = value
        self._tol = 0.0
        self._tc = unyt_quantity(0.0, "1/K")

    @property
    def value(self):
        """Return value of component"""
        return self._value

    @value.setter
    def value(self, value):
        correct_unit = not issubclass(
            type(value / unyt_quantity(1, "V")), unyt_quantity
        )
        if not correct_unit:
            raise ValueError(f"{value} must be in unit volt")
        self._value = value

    @property
    def tol(self):
        """value (float): tolerance"""
        return self._tol

    @tol.setter
    def tol(self, value):
        self._tol = value

    @property
    def tc(self):
        """value (unyt_quantity or float): temperature coefficient, drift per kelvin"""
        return self._tc

    @tc.setter
    def tc(self, value):
        self._tc = unyt_quantity(value, "1/K")

    def __repr__(self):
        return f"<VoltageSource:{self._name},{self._value}>"


class Opamp(Component):
    """Opamp

    Pins
    ----
    1 'IN+' positive input
    2 'IN-' negative input
    3 'OUT' output

    Parameters
    ----------
    name : str
    aol : sympy expression, open-loop transfer function Aol(s)
    """

    def __init__(self, name, aol, **kwargs):
        pins = [
            Pin("IN+", 1, self, direction=PortDirection.IN),
            Pin("IN-", 2, self, direction=PortDirection.IN),
            Pin("OUT", 3, self, direction=PortDirection.OUT),
        ]
        super().__init__(name, pins, **kwargs)
        self.aol = sympify(aol)
