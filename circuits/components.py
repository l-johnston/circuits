"""Definition of component types"""
import numbers
from sympy import Symbol, sympify
from unyt import unyt_quantity, degC, delta_degC, V
from circuits.common import PortDirection, temperature_difference


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
        self._parasitic = False
        self.parasitics = {}
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

    @property
    def parasitic(self):
        """Whether a component is parasitic

        Parameters
        ----------
        value : bool
        """
        return self._parasitic

    @parasitic.setter
    def parasitic(self, value):
        self._parasitic = bool(value)

    @property
    def has_parasitics(self):
        """Whether this component has internally defined parasitics"""
        return bool(len(self.parasitics))


class PassiveComponent(Component):
    """Class for passive, two-port resistors, capacitors and inductors

    Parameters
    ----------
    name : str
        name of passive component
    value : float or unyt_quantity
        nominal value
    """

    def __init__(self, name, value, **kwargs):
        try:
            pin_names = kwargs.pop("pin_names")
        except KeyError:
            pin_names = ["1", "2"]
        pins = [Pin(name, int(name), self) for name in pin_names]
        self._value = value
        try:
            tol = kwargs.pop("tol")
        except KeyError:
            tol = 0.0
        self._tol = tol
        try:
            tc = kwargs.pop("tc")
        except KeyError:
            tc = 0.0 / delta_degC
        super().__init__(name, pins, **kwargs)
        self._tc = unyt_quantity(tc, "1/K")
        self._ref_temp = 20 * degC
        self._refs = []
        self._laplace_s = Symbol("s")
        self._laplace_admittance = None

    @property
    def value(self):
        """Return value of component"""
        return self._value

    @value.setter
    def value(self, value):
        ratio = self._value / value
        if not ratio.units.is_dimensionless:
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
        self._tc = unyt_quantity(value, "1/delta_degC")

    @property
    def reference_temperature(self):
        """value : unyt_quantity
        reference temperature for drift calculation
        """
        return self._ref_temp

    @reference_temperature.setter
    def reference_temperature(self, value):
        self._ref_temp = unyt_quantity(value, "degC")

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

    def max(self, temperature=None):
        """Calculate the maximum component value at the given temperature

        Parameters
        ----------
        temperature : unyt_quantity in degree Celcius
            component temperature for drift from reference temperature
            default of None means to only consider tolerance
        """
        if temperature is None:
            temperature = self._ref_temp
        deltaT = abs(temperature_difference(self._ref_temp, temperature))
        return self.value * (1 + self._tol + deltaT * self._tc)

    def min(self, temperature=None):
        """Calculate the minimum component value at the given temperature

        Parameters
        ----------
        temperature : unyt_quantity in degree Celcius
            component temperature for drift from reference temperature
            default of None means to only consider tolerance
        """
        if temperature is None:
            temperature = self._ref_temp
        deltaT = abs(temperature_difference(self._ref_temp, temperature))
        return self.value * (1 - (self._tol + deltaT * self._tc))


class Resistor(PassiveComponent):
    """Two-port linear resistor

    Parameters
    ----------
    name : str
        name such as reference designator
    value : float or unyt_quantity
        resistance in unit ohm
    """

    def __init__(self, name, value=unyt_quantity(1, "Ω"), **kwargs):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "Ω")
        correct_unit = (value / unyt_quantity(1, "Ω")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit ohm")
        super().__init__(name, value, **kwargs)
        self._laplace_admittance = 1 / self._symbol

    def parallel(self, other):
        """Compute the parallel resistance with `other`

        Parameters
        ----------
        other : Resistor
        """
        if not isinstance(other, Resistor):
            raise TypeError(f"'{other}' is not a Resistor")
        r1 = self.value
        r2 = other.value
        name = f"{self.name}||{other.name}"
        return Resistor(name, (r1 * r2) / (r1 + r2))


class Capacitor(PassiveComponent):
    """Two-port linear capacitor

    Parameters
    ----------
    name :str
        name such as reference designator
    value : float or unyt_quantity
        capacitance in unit farad
    """

    def __init__(self, name, value=unyt_quantity(1, "F"), **kwargs):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "F")
        correct_unit = (value / unyt_quantity(1, "F")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit farad")
        super().__init__(name, value, **kwargs)
        self._laplace_admittance = self._laplace_s * self._symbol

    def series(self, other):
        """Compute the series capacitance with `other`

        Parameters
        ----------
        other : Capacitor
        """
        if not isinstance(other, Capacitor):
            raise TypeError(f"'{other}' is not a Capacitor")
        c1 = self.value
        c2 = other.value
        name = f"{self.name}--{other.name}"
        return Capacitor(name, (c1 * c2) / (c1 + c2))


class Inductor(PassiveComponent):
    """Two-port linear inductor

    Parameters
    ----------
    name :str
        name such as reference designator
    value : float or unyt_quantity
        inductance in unit henry
    """

    def __init__(self, name, value=unyt_quantity(1, "H"), **kwargs):
        if isinstance(value, numbers.Number):
            value = unyt_quantity(value, "H")
        correct_unit = (value / unyt_quantity(1, "H")).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"{value} must be in unit henry")
        super().__init__(name, value, **kwargs)
        self._laplace_admittance = 1 / (self._laplace_s * self._symbol)

    def parallel(self, other):
        """Compute the parallel inductance with `other`

        Parameters
        ----------
        other : Inductor
        """
        if not isinstance(other, Inductor):
            raise TypeError(f"'{other}' is not an Inductor")
        l1 = self.value
        l2 = other.value
        name = f"{self.name}||{other.name}"
        return Inductor(name, (l1 * l2) / (l1 + l2))


class VoltageSource(Component):
    """A ideal voltage source

    Parameters
    ----------
    name : str
        name such as reference designator
    value : float or unyt_quantity
        value in unit volt
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
        ratio = value / unyt_quantity(1, "V")
        if not ratio.units.is_dimensionless:
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
        self._tc = unyt_quantity(value, "1/delta_degC")

    def __repr__(self):
        return f"<VoltageSource:{self._name},{self._value}>"


class Opamp(Component):
    """Opamp

    Parameters
    ----------
    name : str
    Aol : sympy expression, open-loop transfer function Aol(s)

    Pins
    ----
    1 'IN+' positive input
    2 'IN-' negative input
    3 'OUT' output
    """

    def __init__(self, name, Aol, **kwargs):
        pins = [
            Pin("IN+", 1, self, direction=PortDirection.IN),
            Pin("IN-", 2, self, direction=PortDirection.IN),
            Pin("OUT", 3, self, direction=PortDirection.OUT),
        ]
        super().__init__(name, pins, **kwargs)
        self.Aol = sympify(Aol)
        if hasattr(self, "Vos"):
            self.Vos = unyt_quantity(self.Vos, V)
            vos = VoltageSource(f"{name}_Vos", value=self.Vos)
            self.parasitics[vos] = [None, self.pin(2)]


class PassiveComponentNetwork(Component):
    """Passive component network

    An `n` element array of passive components such as a resistor network.

    Parameters
    ----------
    name : str
        name of passive component such as the reference designator
    values : list of float or unyt_quantity of length n
        nominal values

    Keyword Attributes
    -------------------
    tol : float
        absolute tolerance
    tc : float or unyt_quantity
        absolute temperature drift per Kelvin
    rel_tol : float
        relative tolerance
    rel_tc : float or unyt_quantity
        relative temperature drift per Kelvin
    reference_temperature : unyt_quantity in unit degree Celsius

    Pins - follows Vishay nomenclature
    ----
    1 <element #1> 2*n
    2 <element #2> 2*n-1
    ...
    n <element #n> n+1
    """

    def __init__(self, name, values, **kwargs):
        self._n = len(values)
        pins = [Pin(f"{i}", i, self) for i in range(1, 2 * self._n + 1)]
        super().__init__(name, pins)
        self._values = values
        self._tol = kwargs.get("tol", 0.0)
        self._tc = kwargs.get("tc", 0.0 / delta_degC)
        self._rel_tol = kwargs.get("rel_tol", 0.0)
        self._rel_tc = kwargs.get("rel_tc", 0.0 / delta_degC)
        self._ref_temp = kwargs.get("reference_temperature", 20 * degC)
        self._elements = []
        self._symbols = []

    @property
    def values(self):
        """Return value of component"""
        return self._values

    @values.setter
    def values(self, values):
        correct_unit = (self._values[0] / values[0]).units.is_dimensionless
        if not correct_unit:
            raise ValueError(f"'{values[0]}' must be in unit '{self._values[0].units}'")
        self._values = values

    @property
    def tol(self):
        """value : float
        absolute tolerance"""
        return self._tol

    @tol.setter
    def tol(self, value):
        self._tol = value
        self._elements[0].tol = value

    @property
    def tc(self):
        """value : unyt_quantity or float
        absolute temperature coefficient as drift per kelvin"""
        return self._tc

    @tc.setter
    def tc(self, value):
        tc = unyt_quantity(value, "1/delta_degC")
        self._tc = tc
        self._elements[0].tc = tc

    @property
    def rel_tol(self):
        """value : float
        relative tolerance
        """
        return self._rel_tol

    @rel_tol.setter
    def rel_tol(self, value):
        self._rel_tol = value
        for element in self._elements[1:]:
            element.tol = value

    @property
    def rel_tc(self):
        """value : float or unyt_quantity
        relative temperature coefficient
        """
        return self._rel_tc

    @rel_tc.setter
    def rel_tc(self, value):
        rel_tc = unyt_quantity(value, "1/delta_degC")
        self._tc = rel_tc
        for element in self._elements[1:]:
            element.tc = rel_tc

    @property
    def reference_temperature(self):
        """value : unyt_quantity
        reference temperature for temperature drift calculations, default 20 °C
        """
        return self._ref_temp

    @reference_temperature.setter
    def reference_temperature(self, value):
        ref_temp = unyt_quantity(value, "degC")
        self._ref_temp = ref_temp
        for element in self._elements:
            element.reference_temperature = ref_temp

    def element_at(self, pin):
        """Retrieve the element at `pin` number

        Parameters
        ----------
        pin : int

        Returns
        -------
        element : PassiveComponent
        """
        i = pin if pin <= self._n else 2 * self._n - pin + 1
        return self._elements[i - 1]

    def __getitem__(self, item):
        try:
            return self._elements[item]
        except TypeError:
            i = int(str(item)[1:]) - 1
            return self._elements[i]


class ResistorNetwork(PassiveComponentNetwork):
    """Resistor network

    Resistor network consisting of `n` individual elements.

    Parameters
    ----------
    name : str
        name such as reference designator (e.g. 'RN1')
    values : list of float or unyt_quantity of length n
        resistances in unit ohm

    Pins - follows Vishay nomenclature
    ----
    1 -R1- 2*n
    2 -R2- 2*n-1
    ...
    n -Rn- n+1
    """

    def __init__(self, name, values, **kwargs):
        super().__init__(name, values, **kwargs)
        n = len(values)
        for i, value in enumerate(values, start=1):
            tol, tc = (self._tol, self._tc) if i == 1 else (self._rel_tol, self._rel_tc)
            res = Resistor(
                f"{name}_R{i}",
                value,
                tol=tol,
                tc=tc,
                pin_names=[f"{i}", f"{2*n-i+1}"],
            )
            self._elements.append(res)
            setattr(self, f"R{i}", res)
        self._symbols = [f"{name}_R{i}" for i in range(1, len(values) + 1)]
