"""Circuit

Circuit is a container of Components defining the connections
- a connection between two Ports instantiates a Node
- a connection to a Port already in the Circuit adds to that Node
- As a container, Circuit is a mapping between a Component and a Node
"""
import numpy as np
from sympy import Matrix, linsolve, lambdify, sympify, Symbol
from unyt import Hz, dB
from circuits.common import PortDirection, j, π, temperature_difference
from circuits.components import (
    Port,
    Pin,
    PowerTap,
    PassiveComponent,
    VoltageSource,
    Opamp,
    PassiveComponentNetwork,
)


class Circuit:
    """Circuit"""

    def __init__(self, name=""):
        self._name = name
        self._components = set()
        self._parasitic_components = set()
        self._ports = set()
        self._nodes = {}
        self._parasitic_nodes = {}
        # node 0 is GND
        self._nodecnt = 1
        self._include_parasitics = True

    @property
    def name(self):
        """name (str): circuit name"""
        return self._name

    @name.setter
    def name(self, name):
        self._name = str(name)

    @property
    def components(self):
        """components (set): set of Components in circuit"""
        return self._components

    @property
    def ports(self):
        """ports (list): list of Ports in circuit"""
        return self._ports

    @property
    def nodes(self):
        """nodes (dict): node dict"""
        return self._nodes

    @property
    def include_parasitics(self):
        """Whether to include parasitics

        Parameters
        ----------
        value : bool
        """
        return self._include_parasitics

    @include_parasitics.setter
    def include_parasitics(self, value):
        self._include_parasitics = value
        if value:
            for node, pins in self._parasitic_nodes.items():
                self.connect(*pins, node_number=node)
        else:
            for node in self._nodes:
                initial_pins = self._nodes[node].copy()
                remaining_pin = None
                for pin in initial_pins:
                    if not pin.owner.parasitic:
                        remaining_pin = pin
                        break
                for pin in initial_pins:
                    if pin.owner.parasitic:
                        self.disconnect(pin, remaining_pin)

    def __repr__(self):
        return f"<Circuit: {self._name}>"

    def connect(self, *ports, node_number=None):
        """Make a connection between two or more ports

        Args:
            ports (Port or Pin): two or more ports or pins
            node_number (int): optional node number assignment
        """
        if node_number is None:
            for node, connections in self.nodes.items():
                if set(ports).intersection(connections):
                    break
            else:
                node = self._nodecnt
                self._nodecnt += 1
                while self._nodecnt in self._nodes.keys():
                    self._nodecnt += 1
            node_number = node
        for port in ports:
            if isinstance(port, Port):
                self._ports.add(port)
            elif isinstance(port, Pin):
                self._components.add(port.owner)
            else:
                raise ValueError(f"'{port}' must be Port or Pin")
            try:
                self._nodes[node_number].add(port)
            except KeyError:
                self._nodes[node_number] = {port}

    def connect_parasitic(self, pin, node):
        """Connect a parasitic component pin to a node"""
        self._parasitic_components.add(pin.owner)
        try:
            self._parasitic_nodes[node].add(pin)
        except KeyError:
            self._parasitic_nodes[node] = {pin}
        if self.include_parasitics:
            self.connect(pin, node_number=node)

    def _checkshortednodes(self):
        """Check for short between two nodes

        Two nodes are shorted if their connection sets have a non-empty intersection
        Remove duplicate node by taking union
        """
        nodes = self._nodes.copy()
        cmp_nodes = self._nodes.copy()
        for node, connections in nodes.items():
            cmp_nodes.pop(node)
            for cmp_node, cmp_connections in cmp_nodes.items():
                if connections.intersection(cmp_connections):
                    self._nodes[node] = connections.union(cmp_connections)
                    self._nodes.pop(cmp_node)

    def disconnect(self, port1, port2):
        """Disconnect port1 from port2 while leaving port2 on the node

        Args:
            port1 (Port or Pin): port or pin
            port2 (Port or Pin): port or pin
        """
        for node, connections in self._nodes.items():
            if {port1, port2}.issubset(connections):
                self._nodes[node].remove(port1)
                break
        self._trimorphanednodes()
        self._trimorphanedports()
        self._trimorphanedcomponents()

    def _trimorphanednodes(self):
        """Trim orphaned nodes"""
        nodes = {}
        for node, connections in self._nodes.items():
            if len(connections) > 1:
                nodes[node] = connections
        self._nodes = nodes

    def _trimorphanedports(self):
        """Trim orphaned ports"""
        conn_ports = set()
        for connections in self._nodes.values():
            for port in filter(lambda p: isinstance(p, Port), connections):
                conn_ports.add(port)
        self._ports = conn_ports

    def _trimorphanedcomponents(self):
        """Trim orphaned components"""
        conn_components = set()
        for connections in self._nodes.values():
            for port in filter(lambda p: isinstance(p, Pin), connections):
                conn_components.add(port.owner)
        self._components = conn_components

    def remove(self, item):
        """Remove a Component or Port"""
        if isinstance(item, Port):
            item = set(item)
        else:
            item = set(item.pins.values())
        for node in self._nodes:
            self._nodes[node].difference_update(item)
        self._trimorphanednodes()
        self._trimorphanedports()
        self._trimorphanedcomponents()

    def check_opens(self):
        """Check for disconnected Pins"""
        all_connections = set()
        for connections in self._nodes.values():
            for connection in connections:
                if isinstance(connection, Pin):
                    all_connections.add(connection)
        all_pins = set()
        for component in self._components:
            for pin in component.pins.values():
                all_pins.add(pin)
        return all_pins.difference(all_connections)

    def clear(self):
        """Reset circuit to the initial empty state"""
        self.__init__(self._name)

    def _get_node_passives(self, node):
        """Get the passive components at a given node"""
        passives = set()
        for obj in self._nodes[node]:
            if not isinstance(obj, Pin):
                continue
            pin = obj
            cmp = pin.owner
            if isinstance(cmp, PassiveComponent):
                passives.add(cmp)
            elif isinstance(cmp, PassiveComponentNetwork):
                passives.add(cmp.element_at(pin.number))
            else:
                pass
        return passives

    def _pin2node(self, pin):
        lu = {}
        for node, pins in self.nodes.items():
            for p in pins:
                lu[p] = node
        return lu[pin]

    def _isopampdriven(self, node):
        # Determine if node is driven by an opamp
        # Assumes only one opamp output connected to node
        for pin in self.nodes[node]:
            cmp = getattr(pin, "owner", None)
            if isinstance(cmp, Opamp) and pin.name == "OUT":
                break
        else:
            return False
        return True

    def _opampadmittancerow(self, node, system_nodes):
        # Form admittances for an opamp driven node
        for pin in self.nodes[node]:
            cmp = getattr(pin, "owner", None)
            if isinstance(cmp, Opamp) and pin.name == "OUT":
                break
        else:
            return False
        inp = self._pin2node(cmp.pin("IN+"))
        inn = self._pin2node(cmp.pin("IN-"))
        Aol = cmp.Aol
        admittance_row = []
        for col in system_nodes:
            if col == inp:
                admittance_row.append(-Aol)
            elif col == inn:
                admittance_row.append(Aol)
            elif col == node:
                admittance_row.append(sympify("1"))
            else:
                admittance_row.append(sympify("0"))
        return admittance_row

    def transfer_function(self, in_node="in", out_node="out", reference="gnd"):
        """Find the transfer function from input to output with respect to reference

        For now, there is only one input and one output allowed.

        Parameters
        ----------
            in_node : node or port name
                input signal
            out_node : node or port name
                output signal
            reference : node or port name
                signal reference

        Returns
        -------
            (Sympy expression)
        """
        system_nodes = set(self._nodes.keys())
        in_node = str(in_node)
        out_node = str(out_node)
        reference = str(reference)
        input_node = in_node if in_node in system_nodes else None
        output_node = out_node if out_node in system_nodes else None
        gnd_nodes = reference if reference in system_nodes else set()
        for node, connections in self._nodes.items():
            if not input_node and Port(in_node, PortDirection.IN) in connections:
                input_node = node
            if not output_node and Port(out_node, PortDirection.OUT) in connections:
                output_node = node
            if PowerTap(reference) in connections:
                gnd_nodes.add(node)
        gnd_nodes = (
            set("0") if "0" in system_nodes and gnd_nodes == set() else gnd_nodes
        )
        gnd_nodes = set(0) if 0 in system_nodes and gnd_nodes == set() else gnd_nodes
        if input_node is None or output_node is None or len(gnd_nodes) == 0:
            raise ValueError("Missing input or output Port or gnd reference")
        system_nodes.remove(input_node)
        system_nodes.difference_update(gnd_nodes)
        admittance_matrix = []
        for row in system_nodes:
            admittance_row = []
            if self._isopampdriven(row):
                admittance_row = self._opampadmittancerow(row, system_nodes)
                admittance_matrix.append(admittance_row)
                continue
            for col in system_nodes:
                if row == col:
                    admittances = [c.admittance for c in self._get_node_passives(col)]
                    admittance_row.append(sum(admittances))
                else:
                    rcmps = self._get_node_passives(row)
                    ccmps = self._get_node_passives(col)
                    cmps = rcmps.intersection(ccmps)
                    negative_admittances = [-1 * c.admittance for c in cmps]
                    admittance_row.append(sum(negative_admittances))
            admittance_matrix.append(admittance_row)
        source_vector = []
        icmps = self._get_node_passives(input_node)
        for row in system_nodes:
            rcmps = self._get_node_passives(row)
            cmps = icmps.intersection(rcmps)
            admittances = [c.admittance for c in cmps]
            source_vector.append(sum(admittances))
        system = Matrix(admittance_matrix), Matrix(source_vector)
        solution_set = linsolve(system)
        solutions = list(solution_set)
        solution_vector = solutions[0]
        tf = solution_vector[list(system_nodes).index(output_node)]
        return tf

    def dc_nom(self, in_node="in", out_node="out", reference="gnd"):
        """Compute the nominal DC value from input to output with respect to reference

        Parameters
        ----------
            in_node : Port or node
                input signal
            out_node : Port or node
                output signal
            reference : Port or node
                signal reference

        Returns
        -------
            unyt_quantity
        """
        tf_expr = self.transfer_function(in_node, out_node, reference)
        syms = tf_expr.free_symbols
        laplace_s = None
        for sym in syms:
            if sym.name == "s":
                laplace_s = sym
        if laplace_s is not None:
            syms.remove(laplace_s)
            tf_expr = tf_expr.subs(laplace_s, 0.0)
        syms = list(syms)
        tf = lambdify(syms, tf_expr)
        cmps = {cmp.name: cmp for cmp in self._components}
        values = []
        for name in syms:
            name = str(name)
            try:
                values.append(cmps[name].value)
            except KeyError:
                pn_name, name = name.split("_")
                values.append(cmps[pn_name][name].value)
        system_nodes = set(self._nodes.keys())
        in_node = str(in_node)
        input_node = in_node if in_node in system_nodes else None
        for node, connections in self._nodes.items():
            if Port(in_node, PortDirection.IN) in connections:
                input_node = node
                break
        cmps = [
            p.owner
            for p in filter(lambda c: isinstance(c, Pin), self._nodes[input_node])
        ]
        source = None
        for cmp in cmps:
            if isinstance(cmp, VoltageSource):
                source = cmp
                break
        return tf(*values) * source.value

    def dc_max(self, in_node="in", out_node="out", reference="gnd", temperature=None):
        """Compute the maximum DC value from input to output with respect to reference

        Parameters
        ----------
        in_node : Port or node
            input signal
        out_node : Portor node
            output signal
        reference : Port or node
            signal reference
        temperature : unyt_quantity in degree Celsius

        Returns
        -------
        dc_max : unyt_quantity
        """
        tf_expr = self.transfer_function(in_node, out_node, reference)
        syms = tf_expr.free_symbols
        laplace_s = None
        for sym in syms:
            if sym.name == "s":
                laplace_s = sym
        if laplace_s is not None:
            syms.remove(laplace_s)
            tf_expr = tf_expr.subs(laplace_s, 0.0)
        syms = list(syms)
        cmps = {cmp.name: cmp for cmp in self._components}
        rows = []
        for sym in syms:
            name = str(sym)
            element = None
            try:
                value = cmps[name].value
            except KeyError:
                name, element = name.split("_")
                value = cmps[name][element].value
            rows.append([cmps[name], element, value])
        for i, sym in enumerate(syms):
            tf_sens = lambdify(syms, tf_expr.diff(sym))
            values = [row[2] for row in rows]
            sensitivity = tf_sens(*values).item()
            rows[i].extend([sensitivity])
        for i in range(len(syms)):
            cmp = rows[i][0]
            sensitivity_i = rows[i][3]
            sign_i = np.sign(sensitivity_i)
            reftemp = cmp.reference_temperature
            if temperature is None:
                temperature = reftemp
            deltaT = abs(temperature_difference(reftemp, temperature))
            if isinstance(cmp, PassiveComponent):
                tol = sign_i * cmp.tol
                tc = sign_i * cmp.tc
            elif isinstance(cmp, PassiveComponentNetwork):
                element_i = rows[i][1]
                if cmp[element_i].tol == cmp.tol:
                    tol = sign_i * cmp[element_i].tol
                    tc = sign_i * cmp[element_i].tc
                else:
                    for k in range(len(syms)):
                        element_k = rows[k][1]
                        if cmp[element_k].tol == cmp.tol:
                            break
                    tol = cmp[element_k].tol
                    tc = cmp[element_k].tc
                    sensitivity_k = rows[k][3]
                    sign_k = np.sign(sensitivity_k)
                    rel_tol = cmp[element_i].tol
                    rel_tc = cmp[element_i].tc
                    median_tol = sign_k * (tol - rel_tol)
                    median_tc = sign_k * (tc - rel_tc)
                    tol = median_tol + sign_i * rel_tol
                    tc = median_tc + sign_i * rel_tc
                rows[i].extend([tol, deltaT, tc])
            else:
                pass
        tf = lambdify(syms, tf_expr)
        tols = []
        for row in rows:
            tol, deltaT, tc = row[4:]
            tols.append(tol + deltaT * tc)
        values = [row[2] for row in rows]
        params = [v * (1 + t) for v, t in zip(values, tols)]
        system_nodes = set(self._nodes.keys())
        in_node = str(in_node)
        input_node = in_node if in_node in system_nodes else None
        for node, connections in self._nodes.items():
            if Port(in_node, PortDirection.IN) in connections:
                input_node = node
                break
        cmps = [
            p.owner
            for p in filter(lambda c: isinstance(c, Pin), self._nodes[input_node])
        ]
        source = None
        for cmp in cmps:
            if isinstance(cmp, VoltageSource):
                source = cmp
                break
        if isinstance(source.tol, tuple):
            src_tol = source.tol[1]
        else:
            src_tol = source.tol
        ret_pos = tf(*params) * source.value * (1 + src_tol)
        ret_neg = tf(*params) * source.value * (1 - src_tol)
        return max(ret_pos, ret_neg)

    def dc_min(self, in_node="in", out_node="out", reference="gnd", temperature=None):
        """Compute the maximum DC value from input to output with respect to reference

        Parameters
        ----------
        in_node : Port or node
            input signal
        out_node : Port or node
            output signal
        reference : Port or node
            signal reference
        temperature : unyt_quantity in degree Celsius

        Returns
        -------
        dc_min : unyt_quantity
        """
        tf_expr = self.transfer_function(in_node, out_node, reference)
        syms = tf_expr.free_symbols
        laplace_s = None
        for sym in syms:
            if sym.name == "s":
                laplace_s = sym
        if laplace_s is not None:
            syms.remove(laplace_s)
            tf_expr = tf_expr.subs(laplace_s, 0.0)
        syms = list(syms)
        cmps = {cmp.name: cmp for cmp in self._components}
        rows = []
        for sym in syms:
            name = str(sym)
            element = None
            try:
                value = cmps[name].value
            except KeyError:
                name, element = name.split("_")
                value = cmps[name][element].value
            rows.append([cmps[name], element, value])
        for i, sym in enumerate(syms):
            tf_sens = lambdify(syms, tf_expr.diff(sym))
            values = [row[2] for row in rows]
            sensitivity = tf_sens(*values).item()
            rows[i].extend([sensitivity])
        for i in range(len(syms)):
            cmp = rows[i][0]
            sensitivity_i = rows[i][3]
            sign_i = -1 * np.sign(sensitivity_i)
            reftemp = cmp.reference_temperature
            if temperature is None:
                temperature = reftemp
            deltaT = abs(temperature_difference(reftemp, temperature))
            if isinstance(cmp, PassiveComponent):
                tol = sign_i * cmp.tol
                tc = sign_i * cmp.tc
            elif isinstance(cmp, PassiveComponentNetwork):
                element_i = rows[i][1]
                if cmp[element_i].tol == cmp.tol:
                    tol = sign_i * cmp[element_i].tol
                    tc = sign_i * cmp[element_i].tc
                else:
                    for k in range(len(syms)):
                        element_k = rows[k][1]
                        if cmp[element_k].tol == cmp.tol:
                            break
                    tol = cmp[element_k].tol
                    tc = cmp[element_k].tc
                    sensitivity_k = rows[k][3]
                    sign_k = -1 * np.sign(sensitivity_k)
                    rel_tol = cmp[element_i].tol
                    rel_tc = cmp[element_i].tc
                    median_tol = sign_k * (tol - rel_tol)
                    median_tc = sign_k * (tc - rel_tc)
                    tol = median_tol + sign_i * rel_tol
                    tc = median_tc + sign_i * rel_tc
                rows[i].extend([tol, deltaT, tc])
            else:
                pass
        tf = lambdify(syms, tf_expr)
        tols = []
        for row in rows:
            tol, deltaT, tc = row[4:]
            tols.append(tol + deltaT * tc)
        values = [row[2] for row in rows]
        params = [v * (1 + t) for v, t in zip(values, tols)]
        system_nodes = set(self._nodes.keys())
        in_node = str(in_node)
        input_node = in_node if in_node in system_nodes else None
        for node, connections in self._nodes.items():
            if Port(in_node, PortDirection.IN) in connections:
                input_node = node
                break
        cmps = [
            p.owner
            for p in filter(lambda c: isinstance(c, Pin), self._nodes[input_node])
        ]
        source = None
        for cmp in cmps:
            if isinstance(cmp, VoltageSource):
                source = cmp
                break
        if isinstance(source.tol, tuple):
            src_tol = source.tol[0]
        else:
            src_tol = -source.tol
        ret_pos = tf(*params) * source.value * (1 + src_tol)
        ret_neg = tf(*params) * source.value * (1 - src_tol)
        return min(ret_pos, ret_neg)

    # def bandwidth(self, in_node="in", out_node="out", reference="gnd", level=-3 * dB):
    #     """Compute frequencies where transfer function is `level`

    #     Parameters
    #     ----------
    #     in_node : Port or node
    #         input signal
    #     out_node : Port or node
    #         output signal
    #     reference : Port or node
    #         signal reference
    #     level : unyt_quantity
    #     """
    #     tf_expr = self.transfer_function(in_node, out_node, reference)
    #     syms = tf_expr.free_symbols
    #     laplace_s = None
    #     for sym in syms:
    #         if sym.name == "s":
    #             laplace_s = sym
    #     if laplace_s is not None:
    #         syms.remove(laplace_s)
    #         f = Symbol("f")
    #         syms.add(f)
    #         tf_expr = tf_expr.subs(laplace_s, j * 2 * π * f)
    #     subs = {}
    #     for sym in syms:
    #         if sym == f:
    #             pass
    #         else:
    #             subs[sym] = cmps[str(sym)].value
    #     tf_expr = tf_expr.subs(subs)

    def bode_plot(
        self, in_node="in", out_node="out", reference="gnd", frequencies=None
    ):
        """Generate bode plot

        Parameters
        ----------
        in_node : Port or node
            input signal
        out_node : Port or node
            output signal
        reference : Port or node
            signal reference
        frequencies : unyt_array or None
            If None, generate frequencies automatically

        Returns
        -------
        Frequency response as tuple (frequency, magnitude)
        """
        tf_expr = self.transfer_function(in_node, out_node, reference)
        syms = tf_expr.free_symbols
        laplace_s = None
        for sym in syms:
            if sym.name == "s":
                laplace_s = sym
        if laplace_s is not None:
            syms.remove(laplace_s)
            f = Symbol("f")
            syms.add(f)
            tf_expr = tf_expr.subs(laplace_s, j * 2 * π * f)
        syms = list(syms)
        tf = lambdify(syms, tf_expr)
        cmps = {cmp.name: cmp for cmp in self._components}
        freq = np.logspace(0, 5, 100) * Hz if frequencies is None else frequencies
        values = []
        for sym in syms:
            name = str(sym)
            if sym == f:
                values.append(freq)
            else:
                try:
                    values.append(cmps[name].value)
                except KeyError:
                    name, element = name.split("_")
                    values.append(cmps[name][element].value)

        return (freq, 20 * np.log10(np.abs(tf(*values).in_base())) * dB)

    def __dir__(self):
        attrs = list(filter(lambda s: not s.startswith("_"), super().__dir__()))
        refdes = [component.name for component in self.components]
        attrs += refdes
        return sorted(attrs)

    def __getattr__(self, name):
        try:
            return self.__getattribute__(name)
        except AttributeError:
            comps = {comp.name: comp for comp in self.components}
            try:
                return comps[name]
            except KeyError:
                raise AttributeError(
                    f"'{self}' object has no attribute '{name}'"
                ) from None
