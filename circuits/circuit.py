"""Circuit

Circuit is a container of Components defining the connections
- a connection between two Ports instantiates a Node
- a connection to a Port already in the Circuit adds to that Node
- As a container, Circuit is a mapping between a Component and a Node
"""
import numpy as np
from sympy import Matrix, linsolve, lambdify
from circuits.common import PortDirection
from circuits.components import Port, Pin, PowerTap, PassiveComponent, VoltageSource


class Circuit:
    """Circuit"""

    def __init__(self, name=""):
        self._name = name
        self._components = set()
        self._ports = set()
        self._nodes = {}
        # node 0 is GND
        self._nodecnt = 1

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

    def __repr__(self):
        return f"<Circuit: {self._name}>"

    def connect(self, *ports, node_number=None):
        """Make a connection between two or more ports

        Args:
            ports (Port or Pin): two or more ports or pins
            node_number (int): optional node number assignment
        """
        if node_number is None:
            node_number = self._nodecnt
            self._nodecnt += 1
            while self._nodecnt in self._nodes.keys():
                self._nodecnt += 1
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
        cmps = [
            p.owner for p in filter(lambda c: isinstance(c, Pin), self._nodes[node])
        ]
        passives = set()
        for cmp in cmps:
            if isinstance(cmp, PassiveComponent):
                passives.add(cmp)
        return passives

    def transfer_function(self, input_port="in", output_port="out", reference="gnd"):
        """Find the transfer function from input to output with respect to reference

        For now, there is only one input and one output allowed and only passives.

        Args:
            input_port (Port): input signal
            output_port (Port): output signal
            reference (Port): signal reference

        Returns:
            (Sympy expression)
        """
        system_nodes = set(self._nodes.keys())
        input_node = None
        output_node = None
        gnd_nodes = set()
        for node, connections in self._nodes.items():
            if Port(input_port, PortDirection.IN) in connections:
                input_node = node
            if Port(output_port, PortDirection.OUT) in connections:
                output_node = node
            if PowerTap(reference) in connections:
                gnd_nodes.add(node)
        if input_node is None or output_node is None or len(gnd_nodes) == 0:
            raise ValueError("Missing input or output Port or gnd reference")
        system_nodes.remove(input_node)
        system_nodes.difference_update(gnd_nodes)
        admittance_matrix = []
        for row in system_nodes:
            admittance_row = []
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

    def dc_nom(self, input_port="in", output_port="out", reference="gnd"):
        """Compute the nominal DC value from input to output with respect to reference

        Args:
            input_port (Port): input signal
            output_port (Port): output signal
            reference (Port): signal reference

        Returns:
            (Quantity)
        """
        tf_expr = self.transfer_function(input_port, output_port, reference)
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
        values = [cmps[str(name)].value for name in syms]
        for node, connections in self._nodes.items():
            if Port(input_port, PortDirection.IN) in connections:
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

    def dc_max(self, input_port="in", output_port="out", reference="gnd"):
        """Compute the maximum DC value from input to output with respect to reference

        Parameters
        ----------
        input_port : Port, input signal
        output_port : Port, output signal
        reference : Port, signal reference

        Returns
        -------
        dc_max : unyt_quantity
        """
        tf_expr = self.transfer_function(input_port, output_port, reference)
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
        values = [cmps[str(name)].value for name in syms]
        tols = []
        for sym in syms:
            tf_sens = lambdify(syms, tf_expr.diff(sym))
            sensitivity = tf_sens(*values)
            tol = cmps[str(sym)].tol
            if np.all(sensitivity > 0):
                if isinstance(tol, tuple):
                    tol = tol[1]
            else:
                if isinstance(tol, tuple):
                    tol = tol[0]
                else:
                    tol = -tol
            tols.append(tol)
        tf = lambdify(syms, tf_expr)
        values = [v * (1 + t) for v, t in zip(values, tols)]
        for node, connections in self._nodes.items():
            if Port(input_port, PortDirection.IN) in connections:
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
        return tf(*values) * source.value * (1 + src_tol)

    def dc_min(self, input_port="in", output_port="out", reference="gnd"):
        """Compute the maximum DC value from input to output with respect to reference

        Parameters
        ----------
        input_port : Port, input signal
        output_port : Port, output signal
        reference : Port, signal reference

        Returns
        -------
        dc_min : unyt_quantity
        """
        tf_expr = self.transfer_function(input_port, output_port, reference)
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
        values = [cmps[str(name)].value for name in syms]
        tols = []
        for sym in syms:
            tf_sens = lambdify(syms, tf_expr.diff(sym))
            sensitivity = tf_sens(*values)
            tol = cmps[str(sym)].tol
            if np.all(sensitivity > 0):
                if isinstance(tol, tuple):
                    tol = tol[0]
                else:
                    tol = -tol
            else:
                if isinstance(tol, tuple):
                    tol = tol[1]
            tols.append(tol)
        tf = lambdify(syms, tf_expr)
        values = [v * (1 + t) for v, t in zip(values, tols)]
        for node, connections in self._nodes.items():
            if Port(input_port, PortDirection.IN) in connections:
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
        return tf(*values) * source.value * (1 + src_tol)

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
