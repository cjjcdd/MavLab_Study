"""Vessel dynamics and simulation utilities.

This module defines the :class:`Vessel` class used in the DC372
manoeuvring simulator.  The vessel is modelled using a linearised
3‑DOF sway–yaw subsystem with a constant surge speed.  Hydrodynamic
coefficients and physical parameters are loaded from YAML files via
the :mod:`read_input` module and dimensionalised according to the
scaling described in the lecture notes and Fossen (2011).

The state vector used throughout the simulator has dimension 13 and
follows the ordering ``[x, y, z, φ, θ, ψ, u, v, w, p, q, r, δ]``.
Only the ``x``, ``y``, ``ψ``, ``u``, ``v``, ``r`` and ``δ``
components are actively simulated in this tutorial; the remaining
components are carried along for completeness and for future
extensions to 6‑DOF motion.
"""

from __future__ import annotations

import numpy as np
from typing import Dict, Any, Tuple, Optional
import sys

sys.path.append('/home/jj/Desktop/MavLab_Study/ros2_ws/')

import src.student.tut_03.module_control


class Vessel:
    """Representation of a planar vessel for manoeuvring simulations."""

    def __init__(self, vessel_params: Dict[str, Any], hyd_params: Dict[str, Any], control_params: Dict[str, Any]) -> None:
        """Construct a new vessel instance.

        Parameters
        ----------
        vessel_params: dict
            Physical parameters of the vessel (mass, inertia, etc.) and
            initial conditions.  See ``input.yml`` for the expected
            structure.
        hyd_params: dict
            Non‑dimensional hydrodynamic derivatives.  See
            ``hyd.yml`` for the expected keys.
        control_params: dict
            Configuration of the rudder control law.  It must contain
            the keys ``type`` (``'fixed'`` or ``'switching'``),
            ``amplitude`` and, for switching control, ``period``.
        """
        # Store raw parameters
        self.params = vessel_params
        self.hyd_params = hyd_params
        self.control_params = control_params

        # Dimensionalise the hydrodynamic derivatives
        self._dimensionalize_coefficients()

        # Generate mass matrix for sway–yaw subsystem
        self._generate_mass_matrix()

        # Initialise the control module
        src.student.tut_03.module_control.initialize(self.control_params)
        ctrl_type = str(self.control_params.get('type', 'fixed')).lower()
        if ctrl_type == 'fixed':
            self.control_func = src.student.tut_03.module_control.fixed_rudder
        elif ctrl_type == 'switching':
            self.control_func = src.student.tut_03.module_control.switching_rudder
        else:
            raise ValueError(f"Unknown control type '{ctrl_type}'")

        # Build the initial state vector (13×1)
        init = self.params.get('initial', {})
        self.initial_state = np.zeros(13, dtype=float)
        # positions
        self.initial_state[0] = float(init.get('x', 0.0))
        self.initial_state[1] = float(init.get('y', 0.0))
        self.initial_state[2] = float(init.get('z', 0.0))
        # Euler angles
        self.initial_state[3] = float(init.get('phi', 0.0))
        self.initial_state[4] = float(init.get('theta', 0.0))
        self.initial_state[5] = float(init.get('psi', 0.0))
        # velocities
        self.initial_state[6] = float(init.get('u', self.params.get('U', 0.0)))
        self.initial_state[7] = float(init.get('v', 0.0))
        self.initial_state[8] = float(init.get('w', 0.0))
        self.initial_state[9] = float(init.get('p', 0.0))
        self.initial_state[10] = float(init.get('q', 0.0))
        self.initial_state[11] = float(init.get('r', 0.0))
        # rudder
        self.initial_state[12] = float(init.get('delta', 0.0))

        # History will be populated after simulation
        self.history: Optional[Dict[str, Any]] = None

    # ------------------------------------------------------------------
    # Helper methods

    def _dimensionalize_coefficients(self) -> None:
        """Convert non‑dimensional hydrodynamic derivatives to dimensional form.

        The conversions implemented here are based on the definitions
        provided in the lecture notes and common practice for
        manoeuvring models.  Force derivatives scale with dynamic
        pressure and the appropriate power of the characteristic
        length, whilst moment derivatives pick up an additional
        length factor.  Added mass derivatives scale with the vessel
        mass and length.
        """
        # Retrieve basic properties
        rho: float = float(self.params.get('rho', 1025.0))
        L: float = float(self.params.get('L', 1.0))
        U: float = float(self.params.get('U', 1.0))
        m: float = float(self.params.get('mass', 1.0))

        # Dynamic pressure
        q = 0.5 * rho * U * U

        # Force derivatives (sway) and moment derivatives (yaw)
        # The scaling formulas follow the assumptions detailed in the
        # accompanying lecture notes: the hydrodynamic force due to
        # lateral velocity scales as q * L² * (v/U), hence the
        # derivative has units of [N/(m/s)] = [kg/s].
        self.Yv = float(self.hyd_params.get('Yv', 0.0)) * q * L * L / max(U, 1e-8)
        self.Yr = float(self.hyd_params.get('Yr', 0.0)) * q * L * L * L / max(U, 1e-8)
        self.Nv = float(self.hyd_params.get('Nv', 0.0)) * q * L * L * L / max(U, 1e-8)
        self.Nr = float(self.hyd_params.get('Nr', 0.0)) * q * L * L * L * L / max(U, 1e-8)

        # Rudder derivatives: these depend directly on the deflection
        # angle and therefore do not divide by U.  The force derivative
        # scales with q * L² and the moment derivative with q * L³.
        self.Y_delta = float(self.hyd_params.get('Y_delta', 0.0)) * q * L * L
        self.N_delta = float(self.hyd_params.get('N_delta', 0.0)) * q * L * L * L

        # Added mass derivatives: non‑dimensional coefficients are
        # multiplied by mass and appropriate powers of length.  See
        # e.g. Fossen (2011, eqs. 7.35–7.37) for a derivation.
        self.Xu_dot = float(self.hyd_params.get('Xu_dot', 0.0)) * m
        self.Yv_dot = float(self.hyd_params.get('Yv_dot', 0.0)) * m
        self.Yr_dot = float(self.hyd_params.get('Yr_dot', 0.0)) * m * L
        self.Nv_dot = float(self.hyd_params.get('Nv_dot', 0.0)) * m * L
        self.Nr_dot = float(self.hyd_params.get('Nr_dot', 0.0)) * m * L * L

    def _generate_mass_matrix(self) -> None:
        """Generate the rigid‑body plus added mass matrix for sway–yaw.

        The mass matrix for the 3‑DOF planar motion considered here is

            M = M_RB + M_A

        where the rigid‑body term ``M_RB`` is computed about the
        origin of the body‑fixed frame (typically at the midship) and
        accounts for an off‑axis centre of gravity ``x_g``.  The added
        mass matrix ``M_A`` is diagonal or block diagonal depending on
        the available hydrodynamic derivatives.
        """
        m: float = float(self.params.get('mass', 1.0))
        Iz: float = float(self.params.get('Iz', 1.0))
        x_g: float = float(self.params.get('x_g', 0.0))

        # Rigid‑body mass matrix in the surge–sway–yaw coordinates.  For
        # simplicity of the linear planar model, the surge–sway
        # coupling is neglected and the cross‑term m*x_g appears in
        # sway–yaw entries.
        M_RB = np.array([
            [m,      0.0,      0.0],
            [0.0,    m,     m * x_g],
            [0.0, m * x_g,  Iz  ]
        ], dtype=float)

        # Added mass matrix using dimensional derivatives.  Only
        # sway–yaw entries are used in this tutorial; surge added
        # mass is included for completeness.
        M_A = -np.array([
            [self.Xu_dot, 0.0,        0.0       ],
            [0.0,         self.Yv_dot, self.Yr_dot],
            [0.0,         self.Nv_dot, self.Nr_dot]
        ], dtype=float)

        # Store the total mass matrix
        self.M = M_RB + M_A

    # ------------------------------------------------------------------
    # Dynamic model

    def vessel_ode(self, t: float, state: np.ndarray) -> np.ndarray:
        """Compute the time derivative of the state vector.

        This method implements a simple planar manoeuvring model with
        constant surge speed.  Only the 3‑DOF kinematics (x, y, ψ) and
        the sway–yaw dynamics are integrated; heave, roll and pitch
        remain fixed at their initial values.  The rudder angle is
        assumed to follow a first‑order lag towards the commanded
        deflection.

        Parameters
        ----------
        t: float
            Current time [s].
        state: np.ndarray
            Current state vector (13×1).

        Returns
        -------
        np.ndarray
            Time derivative of the state vector (13×1).
        """
        # Ensure state has correct length
        y = np.asarray(state, dtype=float).flatten()
        if y.size != 13:
            raise ValueError("State vector must have length 13")

        # Extract positions and orientations
        x, y_pos, z_pos = y[0], y[1], y[2]
        phi, theta, psi = y[3], y[4], y[5]
        # Velocities
        u, v, w = y[6], y[7], y[8]
        p, q, r = y[9], y[10], y[11]
        delta = y[12]

        # Constant surge assumption: u is fixed and equal to the
        # commanded surge speed in the input file.  However, the state
        # carries u so that the full 6‑DOF vector has the correct
        # length; u_dot = 0 in this simplified model.

        # Kinematics: planar motion in the horizontal plane
        x_dot = u * np.cos(psi) - v * np.sin(psi)
        y_dot = u * np.sin(psi) + v * np.cos(psi)
        z_dot = 0.0

        # Euler angle rates: only yaw evolves, roll/pitch fixed
        phi_dot = 0.0
        theta_dot = 0.0
        psi_dot = r

        # Surge derivative (no surge dynamics modelled here)
        u_dot = 0.0

        # Heave, roll, pitch velocities derivatives are zero
        v_dot = 0.0  # placeholder; overwritten below
        w_dot = 0.0
        p_dot = 0.0
        q_dot = 0.0
        r_dot = 0.0  # placeholder; overwritten below

        # Compute rudder command using control law
        delta_cmd = float(self.control_func(t, state))

        # First‑order rudder actuator dynamics: the actual rudder
        # deflection delta approaches the commanded value with a time
        # constant tau (settable via control_params, default 0.5 s)
        tau = float(self.control_params.get('tau', 0.5))
        delta_dot = (delta_cmd - delta) / max(tau, 1e-3)

        # Compute hydrodynamic forces and moments (sway and yaw)
        Y = self.Yv * v + self.Yr * r + self.Y_delta * delta
        N = self.Nv * v + self.Nr * r + self.N_delta * delta

        # Solve for sway–yaw accelerations: M_sub * [v_dot, r_dot]^T = [Y, N]^T
        # Extract the 2×2 submatrix corresponding to v/r dynamics
        M_sub = self.M[1:, 1:]
        forces = np.array([Y, N], dtype=float)
        # Solve linear system; note that M_sub should be nonsingular for
        # realistic parameters
        vr_dot = np.linalg.solve(M_sub, forces)
        v_dot, r_dot = vr_dot

        # Assemble the derivative vector
        ydot = np.zeros_like(y)
        ydot[0] = x_dot
        ydot[1] = y_dot
        ydot[2] = z_dot
        ydot[3] = phi_dot
        ydot[4] = theta_dot
        ydot[5] = psi_dot
        ydot[6] = u_dot
        ydot[7] = v_dot
        ydot[8] = w_dot
        ydot[9] = p_dot
        ydot[10] = q_dot
        ydot[11] = r_dot
        ydot[12] = delta_dot

        return ydot