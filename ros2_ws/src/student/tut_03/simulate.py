"""Main entry point for the DC372 manoeuvring simulator.

This script brings together the configuration parser, the vessel
dynamics and the plotting routines to run a complete simulation
without any external dependencies beyond NumPy, SciPy and
matplotlib.  Executing the file from the command line will read
``input.yml`` and ``hyd.yml``, integrate the equations of motion
over the specified duration and save a plot of the results to
``simulation_results.png``.

For clarity and modularity, no ROS2 functionality is included in this
module.  The subsequent tutorial introduces how to wrap the vessel
model into a ROS2 node.  Here we focus solely on the open‑loop
simulation.
"""

from __future__ import annotations

import os
import numpy as np
from scipy.integrate import solve_ivp

from read_input import read_input
from class_vessel import Vessel
from plot_results import plot_results


def simulate(input_file: str = None) -> None:
    """Run the vessel simulation based on the provided input file.

    Parameters
    ----------
    input_file: str, optional
        Path to the YAML file containing simulation parameters.  If
        omitted, the default ``input.yml`` relative to this script is
        used.
    """
    # Determine the path to the input file
    if input_file is None:
        # Default to input.yml in the same directory as this file
        here = os.path.dirname(os.path.abspath(__file__))
        input_file = os.path.join(here, 'input.yml')

    # Parse configuration
    vessel_params, hyd_params, control_params = read_input(input_file)

    # Create vessel instance
    vessel = Vessel(vessel_params, hyd_params, control_params)

    # Simulation time parameters
    sim_params = vessel_params.get('sim', {})
    dt = float(sim_params.get('dt', 0.1))
    t_end = float(sim_params.get('t_end', 60.0))
    t_span = (0.0, t_end)
    # Create a time grid for output; note that solve_ivp will not
    # necessarily honour exactly these times, but t_eval forces
    # interpolation at the specified points.
    t_eval = np.arange(0.0, t_end + dt, dt)

    # Integrate the equations of motion
    sol = solve_ivp(
        fun=lambda t, y: vessel.vessel_ode(t, y),
        t_span=t_span,
        y0=vessel.initial_state,
        t_eval=t_eval,
        method='RK45',
        rtol=1e-8,
        atol=1e-8
    )

    # Store history on the vessel instance
    vessel.history = {'t': sol.t, 'state': sol.y.T}

    # Save plots
    output_path = os.path.join(os.path.dirname(os.path.abspath(input_file)), 'simulation_results.png')
    plot_results(vessel.history, output_path)

    print(f"Simulation complete. Results saved to {output_path}")


if __name__ == '__main__':
    simulate()