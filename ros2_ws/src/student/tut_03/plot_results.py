"""Plotting utilities for the vessel simulator.

This module contains a helper function to visualise the outcome of a
manoeuvring simulation.  It generates a multi‑panel figure showing
the ship's trajectory, velocity, turn rate, rudder deflection and
heading angle over time.  The figure is saved to a file specified
by the caller.
"""

from __future__ import annotations

import numpy as np
import matplotlib
# Use Agg backend for environments without a display (e.g. during testing)
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Dict, Any


def plot_results(history: Dict[str, Any], filename: str) -> None:
    """Create a multi‑panel plot of the simulation results.

    Parameters
    ----------
    history: dict
        A dictionary with keys ``'t'`` (1D array of time stamps) and
        ``'state'`` (2D array with shape ``(n_steps, 13)``) containing
        the state history returned by the solver.
    filename: str
        The path to the output image file.  The directory must
        already exist; the file will be overwritten.
    """
    t = np.asarray(history['t'], dtype=float)
    state = np.asarray(history['state'], dtype=float)
    # Extract relevant quantities
    x = state[:, 0]
    y = state[:, 1]
    u = state[:, 6]
    v = state[:, 7]
    r = state[:, 11]
    delta = state[:, 12]
    psi = state[:, 5]

    speed = np.hypot(u, v)

    fig, axs = plt.subplots(5, 1, figsize=(8, 10), sharex=False)

    # Trajectory
    axs[0].plot(x, y, label='Trajectory')
    axs[0].set_xlabel('x [m]')
    axs[0].set_ylabel('y [m]')
    axs[0].set_title('Ship trajectory in the horizontal plane')
    axs[0].grid(True)

    # Velocity magnitude
    axs[1].plot(t, speed, label='Speed')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Speed [m/s]')
    axs[1].set_title('Speed time history')
    axs[1].grid(True)

    # Turn rate (yaw rate)
    axs[2].plot(t, r, label='Yaw rate')
    axs[2].set_xlabel('Time [s]')
    axs[2].set_ylabel('r [rad/s]')
    axs[2].set_title('Yaw rate time history')
    axs[2].grid(True)

    # Rudder angle
    axs[3].plot(t, delta, label='Rudder angle')
    axs[3].set_xlabel('Time [s]')
    axs[3].set_ylabel('δ [rad]')
    axs[3].set_title('Rudder angle time history')
    axs[3].grid(True)

    # Heading angle
    axs[4].plot(t, psi, label='Heading angle')
    axs[4].set_xlabel('Time [s]')
    axs[4].set_ylabel('ψ [rad]')
    axs[4].set_title('Heading angle time history')
    axs[4].grid(True)

    plt.tight_layout()
    fig.savefig(filename, dpi=300)
    plt.close(fig)