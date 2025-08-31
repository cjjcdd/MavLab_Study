"""Control law implementations for the vessel simulator.

This module provides simple rudder angle controllers used for
open‑loop simulations.  Two strategies are implemented:

* **Fixed rudder control**: a constant rudder angle throughout the
  simulation.  This is often used to study steady turning behaviour
  such as a turning circle manoeuvre.
* **Switching rudder control**: a rudder angle that alternates
  periodically between positive and negative values.  This creates
  oscillatory manoeuvres such as zig‑zag tests.

The control functions expect that a global configuration dictionary
has been initialised via :func:`initialize`.  The configuration
contains the amplitude and period of the rudder motion as well as the
selected control type.  See ``input.yml`` for details.
"""

from __future__ import annotations

from typing import Dict, Optional


# Global control configuration populated by initialize().  Using a
# module‑level variable avoids having to thread configuration through
# every function call.
_CONTROL_CONFIG: Optional[Dict[str, float]] = None


def initialize(control_config: Dict[str, float]) -> None:
    """Initialise the control module with parameters.

    Parameters
    ----------
    control_config: dict
        A dictionary containing at least the keys ``type``,
        ``amplitude`` and, for switching control, ``period``.  The
        values should be numeric (the type may be a string).
    """
    global _CONTROL_CONFIG
    _CONTROL_CONFIG = control_config.copy()


def _get_config() -> Dict[str, float]:
    if _CONTROL_CONFIG is None:
        raise RuntimeError(
            "Control module has not been initialised. Call initialize() "
            "with a control configuration before using the control laws.")
    return _CONTROL_CONFIG


def fixed_rudder(t: float, state) -> float:
    """Return a constant rudder angle.

    The constant angle is read from the control configuration.  No
    feedback is used; the argument ``state`` is ignored in this
    implementation.

    Parameters
    ----------
    t: float
        The current simulation time in seconds.  It is ignored for
        fixed control.
    state: any
        The current vessel state (unused).

    Returns
    -------
    float
        The commanded rudder angle in radians.
    """
    cfg = _get_config()
    amplitude = float(cfg.get('amplitude', 0.0))
    return amplitude


def switching_rudder(t: float, state) -> float:
    """Return a periodically switching rudder angle.

    This control law alternates the sign of the rudder angle at
    regular intervals defined by the ``period`` parameter in the
    configuration.  The amplitude of the rudder angle is given by
    ``amplitude``.  The state argument is currently unused but is
    included for future extensions (e.g., state‑dependent switching).

    Parameters
    ----------
    t: float
        The current simulation time in seconds.
    state: any
        The current vessel state (unused).

    Returns
    -------
    float
        The commanded rudder angle in radians.
    """
    cfg = _get_config()
    amplitude = float(cfg.get('amplitude', 0.0))
    period = float(cfg.get('period', 1.0))
    if period <= 0:
        return amplitude
    # Determine how many complete half‑periods have elapsed
    # A full period results in the same sign, so switching occurs every half period
    half_period = period / 2.0
    if half_period <= 0:
        return amplitude
    # Use floor division to determine how many switches have occurred
    switches = int(t // half_period)
    sign = 1.0 if (switches % 2 == 0) else -1.0
    return sign * amplitude