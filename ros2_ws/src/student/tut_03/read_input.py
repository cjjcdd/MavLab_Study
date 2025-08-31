"""Utility functions for reading simulation configuration files.

This module reads YAML configuration files that describe the vessel
parameters, hydrodynamic coefficients and control settings for the
simulation.  Keeping the reading logic separate makes it easier to
modify the configuration format in the future and to unit test the
parsing independently of the simulator itself.

The expected structure of the ``input.yml`` file is documented in
the template provided in this repository.  The hydrodynamic
coefficients are stored in a separate file whose relative path is
specified by the ``hydro_coeffs`` key in ``input.yml``.
"""

from __future__ import annotations

import os
import yaml
from typing import Any, Dict, Tuple


def _load_yaml(path: str) -> Dict[str, Any]:
    """Load a YAML file and return its contents as a dictionary.

    Parameters
    ----------
    path: str
        The path to the YAML file to load.

    Returns
    -------
    dict
        The parsed YAML content.

    Raises
    ------
    FileNotFoundError
        If the specified file does not exist.
    yaml.YAMLError
        If the file contains invalid YAML.
    """
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def read_input(input_path: str) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    """Read the main input file and corresponding hydrodynamic coefficients.

    Parameters
    ----------
    input_path: str
        Path to the YAML file containing all simulation inputs.

    Returns
    -------
    vessel_params: dict
        A dictionary containing the physical properties and initial
        conditions of the vessel.
    hyd_params: dict
        A dictionary containing the non‑dimensional hydrodynamic
        derivatives.
    control_params: dict
        A dictionary describing the control strategy and associated
        parameters.
    """
    cfg = _load_yaml(input_path)

    # Basic sanity checking
    if 'vessel' not in cfg:
        raise KeyError("Missing 'vessel' section in input file")
    if 'simulation' not in cfg:
        raise KeyError("Missing 'simulation' section in input file")
    if 'control' not in cfg:
        raise KeyError("Missing 'control' section in input file")
    if 'hydro_coeffs' not in cfg:
        raise KeyError("Missing 'hydro_coeffs' key in input file")

    vessel_params: Dict[str, Any] = cfg['vessel']
    sim_params: Dict[str, Any] = cfg['simulation']
    control_params: Dict[str, Any] = cfg['control']

    # Merge simulation parameters into vessel params for convenience
    vessel_params['sim'] = sim_params

    # Resolve path to hydrodynamic coefficients relative to the input file
    input_dir = os.path.dirname(os.path.abspath(input_path))
    hydro_path = os.path.join(input_dir, cfg['hydro_coeffs'])
    hyd_params: Dict[str, Any] = _load_yaml(hydro_path)

    return vessel_params, hyd_params, control_params