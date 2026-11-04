"""6-DOF flight simulation, reference controller, onboard supervisor and ground link.

Package invariant (docs/SPEC.md §8-10): every dimension, mass or gain used here
is read from ``params.json`` / ``cad/layout.py`` (through ``sim.vehicle``) or
from a CAD / sizing report; nothing is hard-coded.  ``sim.controller``,
``sim.estimator``, ``sim.onboard`` (v0.4 supervisor: commands + interlocks) and
``sim.state_machine.AltitudeFilter`` are the executable specification that the
firmware mirrors.  ``sim.link_proto`` (frame format), ``sim.link_model``
(latency / loss) and ``sim.ground_station_logic`` (ground assist) are pure
Python so the ground station program can import them without numpy.
"""
from __future__ import annotations
