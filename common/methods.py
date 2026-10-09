#!/usr/bin/env python3
"""Facade kept for the worker (`import methods as M`): re-exports the shared control-layer pieces and the method loops from their folders.
Stack variants (common/variants/*) overlay this file with their own monolithic version."""
import sys
from pathlib import Path

_R = Path(__file__).resolve().parent.parent
for _p in [_R / "common", _R / "ours", _R / "baselines/cbtamp", _R / "baselines/tp", _R / "baselines/tp_sipp"]:
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
from control_common import *
from control_common import _single_problem_round
from ours_loop import ours_loop
from cbtamp_loop import cbtamp_loop
from tp_loop import planner_only_loop
from tp_sipp_loop import prioritized_loop
