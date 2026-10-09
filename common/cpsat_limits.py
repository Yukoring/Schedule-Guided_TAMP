#!/usr/bin/env python3
"""CP-SAT time limit per scheduler call, min(5 s, remaining budget), and a log of every solve."""
import time, types
from ortools.sat.python import cp_model as _real

BASE_S = 5.0


class _Count(_real.CpSolverSolutionCallback):
    def __init__(self, t0):
        super().__init__()
        self.t0 = t0
        self.n = 0
        self.first_s = None
        self.objectives = []

    def on_solution_callback(self):
        self.n += 1
        el = time.monotonic() - self.t0
        if self.first_s is None:
            self.first_s = el
        try:
            self.objectives.append((round(el, 3), self.ObjectiveValue()))
        except Exception:
            pass


def install(cpsat_module, budget, log):
    """log: list that receives one dict per solve."""

    class LimitedSolver(_real.CpSolver):
        # ortools 9.14: CpSolver.Solve is a deprecated alias that calls self.solve; override solve only.
        def solve(self, model, solution_callback=None):
            assert solution_callback is None, "the scheduler never passes a callback"
            rem = budget.remaining()
            base_cap = self.parameters.max_time_in_seconds
            cap = max(0.05, min(BASE_S, rem))
            self.parameters.max_time_in_seconds = cap
            t0 = time.monotonic()
            cb = _Count(t0)
            rec = {
                "base_cap_s": base_cap,
                "applied_cap_s": round(cap, 3),
                "remaining_at_start_s": round(rem, 3),
                "base_s": BASE_S,
                "workers": self.parameters.num_search_workers,
                "seed": self.parameters.random_seed,
            }
            status = super().solve(model, cb)
            rec.update(
                {
                    "status": self.StatusName(status),
                    "wall_s": round(time.monotonic() - t0, 3),
                    "solutions": cb.n,
                    "first_solution_s": cb.first_s,
                    "objective_trace": cb.objectives[:50],
                }
            )
            log.append(rec)
            return status

    proxy = types.SimpleNamespace(**{k: getattr(_real, k) for k in dir(_real) if not k.startswith("_")})
    proxy.CpSolver = LimitedSolver
    cpsat_module.cp_model = proxy
    return proxy
