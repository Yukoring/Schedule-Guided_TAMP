#!/usr/bin/env python3
"""Shared parts of the control layer: run context, candidate store, event log, helpers. The method loops are in ours/ and baselines/."""
import copy, hashlib, json, time
from pathlib import Path
from types import SimpleNamespace as NS
from planner_pool import BudgetExpired

# rounds after a valid plan while a colliding candidate is shorter than the incumbent
EXTRA_ROUNDS = 2
# colliding candidates repaired with SIPP per round
SIPP_REPAIR_MAX = 3
GCONST = {"U": 0, "AD": 1, "PD": 4, "RT": 2}
ACTIVE = {
    "FULL": ["U", "AD", "PD", "RT"],
    "NOFB": ["U", "AD", "PD", "RT"],
    "U": ["U"],
    "U_AD_PD": ["U", "AD", "PD"],
    "U_RT": ["U", "RT"],
}
CS = ("-c", "-S")
N = ("-n",)
TIE = 0.01  # planner option profiles and tie tolerance


def label(meta):
    return "%s-%s" % (
        meta.get("guidance", "T"),
        meta.get("profile") or ("cS" if tuple(meta.get("flags", ())) == CS else "n"),
    )


ROTATION = ["U", "AD", "PD", "RT"]
BETA_SCALES = [1.0, 4.0, 16.0]
BETA_MAX = 16.0
SCALE = 100


def sha_obj(x):
    return hashlib.sha256(json.dumps(x, sort_keys=True, default=str).encode()).hexdigest()


class Ctx:
    """Per-run services: candidate store, motion-candidate trace, event log, module timers."""

    def __init__(
        self,
        out,
        world,
        budget,
        pool,
        tool,
        case_index,
        tool_errors,
        timed_events,
        validate_final,
        collision_check,
        load_plan,
    ):
        self.out = Path(out)
        self.world = world
        self.budget = budget
        self.pool = pool
        self.tool = tool
        self.case_index = case_index
        self.tool_errors = tool_errors
        self._timed = timed_events
        self._validate = validate_final
        self.collision_check = collision_check
        self._load = load_plan
        self.modules = {}
        self.n_cand = 0
        self.n_motion = 0
        self.first_valid_s = None
        self.first_valid = None
        self.valid = []
        (self.out / "candidates").mkdir(exist_ok=True)
        (self.out / "motion_candidates").mkdir(exist_ok=True)

    def event(self, **kw):
        with (self.out / "events.jsonl").open("a") as f:
            f.write(json.dumps({"t": round(self.budget.elapsed(), 4), **kw}, default=str) + "\n")

    def add(self, key, dt):
        self.modules[key] = self.modules.get(key, 0.0) + dt

    def validate(self, robot_path, obj_ins):
        v = self._validate(robot_path, obj_ins, self.world.env.robots_map)
        if self.tool:
            v = v + [(e, "portable tool") for e in self.tool_errors(obj_ins, robot_path)]
        return v

    def horizon(self, robot_path):
        ev = {r: self._timed(p["path_cost"]) for r, p in robot_path.items()}
        return max((e[0][-1][1] for e in ev.values() if e[0]), default=0.0), {
            r: [{"t0": s[0], "t1": s[1], "task": s[2], "job": s[3], "pos": list(s[4])} for s in e[1]]
            for r, e in ev.items()
        }

    def record(self, robot_path, obj_ins, pipeline_makespan, meta, source):
        """Validate and store a candidate; returns its record."""
        t = self.budget.elapsed()
        v0 = time.monotonic()
        viol = self.validate(robot_path, obj_ins)
        self.add("validation", time.monotonic() - v0)
        hz, services = self.horizon(robot_path)
        done = self.budget.elapsed()
        n = self.n_cand
        self.n_cand += 1
        rec = {
            "candidate_id": n,
            "pipeline_makespan": pipeline_makespan,
            "trajectory_makespan": hz,
            "validation_start_s": t,
            "validated_elapsed_s": done,
            "within_budget": done <= self.budget.budget_s,
            "budget_s": self.budget.budget_s,
            "violations": viol,
            "valid": not viol,
            "source": source,
            **meta,
        }
        full = {
            **rec,
            "robot_path": robot_path,
            "services": services,
            "model": contract(obj_ins),
            "radii": {k: v.robot_radius for k, v in self.world.env.robots_map.items()},
        }
        tmp = self.out / "candidates" / ("%04d.json.tmp" % n)
        tmp.write_text(json.dumps(full, indent=1, default=encode) + "\n")
        tmp.replace(self.out / "candidates" / ("%04d.json" % n))
        with (self.out / "incumbents.jsonl").open("a") as f:
            f.write(json.dumps(rec, default=encode) + "\n")
        if not viol:
            self.valid.append(rec)
            if self.first_valid_s is None:
                self.first_valid_s = done
                self.first_valid = rec
        return rec

    def best_valid(self):
        """Validated incumbent with the smallest makespan."""
        ok = [r for r in self.valid if r["validated_elapsed_s"] <= self.budget.budget_s]
        return (
            min(ok, key=lambda r: (r["trajectory_makespan"], r["pipeline_makespan"], r["candidate_id"])) if ok else None
        )

    def motion_candidate(self, plan, meta, collisions=None):
        self.n_motion += 1
        rec = {
            "ordinal": self.n_motion,
            "elapsed_s": self.budget.elapsed(),
            "pipeline_makespan": plan.motion_cost,
            "total_cost": plan.total_cost,
            "colliding": None if collisions is None else bool(collisions),
            "collision_count": None if collisions is None else len(collisions),
            **meta,
        }
        full = {**rec, "robot_path": plan.path}
        if self.n_motion == 1:
            (self.out / "motion_candidates" / "first.json").write_text(
                json.dumps(full, indent=1, default=encode) + "\n"
            )
        (self.out / "motion_candidates" / "last.json").write_text(json.dumps(full, indent=1, default=encode) + "\n")
        with (self.out / "motion_candidates" / "index.jsonl").open("a") as f:
            f.write(json.dumps(rec, default=encode) + "\n")
        return rec

    def load(self, generator, call, label, count):
        """Write the candidate block as a plan file and build its trajectories."""
        b = call.candidate_block()
        if b is None:
            return None, None
        pf = call.dir / "candidate.pddl"
        pf.write_text(call.plan_text(b))
        m0 = time.monotonic()
        plan = self._load(generator, str(pf), label, count, False)
        self.add("motion", time.monotonic() - m0)
        if plan is None:
            self.event(type="parser_error", call=call.id, block=b["index"])
            call.meta["parser_error"] = True
            self.pool._write_call(call)
        return plan, b

    def collide(self, path, prm, col_env, tmem, count):
        c0 = time.monotonic()
        r = self.collision_check(path, self.world.env.robots_map, prm, col_env, tmem, count)
        self.add("collision", time.monotonic() - c0)
        return r


def encode(x):
    if hasattr(x, "tolist"):
        return x.tolist()
    if hasattr(x, "item"):
        return x.item()
    if isinstance(x, set):
        return sorted(x)
    raise TypeError(type(x).__name__)


def contract(obj):
    attrs = {
        "robot": ["ind", "action_duration"],
        "job": ["ind", "located", "order"],
        "task": ["ind", "loc"],
        "waypoint": ["ind", "loc"],
    }
    return {k: [{a: getattr(o, a) for a in names} for o in obj.get(k, [])] for k, names in attrs.items()}


def error_exit(c):
    """A call that exited with an error before producing a plan."""
    return bool(c.end_how) and c.end_how.startswith("exited") and c.rc not in (0, None) and not c.blocks


def finish(ctx, end_reason, sub=None, candidate=None, **extra):
    co = (
        [
            r["candidate_id"]
            for r in ctx.valid
            if candidate
            and r["candidate_id"] != candidate["candidate_id"]
            and abs(r["trajectory_makespan"] - candidate["trajectory_makespan"]) <= TIE
        ]
        if candidate
        else []
    )
    fv = ctx.first_valid
    return {
        "found": candidate is not None,
        "adopted_candidate": candidate["candidate_id"] if candidate else None,
        "end_reason": end_reason,
        "sub_reason": sub,
        "return_ready_s": ctx.budget.elapsed(),
        "validation_end_s": candidate["validated_elapsed_s"] if candidate else None,
        "makespan": candidate["trajectory_makespan"] if candidate else None,
        "pipeline_makespan": candidate["pipeline_makespan"] if candidate else None,
        "selected_source": candidate.get("source_label") if candidate else None,
        "selected_release": candidate.get("release") if candidate else None,
        "selected_iteration": candidate.get("iteration") if candidate else None,
        "selected_beta_attempt": candidate.get("beta_attempt") if candidate else None,
        "selected_phase": candidate.get("phase") if candidate else None,
        "co_winners": co,
        "co_winner_sources": [r["source_label"] for r in ctx.valid if r["candidate_id"] in co and "source_label" in r],
        "first_valid_candidate": fv["candidate_id"] if fv else None,
        "first_valid_s": fv["validated_elapsed_s"] if fv else None,
        "first_valid_makespan": fv["trajectory_makespan"] if fv else None,
        "first_valid_source": fv.get("source_label") if fv else None,
        "validated_candidates_total": len(ctx.valid),
        **extra,
    }


def representative(calls):
    """Representative call: smallest planner-reported makespan among complete plans; ties by call order."""
    ready = [c for c in calls if c.has_block()]
    if not ready:
        return None
    return min(
        ready,
        key=lambda c: (
            (
                c.candidate_block()["plan_makespan"]
                if c.candidate_block()["plan_makespan"] is not None
                else float("inf")
            ),
            c.id,
        ),
    )


def two_phase(pool, mk_A, mk_B):
    """Phase A calls first; phase B starts after A has finished."""
    A = mk_A()
    pool.base_round(A)
    B = mk_B()
    pool.base_round(B)
    return A, B


def rebuild_goal_bound(oracle, margin_scale):
    """Deadline vector of the same schedule under another beta."""
    sel = oracle.last_stats["deadline_guidance"]["selected"]
    max_diam = max((r.radius * 2 for r in oracle.robots), default=0.6)
    margin = max(int(round(max_diam / oracle.velocity * SCALE)), SCALE // 2)
    margin = int(round(margin * margin_scale))
    per = {}
    for c in sel:
        per.setdefault(c["robot"], []).append(c)
    gb = {}
    for c in sel:
        legs = per[c["robot"]]
        k = next(i for i, l in enumerate(legs) if l["start_cs"] == c["start_cs"])
        flag = 2 if k == 0 else 0
        gb.setdefault(c["goal"], []).append((c["task"], c["end_cs"] + margin * (k + 1), flag))
    return gb, margin / SCALE


def _single_problem_round(pool, domain, p_name, count):
    """TP and TP+SIPP: one planning round (phase A then phase B, late release); representative by the makespan rule."""
    mkA = lambda: [
        pool.new_call(domain, p_name, flags=CS, profile="cS", release="late", phase="A", geometry_iteration=count)
    ]
    mkB = lambda: [
        pool.new_call(domain, p_name, flags=N, profile="n", release="late", phase="B", geometry_iteration=count)
    ]
    A, B = two_phase(pool, mkA, mkB)
    calls = A + B
    rep = representative(calls)
    how = "base"
    if rep is None:
        arrivals, reason = pool.extend(calls)
        rep = representative(arrivals) if arrivals else None
        how = "extension" if reason == "candidate" else reason
    return calls, rep, how
