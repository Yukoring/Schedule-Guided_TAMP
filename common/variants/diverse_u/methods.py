#!/usr/bin/env python3
"""Control layer of this stack variant (shared pieces and all method loops in one file)."""
import copy, json, time
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
    "U_MATCH": ["U", "AD", "PD", "RT"],
}
# Diverse U: the call structure of Ours with varied planner search settings instead of guidance
# opportunities, extension) is kept, but every guided slot receives an UNGUIDED problem (const 0: no schedule deadline, no robot-service
# restriction, no schedule-derived initial-edge release) whose object/init-fact order and whose domain action order are permuted by a
# deterministic semantics-preserving permutation (u_match.py). The scheduler is still called (same cost) and gates the slots as in FULL.
U_MATCH_DIV_SEED = 0
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


def ours_loop(ctx, config, gen_cls, domains, oracle_cls, read_domain):
    """One run of Ours. Each round runs guided and unguided planner calls in two phases, refines the candidates with SIPP, validates them, keeps the best valid plan and feeds the conflicts of colliding candidates back as collision regions; when the guided calls return nothing, beta is increased and they are retried."""
    env, prm = ctx.world.env, ctx.world.prm
    budget = ctx.budget
    pool = ctx.pool
    active = ACTIVE[config]
    d_name = read_domain(domains["early"])
    s0 = time.monotonic()
    task_prob = gen_cls(env, prm, d_name, "problem.pddl")
    task_prob.env_instance(pots=0)
    ctx.add("setup", time.monotonic() - s0)
    sched_prob = task_prob
    margin_scale = 1.0
    col_env = []
    tmem = []
    count = 0
    beta_events = []
    pending_beta = None
    iters = []
    incumbent = None
    extra_used = 0
    extra_events = []
    while True:
        budget.check()
        it = {"iteration": count, "margin_scale_at_start": margin_scale, "rounds": []}
        it_coll = []
        task_prob.transient_regions = [m["samples"] for m in tmem] if tmem else []
        v0 = time.monotonic()
        oracle = oracle_cls(sched_prob, margin_scale, region_exclusion=True)
        _, relax_ms, vrp_info = oracle.solver()
        ctx.add("scheduler", time.monotonic() - v0)
        if pending_beta is not None:
            pending_beta["applied"] = True
            pending_beta["applied_iteration"] = count
            pending_beta["applied_margin_scale"] = margin_scale
            beta_events.append(pending_beta)
            ctx.event(**{**pending_beta, "type": "beta_applied"})
            pending_beta = None
        avail = set(["U"])
        if vrp_info.get("goal_bound"):
            avail |= {"AD", "PD"}
        if any(vrp_info.get("robot_task", {}).values()):
            avail.add("RT")
        unavailable = [g for g in active if g not in avail]
        if unavailable:
            ctx.event(
                type="guidance_unavailable",
                iteration=count,
                configs=unavailable,
                cp_status=oracle.last_stats.get("cp_status"),
                calls_not_created=[
                    {"guidance": g, "release": rel, "profile": "n", "phase": "A", "call_status": "GUIDANCE_UNAVAILABLE"}
                    for rel in ("early", "late")
                    for g in unavailable
                ],
            )
        rot = (ctx.case_index + count) % 4
        order = [g for g in ROTATION[rot:] + ROTATION[:rot] if g in active and g in avail]
        it.update(
            {
                "rotation": rot,
                "order": order,
                "guidance_unavailable": unavailable,
                "relax_ms": relax_ms,
                "cp_status": oracle.last_stats.get("cp_status"),
            }
        )
        g0 = time.monotonic()
        p_name = task_prob.generate_problem(
            vrp_info, count, consts=([0] if config == "U_MATCH" else [GCONST[g] for g in order])
        )
        ctx.add("pddl", time.monotonic() - g0)

        def mk(g, rel, flags, phase, attempt=0, pn=None, ms=None):
            ms = ms or margin_scale
            if config == "U_MATCH" and g != "U":
                # guided slot role -> unguided problem with a slot-specific permutation (problem and domain); the role is kept for rotation/retry/logging
                import u_match

                pk = u_match.perm_key(g, rel, count, attempt, "problem", U_MATCH_DIV_SEED)
                dk = u_match.perm_key(g, rel, count, attempt, "domain", U_MATCH_DIV_SEED)
                base_prob = (pn or p_name)[0]
                pf = ctx.out / "pddl_prob_plan" / ("umatch_%s_%s_round%d_att%d.pddl" % (g, rel, count, attempt))
                df = ctx.out / "pddl_prob_plan" / ("umatch_domain_%s_%s_round%d_att%d.pddl" % (g, rel, count, attempt))
                psha = u_match.permute_problem(base_prob, pf, pk)
                dsha = u_match.permute_domain(domains[rel], df, dk)
                eq_p = u_match.check_equivalent(base_prob, pf)
                eq_d = u_match.check_equivalent(domains[rel], df)
                ctx.event(
                    type="umatch_permutation",
                    slot_role=g,
                    release=rel,
                    iteration=count,
                    attempt=attempt,
                    problem_key=pk,
                    domain_key=dk,
                    canonical_problem_sha256=base_prob
                    and __import__("hashlib").sha256(open(base_prob, "rb").read()).hexdigest(),
                    semantic_equivalent_problem=eq_p["equivalent"],
                    semantic_equivalent_domain=eq_d["equivalent"],
                )
                assert eq_p["equivalent"] and eq_d["equivalent"], "U_MATCH permutation changed the semantics"
                return pool.new_call(
                    str(df),
                    str(pf),
                    flags=flags,
                    profile="cS" if flags == CS else "n",
                    guidance=g,
                    release=rel,
                    phase=phase,
                    geometry_iteration=count,
                    beta_attempt=attempt,
                    margin_scale=ms,
                    beta_s=round(0.6 * ms, 3),
                    const=0,
                    slot_role=g,
                    problem_kind="U_permuted",
                    permutation_key=pk,
                )
            return pool.new_call(
                domains[rel],
                (pn or p_name)[GCONST[g]],
                flags=flags,
                profile="cS" if flags == CS else "n",
                guidance=g,
                release=rel,
                phase=phase,
                geometry_iteration=count,
                beta_attempt=attempt,
                margin_scale=ms,
                beta_s=round(0.6 * ms, 3),
                const=GCONST[g],
                slot_role=g,
                problem_kind=("U_canonical" if g == "U" else "guided"),
                permutation_key=None,
            )

        all_calls = []
        beta_attempt = 0
        dr_cand_seen = False
        outcome = None

        def judge(cands_calls, phase):
            """Frozen policy: build every ready candidate, screen collisions, validate the clean ones."""
            nonlocal dr_cand_seen
            loaded = []
            parser_errors = 0
            for c in cands_calls:
                plan, b = ctx.load(task_prob, c, c.meta["guidance"], count)
                if plan is None:
                    parser_errors += 1
                    continue
                if c.meta["guidance"] in ("AD", "PD"):
                    dr_cand_seen = True
                loaded.append((c, plan, b))
            clean, coll = [], []
            for c, plan, b in loaded:
                col, _ = ctx.collide(plan.path, prm, col_env, None, count)
                ctx.motion_candidate(
                    plan,
                    {
                        "call_id": c.id,
                        "source_label": label(c.meta),
                        "guidance": c.meta["guidance"],
                        "profile": c.meta["profile"],
                        "release": c.meta["release"],
                        "iteration": count,
                        "beta_attempt": c.meta["beta_attempt"],
                        "block": b["index"],
                        "phase": phase,
                    },
                    col,
                )
                (coll if col else clean).append((c, plan, col, b))
            valid = []
            invalid = []
            repaired = set()
            if coll:
                import sipp_repair

                for c, plan, col, b in sorted(coll, key=lambda x: x[1].motion_cost)[:SIPP_REPAIR_MAX]:
                    rec = sipp_repair.repair(
                        ctx,
                        task_prob,
                        prm,
                        c,
                        b,
                        {
                            "call_id": c.id,
                            "source_label": label(c.meta) + "+sipp",
                            "guidance": c.meta["guidance"],
                            "profile": c.meta["profile"],
                            "release": c.meta["release"],
                            "iteration": count,
                            "beta_attempt": c.meta["beta_attempt"],
                            "block": b["index"],
                            "variant": GCONST[c.meta["guidance"]],
                            "phase": phase,
                            "repair": "sipp",
                            "naive_pipeline_makespan": plan.motion_cost,
                        },
                        count,
                        col_env,
                    )
                    if rec is None:
                        continue
                    (valid if rec["valid"] else invalid).append(rec)
                    if rec["valid"]:
                        repaired.add(c.id)
            it_coll.extend([x for x in coll if x[0].id not in repaired])
            for c, plan, col, b in clean:
                rec = ctx.record(
                    plan.path,
                    task_prob.obj_ins,
                    plan.motion_cost,
                    {
                        "call_id": c.id,
                        "source_label": label(c.meta),
                        "guidance": c.meta["guidance"],
                        "profile": c.meta["profile"],
                        "release": c.meta["release"],
                        "iteration": count,
                        "beta_attempt": c.meta["beta_attempt"],
                        "block": b["index"],
                        "variant": GCONST[c.meta["guidance"]],
                        "phase": phase,
                        "deadline_vector": (
                            vrp_info.get("goal_bound")
                            if c.meta["guidance"] in ("AD", "PD")
                            and c.meta["beta_attempt"] == 0
                            and c.meta.get("problem_kind") != "U_permuted"
                            else None
                        ),
                    },
                    "round_candidate",
                )
                (valid if rec["valid"] else invalid).append(rec)
            it["rounds"].append(
                {
                    "phase": phase,
                    "beta_attempt": beta_attempt,
                    "calls": [c.id for c in cands_calls],
                    "loaded": len(loaded),
                    "parser_errors": parser_errors,
                    "clean": len(clean),
                    "colliding": len(coll),
                    "valid": len(valid),
                    "invalid": len(invalid),
                    "sipp_repaired_valid": len(repaired),
                }
            )
            return loaded, clean, coll, valid, invalid, parser_errors

        def decide(loaded, coll, valid, invalid):
            if valid:
                return (
                    "VALID",
                    min(valid, key=lambda r: (r["trajectory_makespan"], r["pipeline_makespan"], r["candidate_id"])),
                )
            if coll:
                return ("FEEDBACK", coll)
            if loaded:
                return ("NO_REPAIRABLE_CONFLICT", invalid)
            return None

        # phase A: U unguided, AD/PD/RT guided (early then late)
        A = [mk(g, rel, CS if g == "U" else N, "A") for rel in ("early", "late") for g in order]
        all_calls += A
        pool.base_round(A)
        la, ca, colA, vA, iA, _ = judge([c for c in A if c.has_block()], "A")
        # phase B: U with the second planner profile (early, late)
        B = [mk("U", rel, N, "B") for rel in ("early", "late")] if "U" in order else []
        all_calls += B
        if B:
            pool.base_round(B)
        lb, cb, colB, vB, iB, _ = judge([c for c in B if c.has_block()], "B")
        outcome = decide(la + lb, colA + colB, vA + vB, iA + iB)
        while outcome is None:
            dr = [g for g in order if g in ("AD", "PD")]
            if dr and margin_scale < BETA_MAX and not dr_cand_seen:
                pool.terminate([c for c in all_calls if c.meta["guidance"] in ("AD", "PD")], "killed_beta_retry")
                margin_scale = min(BETA_MAX, margin_scale * 4)
                beta_attempt += 1
                gb, beta_s = rebuild_goal_bound(oracle, margin_scale)
                info2 = dict(vrp_info)
                info2["goal_bound"] = gb
                if config == "U_MATCH":
                    p2 = p_name  # shadow beta: same retry opportunity, next permutation of the canonical unguided problem, no deadline is generated
                    ctx.event(
                        type="beta_shadow",
                        iteration=count,
                        beta_attempt=beta_attempt,
                        margin_scale=margin_scale,
                        beta_s=beta_s,
                        note="U_MATCH: retry slots get the next permutation of the unguided problem; deadline vector not used",
                    )
                else:
                    keep = task_prob.pFilename
                    task_prob.pFilename = "problem_beta%d.pddl" % beta_attempt
                    g0 = time.monotonic()
                    p2 = task_prob.generate_problem(info2, count, consts=[GCONST[g] for g in dr])
                    ctx.add("pddl", time.monotonic() - g0)
                    task_prob.pFilename = keep
                new = [
                    mk(g, rel, N, "beta", beta_attempt, pn=p2, ms=margin_scale) for rel in ("early", "late") for g in dr
                ]
                ev = {
                    "type": "beta_increase",
                    "trigger": "no parseable candidate from any call of the base round (beta attempt)",
                    "iteration": count,
                    "beta_attempt": beta_attempt,
                    "margin_scale": margin_scale,
                    "beta_s": beta_s,
                    "call_ids": [c.id for c in new],
                    "applied": True,
                    "scheduler_resolved": False,
                }
                beta_events.append(ev)
                ctx.event(**ev)
                all_calls += new
                pool.base_round(new)
                l, c_, co, v, i, _ = judge([c for c in new if c.has_block()], "beta%d" % beta_attempt)
                outcome = decide(l, co, v, i)
                continue
            arrivals, reason = pool.extend(all_calls)
            if reason == "candidate":
                l, c_, co, v, i, _ = judge(arrivals, "extension")
                outcome = decide(l, co, v, i)
                continue  # parse failure of the arriving block: keep extending
            outcome = ("EXHAUSTED", None)
        it["beta_attempts"] = beta_attempt
        it["calls"] = [c.id for c in all_calls]
        it["outcome"] = outcome[0]
        iters.append(it)
        ctx.event(type="iteration_end", **{k: v for k, v in it.items() if k != "rounds"})
        if outcome[0] == "VALID" and (
            incumbent is None or outcome[1]["trajectory_makespan"] < incumbent["trajectory_makespan"] - TIE
        ):
            incumbent = outcome[1]
        if config == "NOFB":
            # No FB: stop after the first round
            # No FB: stop after the first round
            pool.terminate(all_calls, "killed_run_end")
            if outcome[0] == "NO_REPAIRABLE_CONFLICT":
                kinds = sorted({k for r in outcome[1] for k, _ in r["violations"]})
                return finish(
                    ctx,
                    "NO_REPAIRABLE_CONFLICT",
                    "INVALID:" + "+".join(kinds),
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                )
            if outcome[0] == "EXHAUSTED":
                errs = [c.id for c in all_calls if error_exit(c)]
                return finish(
                    ctx,
                    "PLANNER_ERROR" if errs and len(errs) == len(all_calls) else "SEARCH_EXHAUSTED",
                    ("calls_with_error_exit=%d" % len(errs)) if errs else None,
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                )
            if incumbent is not None:
                return finish(
                    ctx,
                    "VALID",
                    "feedback disabled: incumbent of the first round",
                    candidate=incumbent,
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                    extra_rounds_used=0,
                    extra_round_events=[],
                )
            return finish(
                ctx,
                "SEARCH_EXHAUSTED",
                "feedback disabled: no validated candidate in the single round (%d colliding)" % len(outcome[1]),
                iterations=iters,
                beta_events=beta_events,
                geometry_iterations=count + 1,
                final_margin_scale=margin_scale,
            )
        if incumbent is not None:
            shorter = [x for x in it_coll if x[1].motion_cost < incumbent["trajectory_makespan"] - TIE]
            if EXTRA_ROUNDS and extra_used < EXTRA_ROUNDS and shorter and outcome[0] in ("VALID", "FEEDBACK"):
                extra_used += 1
                best_short = min(shorter, key=lambda x: x[1].motion_cost)
                ev = {
                    "type": "extra_round",
                    "iteration": count,
                    "extra_round": extra_used,
                    "incumbent_candidate": incumbent["candidate_id"],
                    "incumbent_makespan": incumbent["trajectory_makespan"],
                    "incumbent_source": incumbent.get("source_label"),
                    "shorter_colliding": len(shorter),
                    "best_colliding_source": label(best_short[0].meta),
                    "best_colliding_pipeline_makespan": best_short[1].motion_cost,
                    "round_outcome": outcome[0],
                }
                extra_events.append(ev)
                ctx.event(**ev)
                outcome = ("FEEDBACK", shorter)
            else:
                pool.terminate(all_calls, "killed_run_end")
                sub = (
                    None
                    if not extra_used
                    else "incumbent returned after %d extra round(s) (%s)"
                    % (
                        extra_used,
                        (
                            "no shorter colliding candidate left"
                            if not shorter
                            else (
                                "extra-round limit %d reached" % EXTRA_ROUNDS
                                if extra_used >= EXTRA_ROUNDS
                                else "round ended with " + outcome[0]
                            )
                        ),
                    )
                )
                return finish(
                    ctx,
                    "VALID",
                    sub,
                    candidate=incumbent,
                    iterations=iters,
                    beta_events=beta_events,
                    geometry_iterations=count + 1,
                    final_margin_scale=margin_scale,
                    extra_rounds_used=extra_used,
                    extra_round_events=extra_events,
                )
        if outcome[0] == "NO_REPAIRABLE_CONFLICT":
            kinds = sorted({k for r in outcome[1] for k, _ in r["violations"]})
            pool.terminate(all_calls, "killed_run_end")
            return finish(
                ctx,
                "NO_REPAIRABLE_CONFLICT",
                "INVALID:" + "+".join(kinds),
                iterations=iters,
                beta_events=beta_events,
                geometry_iterations=count + 1,
                final_margin_scale=margin_scale,
            )
        if outcome[0] == "EXHAUSTED":
            errs = [c.id for c in all_calls if error_exit(c)]
            pool.terminate(all_calls, "killed_run_end")
            return finish(
                ctx,
                "PLANNER_ERROR" if errs and len(errs) == len(all_calls) else "SEARCH_EXHAUSTED",
                ("calls_with_error_exit=%d" % len(errs)) if errs else None,
                iterations=iters,
                beta_events=beta_events,
                geometry_iterations=count + 1,
                final_margin_scale=margin_scale,
            )
        # feedback: the colliding candidate with the smallest makespan supplies the collision regions
        coll = outcome[1]
        c, plan, col, b = min(coll, key=lambda x: x[1].motion_cost)
        _, sel_env = ctx.collide(plan.path, prm, col_env, tmem, count)
        if sel_env != -1:
            col_env = sel_env
        s0 = time.monotonic()
        task_prob.env_update(col_env)
        ctx.add("setup", time.monotonic() - s0)
        ctx.event(
            type="feedback",
            iteration=count,
            repaired_call=c.id,
            source=label(c.meta),
            makespan=plan.motion_cost,
            regions=len(col_env) if isinstance(col_env, list) else None,
            transient=len(tmem),
        )
        if [g for g in order if g in ("AD", "PD")] and not dr_cand_seen and margin_scale < BETA_MAX:
            margin_scale = min(BETA_MAX, margin_scale * 4)
            pending_beta = {
                "type": "beta_increase",
                "trigger": "AD/PD produced no parseable candidate in this iteration; next iteration goes on with U/RT feedback",
                "iteration": count,
                "margin_scale": margin_scale,
                "beta_s": round(0.6 * margin_scale, 3),
                "applied": False,
            }
            ctx.event(**{**pending_beta, "type": "beta_increase_pending"})
        pool.terminate(all_calls, "killed_next_iteration")
        count += 1


# ----------------------------------------------------------------------------------------------- CBTAMP
def cbtamp_loop(ctx, gen_cls, domains, read_domain):
    """CBTAMP: an initial planning round, then repair rounds in which collision regions revise the planning problem; candidates are adopted by the original CBTAMP rule."""
    env, prm = ctx.world.env, ctx.world.prm
    budget = ctx.budget
    pool = ctx.pool
    d_name = read_domain(domains["early"])
    s0 = time.monotonic()
    makeprob = gen_cls(env, prm, d_name, "problem.pddl")
    makeprob.env_instance()
    ctx.add("setup", time.monotonic() - s0)
    g0 = time.monotonic()
    p_name = makeprob.generate_problem()
    ctx.add("pddl", time.monotonic() - g0)
    count = 0
    col_env = []
    iters = []

    def produce(count):
        mkA = lambda: [
            pool.new_call(
                domains[rel], p_name, flags=CS, profile="cS", release=rel, phase="A", geometry_iteration=count
            )
            for rel in ("early", "late")
        ]
        mkB = lambda: [
            pool.new_call(domains[rel], p_name, flags=N, profile="n", release=rel, phase="B", geometry_iteration=count)
            for rel in ("early", "late")
        ]
        A, B = two_phase(pool, mkA, mkB)
        calls = A + B
        ready = [c for c in calls if c.has_block()]
        how = "base"
        if not ready:
            arrivals, reason = pool.extend(calls)
            ready = arrivals
            how = "extension" if reason == "candidate" else reason
        return calls, ready, how

    def meta_of(c, b, how):
        return {
            "call_id": c.id,
            "source_label": label(c.meta),
            "profile": c.meta["profile"],
            "release": c.meta["release"],
            "iteration": c.meta["geometry_iteration"],
            "block": b["index"],
            "phase": how,
        }

    calls, ready, how = produce(0)
    if not ready:
        errs = [c for c in calls if error_exit(c)]
        pool.terminate(calls, "killed_run_end")
        return finish(
            ctx,
            "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
            "initial round: " + how,
            iterations=iters,
        )
    chosen = representative(ready)
    init, b = ctx.load(makeprob, chosen, "Initial Plan", 0)
    if init is None:
        pool.terminate(calls, "killed_run_end")
        return finish(ctx, "PARSER_ERROR", "initial plan", iterations=iters)
    early_col, col_env = ctx.collide(init.path, prm, col_env, None, 0)
    ctx.motion_candidate(init, meta_of(chosen, b, how), early_col)
    iters.append(
        {
            "iteration": 0,
            "calls": [c.id for c in calls],
            "ready": [c.id for c in ready],
            "chosen": chosen.id,
            "chosen_source": label(chosen.meta),
            "how": how,
            "colliding": bool(early_col),
        }
    )
    if not early_col:
        rec = ctx.record(
            init.path,
            makeprob.obj_ins,
            init.motion_cost,
            {**meta_of(chosen, b, how), "variant": 0 if chosen.meta["release"] == "early" else 1},
            "initial",
        )
        pool.terminate(calls, "killed_run_end")
        if rec["valid"]:
            return finish(ctx, "VALID", candidate=rec, iterations=iters)
        return finish(
            ctx,
            "NO_REPAIRABLE_CONFLICT",
            "INVALID:" + "+".join(sorted({k for k, _ in rec["violations"]})),
            iterations=iters,
        )
    next_collisions = early_col
    while next_collisions:
        pool.terminate(calls, "killed_next_iteration")
        budget.check()
        count += 1
        s0 = time.monotonic()
        makeprob.env_update(col_env)
        ctx.add("setup", time.monotonic() - s0)
        g0 = time.monotonic()
        p_name = makeprob.generate_problem(count)
        ctx.add("pddl", time.monotonic() - g0)
        calls, ready, how = produce(count)
        if not ready:
            errs = [c for c in calls if error_exit(c)]
            pool.terminate(calls, "killed_run_end")
            return finish(
                ctx,
                "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
                "iteration %d: %s" % (count, how),
                iterations=iters,
            )
        rep_e = representative([c for c in ready if c.meta["release"] == "early"])
        rep_l = representative([c for c in ready if c.meta["release"] == "late"])
        early, eb = ctx.load(makeprob, rep_e, "Early Plan", count) if rep_e else (None, None)
        late, lb = ctx.load(makeprob, rep_l, "Late Plan", count) if rep_l else (None, None)
        if not early and not late:
            pool.terminate(calls, "killed_run_end")
            return finish(ctx, "PARSER_ERROR", "iteration %d" % count, iterations=iters)
        early_col, late_col = [], []
        env_e = env_l = None
        if early:
            early_col, env_e = ctx.collide(early.path, prm, col_env, None, 0)
            ctx.motion_candidate(early, meta_of(rep_e, eb, how), early_col)
        if late:
            base = -1 if early_col else col_env
            late_col, env_l = ctx.collide(late.path, prm, base, None, 0)
            ctx.motion_candidate(late, meta_of(rep_l, lb, how), late_col)
        next_collisions = []
        valid = []
        invalid = []
        if late:
            if not late_col:
                rec = ctx.record(
                    late.path,
                    makeprob.obj_ins,
                    late.motion_cost,
                    {**meta_of(rep_l, lb, how), "variant": 1},
                    "round_candidate",
                )
                (valid if rec["valid"] else invalid).append(rec)
            else:
                next_collisions = late_col
                col_env = env_l
        if early:
            if not early_col:
                rec = ctx.record(
                    early.path,
                    makeprob.obj_ins,
                    early.motion_cost,
                    {**meta_of(rep_e, eb, how), "variant": 0},
                    "round_candidate",
                )
                (valid if rec["valid"] else invalid).append(rec)
            else:
                next_collisions = early_col
                col_env = env_e
        iters.append(
            {
                "iteration": count,
                "calls": [c.id for c in calls],
                "ready": [c.id for c in ready],
                "how": how,
                "early_rep": rep_e.id if rep_e else None,
                "late_rep": rep_l.id if rep_l else None,
                "valid": len(valid),
                "invalid": len(invalid),
                "colliding": bool(next_collisions),
            }
        )
        if valid:
            best = min(valid, key=lambda r: (r["pipeline_makespan"], r["candidate_id"]))
            pool.terminate(calls, "killed_run_end")
            return finish(ctx, "VALID", candidate=best, iterations=iters)
        if not next_collisions:
            pool.terminate(calls, "killed_run_end")
            return finish(
                ctx,
                "NO_REPAIRABLE_CONFLICT",
                "INVALID:" + "+".join(sorted({k for r in invalid for k, _ in r["violations"]})),
                iterations=iters,
            )
    pool.terminate(calls, "killed_run_end")
    return finish(ctx, "SEARCH_EXHAUSTED", "loop left without collisions", iterations=iters)


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


def planner_only_loop(ctx, gen_cls, domains, read_domain, prm_cls, log_prm):
    env = ctx.world.env
    prm = ctx.world.prm
    budget = ctx.budget
    pool = ctx.pool
    d_name = read_domain(domains["early"])
    n = 0
    iters = []
    while True:
        budget.check()
        s0 = time.monotonic()
        makeprob = gen_cls(env, prm, d_name, "problem.pddl")
        makeprob.env_instance()
        ctx.add("setup", time.monotonic() - s0)
        g0 = time.monotonic()
        p_name = makeprob.generate_problem(n)
        ctx.add("pddl", time.monotonic() - g0)
        calls, rep, how = _single_problem_round(pool, domains["late"], p_name, n)
        if rep is None:
            pool.terminate(calls, "killed_run_end")
            errs = [c for c in calls if error_exit(c)]
            return finish(
                ctx,
                "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
                "resample %d: %s" % (n, how),
                iterations=iters,
                prm_resamples=n,
            )
        plan, b = ctx.load(makeprob, rep, "Plan", 0)
        if plan is None:
            pool.terminate(calls, "killed_run_end")
            return finish(ctx, "PARSER_ERROR", "resample %d" % n, iterations=iters, prm_resamples=n)
        meta = {
            "call_id": rep.id,
            "source_label": label(rep.meta),
            "profile": rep.meta["profile"],
            "release": "late",
            "iteration": n,
            "block": b["index"],
            "phase": how,
        }
        next_col, col_env = ctx.collide(plan.path, prm, [], None, 0)
        ctx.motion_candidate(plan, meta, next_col)
        iters.append(
            {
                "iteration": n,
                "calls": [c.id for c in calls],
                "representative": rep.id,
                "source": label(rep.meta),
                "how": how,
                "colliding": bool(next_col),
            }
        )
        pool.terminate(calls, "killed_next_iteration" if next_col else "killed_run_end")
        if not next_col:
            rec = ctx.record(plan.path, makeprob.obj_ins, plan.motion_cost, {**meta, "variant": 0}, "final")
            if rec["valid"]:
                return finish(ctx, "VALID", candidate=rec, iterations=iters, prm_resamples=n)
            return finish(
                ctx,
                "NO_REPAIRABLE_CONFLICT",
                "INVALID:" + "+".join(sorted({k for k, _ in rec["violations"]})),
                iterations=iters,
                prm_resamples=n,
            )
        # resample the roadmap and plan again; no region feedback
        budget.check()
        r0 = time.monotonic()
        prm = prm_cls(env)
        init_samples, init_roadmap, goal_samples = prm.ConstructPhase(goals_map=env.goals_map)
        ctx.world.goal_samples = goal_samples
        n += 1
        ctx.add("prm_resample", time.monotonic() - r0)
        log_prm(n, init_samples, init_roadmap, goal_samples, prm)


# ----------------------------------------------------------------------------------------------- Task + prioritized
def prioritized_loop(ctx, gen_cls, domains, read_domain, solver_factory):
    env, prm = ctx.world.env, ctx.world.prm
    pool = ctx.pool
    d_name = read_domain(domains["early"])
    s0 = time.monotonic()
    makeprob = gen_cls(env, prm, d_name, "problem.pddl")
    makeprob.env_instance()
    ctx.add("setup", time.monotonic() - s0)
    g0 = time.monotonic()
    p_name = makeprob.generate_problem()
    ctx.add("pddl", time.monotonic() - g0)
    calls, rep, how = _single_problem_round(pool, domains["late"], p_name, 0)
    if rep is None:
        pool.terminate(calls, "killed_run_end")
        errs = [c for c in calls if error_exit(c)]
        return finish(
            ctx,
            "PLANNER_ERROR" if errs and len(errs) == len(calls) else "SEARCH_EXHAUSTED",
            "task plan: " + how,
            motion_attempts=0,
        )
    b = rep.candidate_block()
    pf = rep.dir / "candidate.pddl"
    pf.write_text(rep.plan_text(b))
    pool.terminate(calls, "killed_run_end")
    meta = {
        "call_id": rep.id,
        "source_label": label(rep.meta),
        "profile": rep.meta["profile"],
        "release": "late",
        "iteration": 0,
        "block": b["index"],
        "phase": how,
    }
    try:
        m0 = time.monotonic()
        parsed = makeprob.parse_plan(str(pf))
        solver = solver_factory(
            prm.task_roadmap, prm.task_samples, parsed, starts={n: r.pos for n, r in env.robots_map.items()}
        )
        robot_path, makespan, total_cost = solver.find_solution()
        ctx.add("motion", time.monotonic() - m0)
    except Exception as e:
        ctx.event(type="parser_error", call=rep.id, error=repr(e)[:300])
        return finish(ctx, "PARSER_ERROR", repr(e)[:160], motion_attempts=1)
    ctx.event(
        type="prioritized_motion",
        call=rep.id,
        source=label(rep.meta),
        success=bool(robot_path),
        makespan=makespan,
        handover_waits=getattr(solver, "handover_waits", None),
        idle_robots=sorted(getattr(solver, "idle", [])),
    )
    if not robot_path:
        return finish(
            ctx,
            "NO_PATH",
            "prioritized motion planning found no path; single attempt by protocol",
            motion_attempts=1,
            task_plan_source=label(rep.meta),
        )
    plan = NS(path=robot_path, motion_cost=makespan, total_cost=total_cost)
    collisions, _ = ctx.collide(robot_path, prm, [], None, 0)
    ctx.motion_candidate(plan, meta, collisions)
    if collisions:
        return finish(
            ctx,
            "MOTION_REJECTED",
            "shared collision checker: %d collision(s)" % len(collisions),
            motion_attempts=1,
            task_plan_source=label(rep.meta),
        )
    rec = ctx.record(robot_path, copy.deepcopy(makeprob.obj_ins), makespan, {**meta, "variant": 0}, "prioritized")
    if rec["valid"]:
        return finish(ctx, "VALID", candidate=rec, motion_attempts=1)
    return finish(
        ctx,
        "MOTION_REJECTED",
        "INVALID:" + "+".join(sorted({k for k, _ in rec["violations"]})),
        motion_attempts=1,
        task_plan_source=label(rep.meta),
    )
