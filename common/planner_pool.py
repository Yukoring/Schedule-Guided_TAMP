#!/usr/bin/env python3
"""POPF2 process pool and budget clock. Each planner call is one process pinned to one core; calls get a base time slice, can be paused and resumed, and stream plans as they are found."""
import json, os, re, shutil, signal, subprocess, threading, time
from pathlib import Path

ACTION_RE = re.compile(r"^\s*([\d.]+)\s*:\s*\((.*)\)\s*\[\s*([\d.]+)\s*\]\s*$")
METRIC_RE = re.compile(r"; Plan found with metric\s+([-\d.]+)")
COST_RE = re.compile(r";\s*Cost:\s*([-\d.]+)")
BINARY = "code/planners/popf2"
# option profiles of a call: "-c -S" (POPF2 -c -S) and "-n" (POPF2 default options)
FLAGS = {("-c", "-S"): ("-c", "-S"), ("-n",): ()}
CLK = os.sysconf("SC_CLK_TCK")
BASE_S = 5.0
QUANTUM_S = 1.0
MAX_ACTIVE = 4
POLL_S = 0.02


class BudgetExpired(Exception):
    pass


class Budget:
    def __init__(self, t0, budget_s):
        self.t0 = t0
        self.budget_s = budget_s
        self.deadline = t0 + budget_s

    def elapsed(self):
        return time.monotonic() - self.t0

    def remaining(self):
        return self.deadline - time.monotonic()

    def expired(self):
        return time.monotonic() >= self.deadline

    def check(self):
        if self.expired():
            raise BudgetExpired("budget %.1f s exhausted" % self.budget_s)


def proc_state(pid):
    try:
        return open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()[0]
    except OSError:
        return None


def proc_cpu_s(pid):
    try:
        f = open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()
        return (int(f[11]) + int(f[12])) / CLK
    except (OSError, IndexError, ValueError):
        return None


def proc_affinity(pid):
    try:
        return next(
            l.split(":", 1)[1].strip() for l in open("/proc/%d/status" % pid) if l.startswith("Cpus_allowed_list")
        )
    except (OSError, StopIteration):
        return None


class PlannerCall:
    def __init__(self, pool, call_id, domain, problem, meta):
        self.pool = pool
        self.id = call_id
        self.dir = pool.calls_dir / ("%04d" % call_id)
        self.dir.mkdir(parents=True)
        (self.dir / "blocks").mkdir()
        self.domain = str(domain)
        self.problem = str(problem)
        self.meta = dict(meta)
        # option profile of this call
        flags = self.meta.pop("flags", None)
        self.flags = tuple(flags) if flags else tuple(pool.flags)
        self.profile = "cS" if "-c" in self.flags else ("n" if "-n" in self.flags else "+".join(self.flags))
        shutil.copy2(domain, self.dir / "domain.pddl")
        shutil.copy2(problem, self.dir / "problem.pddl")
        self.state = "queued"
        self.proc = None
        self.pid = None
        self.pgid = None
        self.cpu = None
        self.cpus_used = []
        self.rc = None
        self.end_how = None
        self.argv = None
        self.affinity = None
        self.events = []
        self.active_s = 0.0
        self.paused_s = 0.0
        self.queued_s = 0.0
        self._last = time.monotonic()
        self.created_at = pool.budget.elapsed()
        self.started_at = None
        self.ended_at = None
        self.blocks = []
        self._cur = None
        self._lock = threading.Lock()
        self.reader = None
        self.eof = False
        self.eof_at = None
        self.partial_tail = None
        self.n_lines = 0
        self.rusage = None
        self.sampled_cpu_s = None
        self.quantum_start = None
        self.incomplete_at_end = False
        self.metric_increase = False
        self.ext_mark = None

    # ---- stdout grammar ----
    def _read(self, fd, raw):
        buf = b""
        while True:
            try:
                chunk = os.read(fd, 65536)
            except OSError:
                break
            if not chunk:
                break
            raw.write(chunk)
            raw.flush()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._line(line.decode("utf-8", "replace"))
        if buf:
            self.partial_tail = buf.decode("utf-8", "replace")[-200:]
        raw.close()
        with self._lock:
            self.eof = True
            self.eof_at = self.pool.budget.elapsed()

    def _line(self, line):
        t = self.pool.budget.elapsed()
        with self._lock:
            self.n_lines += 1
            if "; Plan found" in line or "; Solution Found" in line:
                m = METRIC_RE.search(line)
                self._cur = {"lines": [], "opened_at": t, "metric": float(m.group(1)) if m else None}
                return
            if self._cur is not None:
                if "; Cost:" in line and self._cur.get("metric") is None:
                    m = COST_RE.search(line)
                    if m:
                        self._cur["metric"] = float(m.group(1))
                if line.startswith(";") or "[" in line:
                    self._cur["lines"].append(line)
                else:
                    if self._cur["lines"]:
                        self._close_block(t, "terminator")
                    self._cur = None

    def _close_block(self, t, how):
        cur = self._cur
        actions = [l for l in cur["lines"] if ACTION_RE.match(l)]
        ends = [float(m.group(1)) + float(m.group(3)) for l in actions for m in [ACTION_RE.match(l)]]
        b = {
            "index": len(self.blocks),
            "metric": cur["metric"],
            "opened_at": cur["opened_at"],
            "complete_at": t,
            "terminated_by": how,
            "n_lines": len(cur["lines"]),
            "n_actions": len(actions),
            "valid_syntax": len(actions) > 0,
            "plan_makespan": max(ends) if ends else None,
            "file": "blocks/%02d.txt" % len(self.blocks),
        }
        if (
            self.blocks
            and b["metric"] is not None
            and self.blocks[-1]["metric"] is not None
            and b["metric"] > self.blocks[-1]["metric"] + 1e-9
        ):
            self.metric_increase = True
        (self.dir / b["file"]).write_text("\n".join(cur["lines"]) + "\n")
        self.blocks.append(b)
        self._cur = None

    def candidate_block(self, since=None):
        """Last complete, syntactically valid block (optionally completed at/after `since`)."""
        with self._lock:
            ok = [b for b in self.blocks if b["valid_syntax"] and (since is None or b["complete_at"] >= since)]
        return ok[-1] if ok else None

    def has_block(self, since=None):
        return self.candidate_block(since) is not None

    def plan_text(self, block):
        return (self.dir / block["file"]).read_text()

    def status(self):
        """Call states: CANDIDATE (plan available), PRESERVED (paused), RUNNING, QUEUED, NOT_STARTED_BUDGET, NO_PLAN_EXITED, PLANNER_ERROR, KILLED."""
        if self.candidate_block() is not None:
            return "CANDIDATE"
        if self.state == "queued":
            return "QUEUED"
        if self.state == "running":
            return "RUNNING"
        if self.state == "paused":
            return "PRESERVED"
        if self.end_how and self.end_how.startswith("never_started"):
            return "NOT_STARTED_BUDGET"
        if self.end_how and self.end_how.startswith("exited"):
            return "NO_PLAN_EXITED" if self.rc == 0 else "PLANNER_ERROR"
        return "KILLED_NO_PLAN"

    # ---- accounting ----
    def _tick(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        if self.state == "running":
            self.active_s += dt
        elif self.state == "paused":
            self.paused_s += dt
        elif self.state == "queued":
            self.queued_s += dt
        if self.state == "running" and self.pid:
            c = proc_cpu_s(self.pid)
            if c is not None:
                self.sampled_cpu_s = c

    def _transition(self, state, event, detail=None):
        self._tick()
        self.state = state
        self.events.append(
            {"t": round(self.pool.budget.elapsed(), 4), "event": event, "pid": self.pid, **(detail or {})}
        )
        self.pool._write_call(self)

    def record(self):
        with self._lock:
            blocks = list(self.blocks)
            cur = self._cur
        cand = next((b for b in reversed(blocks) if b["valid_syntax"]), None)
        cpu = self.rusage["cpu_s"] if self.rusage else self.sampled_cpu_s
        return {
            "call_id": self.id,
            **self.meta,
            "profile": self.profile,
            "call_status": self.status(),
            "argv": self.argv,
            "flags": list(self.flags),
            "pid": self.pid,
            "pgid": self.pgid,
            "cpu": self.cpu,
            "cpus_used": self.cpus_used,
            "affinity_observed": self.affinity,
            "state": self.state,
            "end_how": self.end_how,
            "returncode": self.rc,
            "created_elapsed_s": self.created_at,
            "started_elapsed_s": self.started_at,
            "ended_elapsed_s": self.ended_at,
            "active_wall_s": round(self.active_s, 4),
            "paused_wall_s": round(self.paused_s, 4),
            "queued_wall_s": round(self.queued_s, 4),
            "cpu_time_s": cpu,
            "cpu_time_source": (
                "wait4 rusage" if self.rusage else ("/proc sample" if self.sampled_cpu_s is not None else None)
            ),
            "rusage": self.rusage,
            "events": self.events,
            "blocks": blocks,
            "block_count": len(blocks),
            "first_block_at": blocks[0]["complete_at"] if blocks else None,
            "last_block_at": blocks[-1]["complete_at"] if blocks else None,
            "candidate_block": cand["index"] if cand else None,
            "candidate_metric": cand["metric"] if cand else None,
            "incomplete_block_at_end": self.incomplete_at_end or (cur is not None and bool(cur["lines"])),
            "metric_increase_observed": self.metric_increase,
            "stdout_lines": self.n_lines,
            "stdout_eof": self.eof,
            "partial_tail": self.partial_tail,
            "stderr_bytes": (self.dir / "stderr.txt").stat().st_size if (self.dir / "stderr.txt").exists() else 0,
        }


class PlannerPool:
    def __init__(self, run_dir, cpus, budget, binary, flags=("-n",), log=None):
        self.run_dir = Path(run_dir)
        self.calls_dir = self.run_dir / "calls"
        self.calls_dir.mkdir(exist_ok=True)
        self.cpus = list(cpus)
        self.budget = budget
        self.binary = str(binary)
        self.flags = tuple(flags)
        self.calls = []
        self.log_path = log or (self.run_dir / "planner_pool.jsonl")
        self._lock = threading.Lock()

    def event(self, **kw):
        with open(self.log_path, "a") as f:
            f.write(json.dumps({"t": round(self.budget.elapsed(), 4), **kw}) + "\n")

    def _write_call(self, call):
        tmp = call.dir / "call.json.tmp"
        tmp.write_text(json.dumps(call.record(), indent=1) + "\n")
        tmp.replace(call.dir / "call.json")

    def new_call(self, domain, problem, **meta):
        with self._lock:
            c = PlannerCall(self, len(self.calls), domain, problem, meta)
            self.calls.append(c)
        self.event(
            event="call_created", call=c.id, **{k: v for k, v in meta.items() if isinstance(v, (str, int, float, bool))}
        )
        self._write_call(c)
        return c

    # ---- process control ----
    def _spawn(self, call, cpu):
        argv = ["taskset", "-c", str(cpu), self.binary, *FLAGS[call.flags], call.domain, call.problem]
        err = open(call.dir / "stderr.txt", "wb")
        raw = open(call.dir / "stdout.txt", "wb")
        call.proc = subprocess.Popen(
            argv, cwd=self.run_dir, stdout=subprocess.PIPE, stderr=err, start_new_session=True, close_fds=True
        )
        err.close()
        call.pid = call.proc.pid
        call.pgid = call.pid
        call.cpu = cpu
        call.cpus_used.append(cpu)
        call.argv = argv
        call.started_at = self.budget.elapsed()
        call.quantum_start = time.monotonic()
        call.reader = threading.Thread(target=call._read, args=(call.proc.stdout.fileno(), raw), daemon=True)
        call.reader.start()
        call._transition("running", "start", {"cpu": cpu})
        self.event(event="start", call=call.id, pid=call.pid, cpu=cpu)

    def _reap(self, call, blocking):
        if call.rc is not None:
            return True
        try:
            pid, status, ru = os.wait4(call.pid, 0 if blocking else os.WNOHANG)
        except ChildProcessError:
            return False
        if pid == 0:
            return False
        call.rc = os.waitstatus_to_exitcode(status)
        call.proc.returncode = call.rc
        call.rusage = {
            "utime_s": ru.ru_utime,
            "stime_s": ru.ru_stime,
            "cpu_s": ru.ru_utime + ru.ru_stime,
            "maxrss_kb": ru.ru_maxrss,
        }
        call.reader.join(timeout=3.0)
        with call._lock:
            if call._cur is not None and call._cur["lines"]:
                if call.rc == 0:
                    call._close_block(call.eof_at if call.eof_at is not None else self.budget.elapsed(), "eof_exit0")
                else:
                    call.incomplete_at_end = True
            call._cur = None
        call.ended_at = self.budget.elapsed()
        return True

    def _pause(self, call, why):
        os.killpg(call.pgid, signal.SIGSTOP)
        st = None
        for _ in range(25):
            st = proc_state(call.pid)
            if st in ("T", "t", None, "Z"):
                break
            time.sleep(0.004)
        call._transition("paused", "pause", {"why": why, "proc_state": st})
        self.event(event="pause", call=call.id, pid=call.pid, why=why, state=st)

    def _resume(self, call, cpu, why):
        try:
            os.sched_setaffinity(call.pid, {cpu})
        except OSError as e:
            self.event(event="affinity_error", call=call.id, error=str(e))
        call.cpu = cpu
        if not call.cpus_used or call.cpus_used[-1] != cpu:
            call.cpus_used.append(cpu)
        os.killpg(call.pgid, signal.SIGCONT)
        call.quantum_start = time.monotonic()
        call._transition("running", "resume", {"cpu": cpu, "why": why})
        self.event(event="resume", call=call.id, pid=call.pid, cpu=cpu, why=why)

    def _kill(self, call, how):
        if call.rc is None:
            try:
                os.killpg(call.pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._reap(call, True)
        call.end_how = how
        call._transition("done", "kill", {"how": how, "rc": call.rc})
        self.event(event="kill", call=call.id, pid=call.pid, how=how, rc=call.rc)

    def _exited(self, call, phase):
        call.end_how = "exited_" + phase
        call._transition("done", "exit", {"rc": call.rc, "phase": phase, "blocks": len(call.blocks)})
        self.event(event="exit", call=call.id, pid=call.pid, rc=call.rc, phase=phase)

    def terminate(self, calls, how):
        for c in calls:
            if c.state in ("running", "paused"):
                self._kill(c, how)
            elif c.state == "queued":
                c.end_how = "never_started_" + how
                c._transition("done", "cancel", {"how": how})

    def terminate_all(self, how):
        self.terminate(self.calls, how)

    def _check_budget(self, calls):
        if self.budget.expired():
            self.terminate(calls, "killed_budget")
            raise BudgetExpired()

    def _observe_affinity(self, call):
        if call.affinity is None and call.pid and time.monotonic() - call.quantum_start > 0.02:
            call.affinity = proc_affinity(call.pid)

    # ---- phases ----
    def base_round(self, calls, base_s=BASE_S, max_active=MAX_ACTIVE):
        """Give every call its base opportunity (base_s of active time, at most max_active at once)."""
        pending = [c for c in calls if c.state == "queued"]
        active = []
        free = list(self.cpus)[:max_active]
        self.event(event="base_round_start", calls=[c.id for c in calls], base_s=base_s)
        while pending or active:
            self._check_budget(calls)
            for c in list(active):
                c._tick()
                self._observe_affinity(c)
                if self._reap(c, False):
                    self._exited(c, "base")
                    active.remove(c)
                    free.append(c.cpu)
                    continue
                if c.active_s >= base_s:
                    if c.has_block():
                        self._kill(c, "killed_after_base_with_plan")
                    else:
                        self._pause(c, "base_time_used_no_plan")
                    active.remove(c)
                    free.append(c.cpu)
            while pending and free:
                c = pending.pop(0)
                self._spawn(c, free.pop(0))
                active.append(c)
            time.sleep(POLL_S)
        for c in calls:
            c._tick()
        self.event(event="base_round_end", outcome={c.id: (c.state, c.end_how, len(c.blocks)) for c in calls})

    def extend(self, calls, quantum_s=QUANTUM_S, max_active=MAX_ACTIVE):
        """Continue preserved (paused) calls until the first complete block(s) arrive. Returns (arrivals, reason)."""
        waiting = [c for c in calls if c.state == "paused"]
        active = []
        free = list(self.cpus)[:max_active]
        mark = self.budget.elapsed()
        for c in waiting:
            c.ext_mark = mark
        self.event(event="extension_start", calls=[c.id for c in waiting], rotation=len(waiting) > max_active)
        if not waiting:
            return [], "none_preserved"
        while True:
            self._check_budget(calls)
            arrivals = []
            for c in list(active):
                c._tick()
                if self._reap(c, False):
                    self._exited(c, "extension")
                    active.remove(c)
                    free.append(c.cpu)
                    if c.has_block(mark):
                        arrivals.append(c)
                    continue
                if c.has_block(mark):
                    arrivals.append(c)
            if arrivals:
                for c in active:
                    if c not in arrivals:
                        self._pause(c, "other_call_delivered_candidate")
                for c in arrivals:
                    if c.state != "done":
                        self._kill(c, "killed_after_extension_plan")
                self.event(event="extension_arrival", calls=[c.id for c in arrivals])
                return arrivals, "candidate"
            if waiting and len(active) + len(waiting) > max_active:
                now = time.monotonic()
                for c in list(active):
                    if now - c.quantum_start >= quantum_s and waiting:
                        self._pause(c, "quantum_rotation")
                        active.remove(c)
                        free.append(c.cpu)
                        waiting.append(c)
            while waiting and len(active) < max_active and free:
                c = waiting.pop(0)
                self._resume(c, free.pop(0), "extension")
                active.append(c)
            if not active and not waiting:
                self.event(event="extension_exhausted")
                return [], "exhausted"
            time.sleep(POLL_S)

    def kill_all(self):
        self.terminate_all("killed_cleanup")

    def summary(self):
        return {
            "calls": len(self.calls),
            "active_wall_s": round(sum(c.active_s for c in self.calls), 3),
            "paused_wall_s": round(sum(c.paused_s for c in self.calls), 3),
            "queued_wall_s": round(sum(c.queued_s for c in self.calls), 3),
            "cpu_time_s": round(sum((c.rusage or {}).get("cpu_s", c.sampled_cpu_s or 0.0) for c in self.calls), 3),
            "unreaped": [c.id for c in self.calls if c.pid and c.rc is None],
            "max_rss_kb": max([(c.rusage or {}).get("maxrss_kb", 0) for c in self.calls] + [0]),
            "end_how": {str(c.id): c.end_how for c in self.calls},
            "metric_increase_calls": [c.id for c in self.calls if c.metric_increase],
        }
