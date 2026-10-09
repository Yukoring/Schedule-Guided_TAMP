"""Replay the emitted native temporal PDDL subset, including tools and TILs."""

import itertools
import math
import re
from collections import defaultdict


def parse(text):
    text = re.sub(r";[^\n]*", "", text).lower()
    stack = []
    root = None
    for t in re.findall(r"\(|\)|[^\s()]+", text):
        if t == "(":
            x = []
            if stack:
                stack[-1].append(x)
            stack.append(x)
        elif t == ")":
            root = stack.pop()
        else:
            stack[-1].append(t)
    assert not stack
    return root


def emit(x):
    return "(" + " ".join(emit(y) if isinstance(y, list) else str(y) for y in x) + ")"


def typed(seq):
    out = []
    pending = []
    i = 0
    while i < len(seq):
        if seq[i] == "-":
            out.extend((x, seq[i + 1]) for x in pending)
            pending = []
            i += 2
        else:
            pending.append(seq[i])
            i += 1
    return out + [(x, "object") for x in pending]


def section(root, key):
    return next(x for x in root if isinstance(x, list) and x and x[0] == key)


def ground(x, bind):
    return [ground(t, bind) for t in x] if isinstance(x, list) else bind.get(x, x)


def conjuncts(x):
    return x[1:] if x and x[0] == "and" else [x]


class Replay:
    def __init__(self, domain, problem):
        self.d = parse(domain)
        self.p = parse(problem)
        self.actions = {}
        for x in self.d:
            if isinstance(x, list) and x and x[0] == ":durative-action":
                self.actions[x[1]] = {x[i]: x[i + 1] for i in range(2, len(x), 2)}
        self.parents = dict(typed(section(self.d, ":types")[1:]))
        self.objects = defaultdict(list)
        for obj, t in typed(section(self.p, ":objects")[1:]):
            seen = set()
            while t not in seen:
                self.objects[t].append(obj)
                seen.add(t)
                if t == "object":
                    break
                t = self.parents.get(t, "object")
        self.initial = set()
        self.numbers = {}
        self.tils = []
        for x in section(self.p, ":init")[1:]:
            if x[0] == "=":
                self.numbers[tuple(x[1])] = float(x[2])
            elif x[0] == "at" and len(x) == 3 and re.fullmatch(r"[\d.]+", x[1]):
                self.tils.append((float(x[1]), x[2]))
            else:
                self.initial.add(tuple(x))

    def numeric(self, x):
        if not isinstance(x, list):
            return float(x)
        if x[0] == "/":
            return self.numeric(x[1]) / self.numeric(x[2])
        if x[0] == "*":
            return self.numeric(x[1]) * self.numeric(x[2])
        if x[0] == "+":
            return self.numeric(x[1]) + self.numeric(x[2])
        if x[0] == "-":
            return self.numeric(x[1]) - self.numeric(x[2])
        return self.numbers[tuple(x)]

    def truth(self, x, state):
        if x[0] == "and":
            return all(self.truth(y, state) for y in x[1:])
        if x[0] == "or":
            return any(self.truth(y, state) for y in x[1:])
        if x[0] == "not":
            return not self.truth(x[1], state)
        if x[0] == "imply":
            return not self.truth(x[1], state) or self.truth(x[2], state)
        if x[0] == "forall":
            pairs = typed(x[1])
            return all(
                self.truth(ground(x[2], dict(zip([a for a, t in pairs], combo))), state)
                for combo in itertools.product(*(self.objects[t] for a, t in pairs))
            )
        return tuple(x) in state

    def reads(self, x, state):
        if x[0] in ["and", "or"]:
            return set().union(*(self.reads(y, state) for y in x[1:]))
        if x[0] == "not":
            return self.reads(x[1], state)
        if x[0] == "imply":
            return self.reads(x[1], state) | (self.reads(x[2], state) if self.truth(x[1], state) else set())
        if x[0] == "forall":
            pairs = typed(x[1])
            out = set()
            for combo in itertools.product(*(self.objects[t] for a, t in pairs)):
                out |= self.reads(ground(x[2], dict(zip([a for a, t in pairs], combo))), state)
            return out
        return {tuple(x)}

    def check(self, plan):
        events = defaultdict(list)
        errors = []
        defs = {}
        active = {}
        state = set(self.initial)
        for t, x in self.tils:
            events[round(t, 6)].append(("til", x))
        for i, a in enumerate(plan):
            if a["name"] not in self.actions:
                errors.append("UNKNOWN_ACTION")
                continue
            raw = self.actions[a["name"]]
            pairs = typed(raw[":parameters"])
            if len(a["args"]) != len(pairs):
                errors.append("ARGUMENT_COUNT")
                continue
            bind = dict(zip([x for x, t in pairs], a["args"]))
            spec = {k: ground(v, bind) for k, v in raw.items()}
            defs[i] = spec
            expected = self.numeric(spec[":duration"][2])
            if abs(expected - a["duration"]) > 0.00101:
                errors.append("ACTION_DURATION")
            if a["duration"] <= 0 or a["start"] < 0:
                errors.append("ACTION_TIME")
            events[round(a["start"], 6)].append(("start", i))
            events[round(a["start"] + a["duration"], 6)].append(("end", i))
        for t, evs in sorted(events.items()):
            for i in active:
                for x in conjuncts(defs[i][":condition"]):
                    if x[:2] == ["over", "all"] and not self.truth(x[2], state):
                        errors.append("INVARIANT_BEFORE")
            updates = []
            changes = []
            for kind, x in evs:
                if kind == "til":
                    updates.append(x)
                    changes.append((("til", str(x)), set(), {tuple(x[1]) if x[0] == "not" else tuple(x)}))
                    continue
                if x not in defs:
                    continue
                reads = set()
                writes = set()
                for c in conjuncts(defs[x][":condition"]):
                    if c[:2] == ["at", kind]:
                        reads |= self.reads(c[2], state)
                        if not self.truth(c[2], state):
                            errors.append("PRECONDITION_" + kind.upper())
                for ef in conjuncts(defs[x][":effect"]):
                    assert ef[0] == "at"
                    if ef[1] == kind:
                        updates.append(ef[2])
                        writes.add(tuple(ef[2][1]) if ef[2][0] == "not" else tuple(ef[2]))
                changes.append((x, reads, writes))
            for i, (owner, reads, writes) in enumerate(changes):
                for other, reads2, writes2 in changes[i + 1 :]:
                    if owner != other and (reads & writes2 or reads2 & writes):
                        errors.append("HAPPENING_MUTEX")
            deletes = {tuple(x[1]) for x in updates if x[0] == "not"}
            adds = {tuple(x) for x in updates if x[0] != "not"}
            if deletes & adds:
                errors.append("CONFLICTING_EFFECTS")
            state -= deletes
            state |= adds
            for kind, x in evs:
                if kind == "start" and x in defs:
                    active[x] = True
                elif kind == "end":
                    active.pop(x, None)
            for i in active:
                for x in conjuncts(defs[i][":condition"]):
                    if x[:2] == ["over", "all"] and not self.truth(x[2], state):
                        errors.append("INVARIANT_AFTER")
        if not self.truth(section(self.p, ":goal")[1], state):
            errors.append("GOAL")
        return sorted(set(errors))


def plans_from_stdout(text):
    blocks = re.split(r"; (?:Plan found|Solution Found)[^\n]*\n", text)[1:]
    plans = []
    for block in blocks:
        actions = []
        for m in re.finditer(r"^\s*([0-9.]+):\s*\(([^)]+)\)\s*\[([0-9.]+)\]", block, re.M):
            names = m[2].lower().split()
            actions.append({"name": names[0], "args": names[1:], "start": float(m[1]), "duration": float(m[3])})
        if actions:
            plans.append(sorted(actions, key=lambda a: (a["start"], a["args"][0], a["name"])))
    return plans
