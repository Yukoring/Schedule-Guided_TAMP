"""U_MATCH diversification (REFERENCE_v2 section 2): semantics-preserving reordering of a PDDL problem/domain by a deterministic
permutation derived from (diversification seed, slot_role, release, iteration, attempt) with sha256 (never Python hash).

Problem file: within:objects, the names of each type group are permuted (groups of the same type are merged); within:init,
the order-independent plain facts are permuted among themselves while numeric initialisations '(= ...)' and timed initial
literals '(at <number> ...)' keep their original relative order and positions;:goal,:metric and everything else are untouched.
Domain file: the top-level (:durative-action ...)/(:action ...) declarations are permuted among themselves; types, predicates,
functions and action bodies are untouched. No names, numbers, conditions, effects, durations or goals are changed.
check_equivalent compares the normalised ASTs (sorted objects per type, sorted init facts, sorted actions) of two files."""

import hashlib, random, re
from pathlib import Path

DIV_SEED = 0


def tokenize(text):
    text = re.sub(r";[^\n]*", "", text)
    return re.findall(r"\(|\)|[^\s()]+", text)


def parse(tokens):
    stack = [[]]
    for t in tokens:
        if t == "(":
            stack.append([])
        elif t == ")":
            node = stack.pop()
            stack[-1].append(node)
        else:
            stack[-1].append(t)
    assert len(stack) == 1, "unbalanced parentheses"
    return stack[0]


def emit(node, indent=0):
    if isinstance(node, str):
        return node
    if not node:
        return "()"
    head = node[0]
    if isinstance(head, str) and head in (":objects", ":init", ":goal", "define"):
        inner = " ".join(
            emit(x, indent + 1) if isinstance(x, str) else "\n" + "  " * (indent + 1) + emit(x, indent + 1)
            for x in node[1:]
        )
        return "(" + head + " " + inner + "\n" + "  " * indent + ")"
    return "(" + " ".join(emit(x, indent + 1) for x in node) + ")"


def rng_for(key):
    return random.Random(int(hashlib.sha256(key.encode()).hexdigest()[:16], 16))


def perm_key(slot_role, release, iteration, attempt, kind, div_seed=DIV_SEED):
    return f"div{div_seed}|{slot_role}|{release}|it{iteration}|att{attempt}|{kind}"


def _objects_groups(items):
    """[(names,type)] from the flat:objects token list 'a b - t c - t2 ...'"""
    groups = []
    cur = []
    i = 0
    while i < len(items):
        if items[i] == "-":
            groups.append((cur, items[i + 1]))
            cur = []
            i += 2
        else:
            cur.append(items[i])
            i += 1
    if cur:
        groups.append((cur, None))
    return groups


def is_numeric_init(f):
    return isinstance(f, list) and f and f[0] == "="


def is_til(f):
    return (
        isinstance(f, list)
        and len(f) >= 2
        and f[0] == "at"
        and isinstance(f[1], str)
        and re.fullmatch(r"[0-9.]+", f[1]) is not None
    )


def permute_problem(src, dst, key):
    ast = parse(tokenize(Path(src).read_text()))
    root = ast[0]
    rng = rng_for(key)
    for sec in root:
        if not isinstance(sec, list) or not sec:
            continue
        if sec[0] == ":objects":
            groups = _objects_groups(sec[1:])
            by_type = {}
            for names, typ in groups:
                by_type.setdefault(typ, []).extend(names)
            new = []
            for typ, names in by_type.items():
                names = list(names)
                rng.shuffle(names)
                new += names + (["-", typ] if typ else [])
            sec[1:] = new
        elif sec[0] == ":init":
            facts = sec[1:]
            plain_idx = [i for i, f in enumerate(facts) if not (is_numeric_init(f) or is_til(f))]
            plain = [facts[i] for i in plain_idx]
            rng.shuffle(plain)
            for i, f in zip(plain_idx, plain):
                facts[i] = f
            sec[1:] = facts
    Path(dst).write_text(emit(root) + "\n")
    return hashlib.sha256(Path(dst).read_bytes()).hexdigest()


def permute_domain(src, dst, key):
    ast = parse(tokenize(Path(src).read_text()))
    root = ast[0]
    rng = rng_for(key)
    idx = [i for i, s in enumerate(root) if isinstance(s, list) and s and s[0] in (":durative-action", ":action")]
    acts = [root[i] for i in idx]
    rng.shuffle(acts)
    for i, a in zip(idx, acts):
        root[i] = a
    Path(dst).write_text(emit(root) + "\n")
    return hashlib.sha256(Path(dst).read_bytes()).hexdigest()


def _norm(node):
    if isinstance(node, str):
        return node
    return "(" + " ".join(_norm(x) for x in node) + ")"


def canonical(path):
    """normalised, order-independent representation of a problem or domain file"""
    root = parse(tokenize(Path(path).read_text()))[0]
    out = []
    for sec in root:
        if not isinstance(sec, list) or not sec:
            out.append(_norm(sec))
            continue
        if sec[0] == ":objects":
            by_type = {}
            for names, typ in _objects_groups(sec[1:]):
                by_type.setdefault(typ, []).extend(names)
            out.append(
                "OBJECTS "
                + " ".join(
                    "%s:%s" % (t, ",".join(sorted(n))) for t, n in sorted(by_type.items(), key=lambda kv: str(kv[0]))
                )
            )
        elif sec[0] == ":init":
            facts = sec[1:]
            out.append(
                "INIT-PLAIN " + " ".join(sorted(_norm(f) for f in facts if not (is_numeric_init(f) or is_til(f))))
            )
            out.append("INIT-ORDERED " + " ".join(_norm(f) for f in facts if is_numeric_init(f) or is_til(f)))
        elif sec[0] in (":durative-action", ":action"):
            out.append("ACTION " + _norm(sec))
        else:
            out.append(_norm(sec))
    acts = sorted(x for x in out if x.startswith("ACTION "))
    rest = [x for x in out if not x.startswith("ACTION ")]
    return "\n".join(rest + acts)


def check_equivalent(a, b):
    ca, cb = canonical(a), canonical(b)
    return {
        "equivalent": ca == cb,
        "canonical_sha256_a": hashlib.sha256(ca.encode()).hexdigest(),
        "canonical_sha256_b": hashlib.sha256(cb.encode()).hexdigest(),
        "file_sha256_a": hashlib.sha256(Path(a).read_bytes()).hexdigest(),
        "file_sha256_b": hashlib.sha256(Path(b).read_bytes()).hexdigest(),
    }


if __name__ == "__main__":
    import sys, json

    src, dst, key = sys.argv[1:4]
    f = permute_domain if "domain" in Path(src).name else permute_problem
    f(src, dst, key)
    print(json.dumps(check_equivalent(src, dst)))
