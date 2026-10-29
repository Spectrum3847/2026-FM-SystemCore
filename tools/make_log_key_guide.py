#!/usr/bin/env python3
"""Generates the log key guide: src/main/deploy/log-keys.json and docs/log-keys.md.

    python tools/make_log_key_guide.py           # write both
    python tools/make_log_key_guide.py --check   # fail if either is stale (CI)

Reads the Java source for every key the robot logs (Telemetry.log/logDash/logEvent,
Logger.recordOutput, and the table.put calls in each LoggableInputs class with the
processInputs prefix it is logged under), its unit, and the comment written next to it. The
robot records the JSON in every log's metadata as LogKeyGuide, so a person or an agent holding
only a .wpilog can find what a key means.

A key built at runtime shows its runtime parts as <name>, e.g. Mechanisms/<name>/Voltage.
ponytail: regex over Java, not a parser. A call it can't read is left out; good enough while
keys are mostly literals or prefix + "/Literal".
"""

import json
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SRC = os.path.join(ROOT, "src", "main", "java")
JSON_OUT = os.path.join(ROOT, "src", "main", "deploy", "log-keys.json")
MD_OUT = os.path.join(ROOT, "docs", "log-keys.md")

OUTPUT_CALL = re.compile(
    r"\b(?:Telemetry\.(?:log|logDash|logDashAlways|logEvent)|Logger\.recordOutput|logEvent)\(")
PUT_CALL = re.compile(r"\b(\w+)\.put\(")
SUBTABLE = re.compile(r"\b(\w+)\s*=\s*\w+\.getSubtable\(([^;]+)\);")
PROCESS = re.compile(r"Logger\.processInputs\(")

# What AdvantageKit logs by itself, which no source here names.
ADVANTAGEKIT = [
    ("/DriverStation/*", "", "Driver Station and match data: enabled, mode, alliance, event, match number and type, joysticks (AdvantageKit)"),
    ("/SystemStats/*", "", "Controller CPU, memory, storage, network, CAN bus, IMU, faults, battery, NT clients and clock validity (AdvantageKit)"),
    ("/RadioStatus/*", "", "Radio status JSON (AdvantageKit; Wi-Fi keys redacted by RadioStatusRedactor)"),
    ("/RealOutputs/LoggedRobot/*", "ms", "Loop timing: FullCycleMS, UserCodeMS, LogPeriodicMS, GC time (AdvantageKit)"),
    ("/RealOutputs/Console", "", "Everything printed to the console (AdvantageKit)"),
    ("/RealMetadata/*", "", "Build and robot identity: GitSHA, GitBranch, BuildDate, RobotIdentity, BootId, LogKeyGuide"),
    ("/Timestamp", "us", "The cycle's robot time (AdvantageKit)"),
    ("/NetworkInputs/*", "", "Values read from the dashboard over NetworkTables (LoggedNetwork*), recorded so replay sees what the robot saw"),
    ("/Params/*", "", "Tuned values from src/main/deploy/params/*.json and their dashboard overrides, by group and name (Params)"),
]


def split_args(text, start):
    """Arguments of the call whose '(' is just before start, and the index after its ')'."""
    depth, args, cur, i, quote = 0, [], "", start, None
    while i < len(text):
        c = text[i]
        if quote:
            cur += c
            if c == "\\":
                cur += text[i + 1]
                i += 1
            elif c == quote:
                quote = None
        elif c in "\"'":
            quote = c
            cur += c
        elif c in "([{":
            depth += 1
            cur += c
        elif c in ")]}":
            if depth == 0:
                args.append(cur.strip())
                return args, i + 1
            depth -= 1
            cur += c
        elif c == "," and depth == 0:
            args.append(cur.strip())
            cur = ""
        else:
            cur += c
        i += 1
    return None, i


def render(expr):
    """A key expression as text: literals kept, anything computed as <name>."""
    parts, depth, cur, quote = [], 0, "", None
    for c in expr:
        if quote:
            cur += c
            if c == quote:
                quote = None
        elif c == '"':
            quote = c
            cur += c
        elif c in "([":
            depth += 1
            cur += c
        elif c in ")]":
            depth -= 1
            cur += c
        elif c == "+" and depth == 0:
            parts.append(cur.strip())
            cur = ""
        else:
            cur += c
    parts.append(cur.strip())
    out = ""
    for p in parts:
        if re.fullmatch(r'"(?:[^"\\]|\\.)*"', p):
            out += p[1:-1]
        else:
            words = re.findall(r"[A-Za-z_]\w*", p.split("(")[0])
            name = words[-1] if words else "x"
            name = re.sub(r"^(get|config)", "", name)
            out += "<" + (name[:1].lower() + name[1:] if name else "x") + ">"
    return out


def comment_above(lines, idx):
    """The // or /** */ comment directly above line idx, as one line of text."""
    out, j = [], idx - 1
    while j >= 0 and lines[j].strip().startswith("//"):
        out.insert(0, lines[j].strip()[2:].strip())
        j -= 1
    if not out and j >= 0 and lines[j].strip().endswith("*/"):
        k = j
        while k >= 0 and "/**" not in lines[k] and "/*" not in lines[k]:
            k -= 1
        for l in lines[k:j + 1]:
            t = l.strip().lstrip("/*").rstrip("*/").strip()
            if t and not t.startswith("@"):
                out.append(t)
    text = " ".join(out)
    text = re.sub(r"\{@(?:code|link|linkplain) #?([^}]+)\}", r"\1", text)
    return re.sub(r"\s+", " ", text).strip()


def line_of(text, pos):
    return text.count("\n", 0, pos)


def unit_of(args):
    if len(args) >= 3 and re.fullmatch(r'"[^"]*"', args[-1]):
        return args[-1][1:-1]
    return ""


def java_files():
    for d, _, fs in os.walk(SRC):
        for f in sorted(fs):
            if f.endswith(".java"):
                yield os.path.join(d, f)


def outputs(path, text, lines):
    rel = os.path.relpath(path, ROOT).replace(os.sep, "/")
    for m in OUTPUT_CALL.finditer(text):
        if text[:m.start()].rstrip().endswith(("void", "static")):
            continue  # a declaration, not a call
        args, _ = split_args(text, m.end())
        if not args or not args[0] or args[0] in ("key", "k", "name"):
            continue  # the helper itself, forwarding its parameter
        key = render(args[0])
        n = line_of(text, m.start())
        yield {
            "key": "/RealOutputs/" + key.lstrip("/"),
            "unit": unit_of(args),
            "doc": comment_above(lines, n),
            "source": rel,
        }


def field_docs(text, lines):
    """Comment above each field declaration, by field name."""
    docs = {}
    decl = (r"^\s*(?:@\w+\s+)*(?:public|private|protected)\s+(?:static\s+)?(?:final\s+)?"
            r"[\w<>\[\],.]+(?:\s*\[\])*\s+(\w+)\s*(?:=[^;]*)?;")
    for m in re.finditer(decl, text, re.M):
        docs[m.group(1)] = comment_above(lines, line_of(text, m.start()))
    return docs


def input_classes():
    """Every LoggableInputs class: its toLog keys, by class name."""
    classes = {}
    for path in java_files():
        text = open(path, encoding="utf-8").read()
        for cm in re.finditer(r"class (\w+)[^{]*implements[^{]*LoggableInputs", text):
            name = cm.group(1)
            lines = text.split("\n")
            docs = field_docs(text, lines)
            start = text.find("void toLog(", cm.end())
            if start < 0:
                continue
            body_start = text.index("{", start)
            depth, i = 0, body_start
            while True:
                depth += {"{": 1, "}": -1}.get(text[i], 0)
                i += 1
                if depth == 0:
                    break
            body = text[body_start:i]
            subs = {v: render(e) + "/" for v, e in SUBTABLE.findall(body)}
            keys = []
            for pm in PUT_CALL.finditer(body):
                args, _ = split_args(body, pm.end())
                if not args or len(args) < 2 or not args[0].startswith('"'):
                    continue
                field = re.findall(r"\w+", args[1])
                field = field[-1] if field else ""
                n = line_of(text, body_start + pm.start())
                keys.append({
                    "key": subs.get(pm.group(1), "") + render(args[0]),
                    "doc": docs.get(field, ""),
                    "source": os.path.relpath(path, ROOT).replace(os.sep, "/"),
                })
            classes[name] = keys
    return classes


def inputs(classes):
    for path in java_files():
        text = open(path, encoding="utf-8").read()
        for m in PROCESS.finditer(text):
            args, _ = split_args(text, m.end())
            if not args or len(args) != 2:
                continue
            var = args[1].split(".")[-1]
            prefix_expr = args[0]
            # A prefix held in a variable: use what it is assigned.
            if re.fullmatch(r"\w+", prefix_expr):
                assigned = re.search(r"\b" + prefix_expr + r"\s*=\s*([^;=][^;]*);", text)
                if assigned:
                    prefix_expr = assigned.group(1)
            prefix = render(prefix_expr)
            types = re.findall(r"\b(\w+)\s+" + re.escape(var) + r"\s*(?:=|;)", text)
            types += re.findall(r"\b" + re.escape(var) + r"\s*=\s*new\s+(\w+)", text)
            cls = next((t for t in types if t in classes), None)
            if cls is None:
                continue
            for k in classes[cls]:
                yield {
                    "key": "/" + prefix.strip("/") + "/" + k["key"],
                    "unit": "",
                    "doc": k["doc"],
                    "source": k["source"],
                    "input": True,
                }


def add(entries, e):
    """First place a key is logged, with the first comment found for it anywhere."""
    have = entries.setdefault(e["key"], e)
    if not have["doc"] and e["doc"]:
        have["doc"] = e["doc"]


def build():
    entries = {}
    for path in java_files():
        text = open(path, encoding="utf-8").read()
        for e in outputs(path, text, text.split("\n")):
            add(entries, e)
    for e in inputs(input_classes()):
        add(entries, e)
    keys = sorted(entries.values(), key=lambda e: e["key"].lower())
    for e in keys:
        e.setdefault("input", False)
    return {
        "about": "What each log key means. Inputs are recorded readings, replayed as-is; outputs are "
        "computed, and replay recomputes them. <name> is filled in at runtime. Generated by "
        "tools/make_log_key_guide.py; do not edit.",
        "advantageKit": [{"key": k, "unit": u, "doc": d} for k, u, d in ADVANTAGEKIT],
        "keys": keys,
    }


def markdown(guide):
    out = [
        "# Log keys",
        "",
        "Generated by `tools/make_log_key_guide.py` from the comments next to each logged value; edit "
        "those, not this file. Every log also carries this list as the `LogKeyGuide` metadata.",
        "",
        "Inputs (I) are recorded readings that replay feeds back as-is; outputs are computed and "
        "replay recomputes them. `<name>` is filled in at runtime.",
        "",
        "## Logged by AdvantageKit",
        "",
        "| Key | Meaning |",
        "|---|---|",
    ]
    out += [f"| `{e['key']}` | {e['doc']} |" for e in guide["advantageKit"]]
    out += ["", "## Logged by robot code", "", "| Key | | Unit | Meaning | Source |", "|---|---|---|---|---|"]
    for e in guide["keys"]:
        doc = e["doc"].replace("|", "\\|")
        out.append(f"| `{e['key']}` | {'I' if e['input'] else ''} | {e['unit']} | {doc} | {e['source']} |")
    return "\n".join(out) + "\n"


def main():
    guide = build()
    files = {
        JSON_OUT: json.dumps(guide, indent=1, ensure_ascii=False) + "\n",
        MD_OUT: markdown(guide),
    }
    if "--check" in sys.argv:
        stale = [p for p, s in files.items()
                 if not os.path.exists(p) or open(p, encoding="utf-8").read() != s]
        for p in stale:
            print(f"stale: {os.path.relpath(p, ROOT)}; run python tools/make_log_key_guide.py")
        sys.exit(1 if stale else 0)
    for p, s in files.items():
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            f.write(s)
    print(f"{len(guide['keys'])} keys ({sum(e['input'] for e in guide['keys'])} inputs)")


if __name__ == "__main__":
    main()
