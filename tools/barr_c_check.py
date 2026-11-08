#!/usr/bin/env python3
"""Heuristic checker for BARR-C:2018 rules that clang-format cannot enforce.

Usage: python tools/barr_c_check.py [files...]
Defaults to firmware/app_program/*.c|h and tests/*.c.  Prints one finding per
line as "file:line: [rule] message" and exits non-zero when anything is found.
It is a line-based approximation, not a parser; review each finding.
"""
import glob
import re
import sys

C_TYPES = (r"(?:u?int(?:8|16|32|64)_t|bool|int|unsigned|signed|char|float|"
           r"double|float32_t|float64_t|size_t|UINT|INT|UB|B|H|UH|W|UW|ER|ID|"
           r"TMO|SZ|RELTIM|SYSTIM|ATR|PRI|[a-z0-9_]+_t)")
DECL = re.compile(r"^\s*(?:static\s+|const\s+|volatile\s+|extern\s+)*" + C_TYPES +
                  r"(?:\s+const|\s+volatile)*\s*(\*\s*)?(?:const\s+)?"
                  r"([A-Za-z_]\w*)\s*(\[[^\]]*\])?\s*(=|;|,)")
FUNC_DEF = re.compile(r"^(?:static\s+)?(?:inline\s+)?[A-Za-z_][\w\s\*]*?\b"
                      r"([A-Za-z_]\w*)\s*\(([^;]*)$")
EQ_CONST_RIGHT = re.compile(
    r"[\w\)\]]\s*==\s*(-?\d[\w\.]*|[A-Z][A-Z0-9_]+\b|NULL\b|true\b|false\b|'.')")
FLOAT_NO_F = re.compile(r"(?<![\w\.])\d+\.\d*(?:[eE][-+]?\d+)?(?![\w\.])")
KEYWORDS = {"if", "while", "for", "switch", "return", "sizeof", "else"}


def strip_strings_and_comments(text):
    """Blank out comments and string/char literals, keeping line numbers."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(c + " " * (j - i - 1) + c)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def check_file(path, findings):
    raw = open(path, encoding="utf-8").read()
    code = strip_strings_and_comments(raw)
    lines = code.split("\n")
    raw_lines = raw.split("\n")

    def report(lineno, rule, msg):
        findings.append(f"{path}:{lineno}: [{rule}] {msg}")

    depth = 0
    func_start = None
    func_name = None
    switch_stack = []      # (depth at which switch body opens, has_default)
    pending_switch = False
    for idx, line in enumerate(lines, 1):
        stripped = line.strip()
        if re.search(r"\bgoto\b", line):
            report(idx, "1.7.c", "goto used")
        if re.search(r"\bcontinue\s*;", line):
            report(idx, "1.7.d", "continue used")
        if re.search(r"\b(short|long)\b", line):
            report(idx, "5.2.b", "short/long keyword")
        if re.search(r"\b(abort|exit|setjmp|longjmp)\s*\(", line):
            report(idx, "8.5.b", "forbidden library call")
        if EQ_CONST_RIGHT.search(line) and not stripped.startswith("#"):
            report(idx, "8.6.a", "constant on the right of ==")
        if not stripped.startswith("#"):
            for m in FLOAT_NO_F.finditer(line):
                report(idx, "5.4.b.ii", f"float constant {m.group(0)} lacks f")
        if re.match(r"^\s*#\s*define\s+\w+\(", line):
            report(idx, "6.3.a", "function-like macro")
        if re.search(r"\bfor\s*\([^;]*;\s*;\s*\)", line) is None and \
                re.search(r"\bwhile\s*\(\s*(1|true)\s*\)", line):
            report(idx, "8.4.c", "infinite loop not written as for (;;)")
        # Assignment inside an if test (single '=' not part of ==, <=, etc.).
        m = re.match(r"^\s*(?:else\s+)?if\s*\((.*)\)\s*$", line)
        if m and re.search(r"(?<![=!<>+\-*/%&|^])=(?!=)", m.group(1)):
            report(idx, "8.2.c", "assignment inside if test")

        # Comma-separated declarations (outside parameter lists).
        if depth >= 0 and DECL.match(line) and "(" not in line.split("=")[0]:
            body = line.split("=")[0]
            if "," in body and not stripped.endswith(","):
                report(idx, "8.1.a", "comma in variable declaration")

        # Variable naming checks on simple declarations.
        dm = DECL.match(line)
        if dm and "(" not in line.split("=")[0] and not stripped.startswith(
                ("return", "typedef", "case", "#")):
            name = dm.group(2)
            is_ptr = dm.group(1) is not None
            if name not in KEYWORDS:
                if re.search(r"[A-Z]", name):
                    report(idx, "7.1.f", f"variable '{name}' has uppercase")
                if len(name) < 3:
                    report(idx, "7.1.e", f"variable '{name}' shorter than 3")
                base = name[2:] if name.startswith("g_") else name
                if is_ptr and not base.startswith(("p_", "pp_")):
                    report(idx, "7.1.k", f"pointer '{name}' lacks p_ prefix")
                if re.search(r"\bbool\b", line.split(name)[0]) and \
                        not re.match(r"(g_)?(p_)?b_", name):
                    report(idx, "7.1.m", f"bool '{name}' lacks b_ prefix")
                if depth == 0 and "extern" not in line and \
                        not name.startswith("g_") and path.endswith(".c"):
                    report(idx, "7.1.j", f"file-scope '{name}' lacks g_ prefix")

        # for-loop counters.
        fm = re.search(r"\bfor\s*\(\s*(?:[\w\s]+\s)?([a-z]\w?)\s*=", line)
        if fm:
            report(idx, "7.1.e", f"loop counter '{fm.group(1)}' shorter than 3")

        # Switch / default tracking and function length.
        if re.search(r"\bswitch\s*\(", line):
            pending_switch = True
        for ch in line:
            if ch == "{":
                if depth == 0:
                    func_start = idx
                    # Function name is on this line or the line(s) above.
                    for back in range(idx - 1, max(0, idx - 6), -1):
                        fm2 = re.search(r"([A-Za-z_]\w*)\s*\(", lines[back - 1])
                        if fm2 and fm2.group(1) not in KEYWORDS:
                            func_name = fm2.group(1)
                            break
                depth += 1
                if pending_switch:
                    switch_stack.append([depth, False, idx])
                    pending_switch = False
            elif ch == "}":
                if switch_stack and switch_stack[-1][0] == depth:
                    sw = switch_stack.pop()
                    if not sw[1]:
                        report(sw[2], "8.3.b", "switch without default")
                depth -= 1
                if depth == 0 and func_start is not None:
                    length = idx - func_start + 1
                    if length > 100 and func_name:
                        report(func_start, "6.2.a",
                               f"function '{func_name}' is {length} lines")
                    func_start = None
                    func_name = None
        if re.match(r"^\s*default\s*:", line) and switch_stack:
            switch_stack[-1][1] = True

    # Function naming: no uppercase.
    for idx, line in enumerate(lines, 1):
        m = re.match(r"^(?:static\s+)?[A-Za-z_][\w\s\*]*?\b([A-Za-z_]\w*)\s*\(",
                     line)
        if m and not line.startswith((" ", "#")) and m.group(1) not in KEYWORDS:
            if re.search(r"[A-Z]", m.group(1)):
                report(idx, "6.1.e", f"function '{m.group(1)}' has uppercase")

    # else-if chains must end with else.
    for idx, line in enumerate(lines, 1):
        if re.match(r"^\s*else\s+if\b", line):
            indent = len(line) - len(line.lstrip())
            j = idx
            # Walk to the closing brace of this else-if block at same indent.
            while j < len(lines):
                nxt = lines[j]
                if nxt.strip() == "}" and \
                        len(nxt) - len(nxt.lstrip()) == indent:
                    after = lines[j + 1] if j + 1 < len(lines) else ""
                    if re.match(r"^\s*else\s+if\b", after):
                        break
                    if not re.match(r"^\s*else\b", after):
                        report(idx, "8.2.d", "else-if chain without final else")
                    break
                j += 1

    # Header guard and end-of-file marker.
    if path.endswith(".h") and not re.search(r"#ifndef\s+\w+_H\b", raw):
        report(1, "4.2.b", "missing include guard")
    tail = [l for l in raw_lines if l.strip()]
    if not tail or "end of file" not in tail[-1]:
        report(len(raw_lines), "3.3.c", "missing end-of-file comment")
    if raw and not raw.endswith("\n"):
        report(len(raw_lines), "3.3.c", "file does not end with a newline")


def main(argv):
    files = argv or sorted(glob.glob("firmware/app_program/*.[ch]") +
                           glob.glob("tests/*.c"))
    findings = []
    for path in files:
        check_file(path, findings)
    for f in findings:
        print(f)
    rules = {}
    for f in findings:
        rule = f.split("[", 1)[1].split("]", 1)[0]
        rules[rule] = rules.get(rule, 0) + 1
    print("\nSummary:", ", ".join(f"{k}={v}" for k, v in sorted(rules.items())),
          file=sys.stderr)
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
