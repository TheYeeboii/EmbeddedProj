#!/usr/bin/env python3
"""Mechanical BARR-C:2018 fixes that clang-format cannot make.

Run after clang-format (tools/barr_c_format.sh does both):
  8.6.a  constant placed on the left of == (only when precedence is safe)
  8.4.c  while (1) / while (true) rewritten as for (;;)
  5.4.b  f suffix on floating-point constants (the project uses no double)
  8.3.a  break aligned with its case label
  3.3.c  end-of-file comment followed by a blank line
"""
import glob
import re
import sys

EOF_MARKER = "/*** end of file ***/"

LVALUE = r"[A-Za-z_]\w*(?:(?:\.|->)[A-Za-z_]\w*|\[[^\[\]()]*\])*"
CONST = r"-?\d[\w.]*|[A-Z][A-Z0-9_]*[A-Z0-9]\b|NULL\b|true\b|false\b|'(?:\\.|[^'])'"
# Left operand must start right after a token that binds looser than ==.
EQ = re.compile(r"(?P<pre>(?:^|[(,?:]|&&|\|\||(?<![=!<>])=(?!=)|\breturn\b)\s*)"
                r"(?P<lhs>" + LVALUE + r")\s*==\s*(?P<rhs>" + CONST + r")"
                r"(?=\s*(?:\)|&&|\|\||;|\?|,|:|$))", re.M)
FLOAT = re.compile(r"(?<![\w.])(\d+\.\d*|\.\d+)(?:([eE][-+]?\d+))?(?![\w.])")
WHILE_TRUE = re.compile(r"\bwhile\s*\(\s*(?:1|true)\s*\)")


def code_mask(text):
    """Return a list of booleans: True where text[i] is real code."""
    mask = [True] * len(text)
    i, n = 0, len(text)
    while i < n:
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
        elif text[i] in "\"'":
            q = text[i]
            j = i + 1
            while j < n and text[j] != q:
                j += 2 if text[j] == "\\" else 1
            j += 1
        else:
            i += 1
            continue
        for k in range(i, min(j, n)):
            mask[k] = False
        i = j
    return mask


def in_code(mask, start, end):
    return all(mask[start:end])


def sub_in_code(pattern, repl, text, skip_directives=False):
    mask = code_mask(text)
    out = []
    last = 0
    for m in pattern.finditer(text):
        if not in_code(mask, m.start(), m.end()):
            continue
        line_start = text.rfind("\n", 0, m.start()) + 1
        line = text[line_start:text.find("\n", m.start())]
        if skip_directives and line.lstrip().startswith("#"):
            continue
        if line.lstrip().startswith("#include"):
            continue
        out.append(text[last:m.start()])
        out.append(repl(m))
        last = m.end()
    out.append(text[last:])
    return "".join(out)


def swap_equality(text):
    # Repeat until stable: one pass may expose a second comparison.
    for _ in range(4):
        new = sub_in_code(
            EQ, lambda m: f"{m.group('pre')}{m.group('rhs')} == {m.group('lhs')}",
            text, skip_directives=True)
        if new == text:
            break
        text = new
    return text


def add_float_suffix(text):
    def repl(m):
        mantissa = m.group(1)
        if mantissa.endswith("."):
            mantissa += "0"
        if mantissa.startswith("."):
            mantissa = "0" + mantissa
        return mantissa + (m.group(2) or "") + "f"
    return sub_in_code(FLOAT, repl, text)


def align_breaks(text):
    lines = text.split("\n")
    label_indent = None
    for idx, line in enumerate(lines):
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if re.match(r"(case\b.*|default\s*):", stripped):
            label_indent = indent
        elif stripped == "break;" and label_indent is not None and \
                indent == label_indent + 4:
            lines[idx] = " " * label_indent + "break;"
        elif stripped and label_indent is not None and indent <= label_indent \
                and not stripped.startswith(("case", "default", "break")):
            label_indent = None
    return "\n".join(lines)


def add_eof_marker(text):
    body = text.rstrip("\n")
    if not body.rstrip().endswith(EOF_MARKER):
        body = body + "\n\n" + EOF_MARKER
    return body + "\n\n"


def fix(text):
    text = swap_equality(text)
    text = sub_in_code(WHILE_TRUE, lambda m: "for (;;)", text)
    text = add_float_suffix(text)
    text = align_breaks(text)
    text = add_eof_marker(text)
    return text


def main(argv):
    files = argv or sorted(glob.glob("firmware/app_program/*.[ch]") +
                           glob.glob("tests/*.c"))
    for path in files:
        with open(path, encoding="utf-8", newline="") as handle:
            old = handle.read().replace("\r\n", "\n")
        new = fix(old)
        if new != old:
            with open(path, "w", encoding="utf-8", newline="\n") as handle:
                handle.write(new)
            print("fixed", path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
