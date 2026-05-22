#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""validate_xml_comments_and_parse.py  --  permanent V23+ pipeline check.

REASON THIS SCRIPT EXISTS
-------------------------
V23 shipped with three illegal XML constructs that caused
"not well-formed (invalid token)" on Ubuntu/xacro:
  * schneider_cell.urdf.xacro line 17:  '--' inside an outer comment
  * fixture_rivet.xacro       line 18:  '--' inside an outer comment
  * new_gripper.xacro         line 21:  nested '<!-- grasp_center -->'
                                         inside an outer comment
XML 1.0 forbids the substring '--' anywhere inside a <!-- ... --> body.
This script enforces that rule + a full XML well-formedness parse on
EVERY .xacro / .urdf / .launch / .xml in the workspace.

It MUST be run before any zip ships.  V23 fixXML / V24 onwards include
this script in the package and the V23 aggregate validator
(`validate_v23.py`) chains to it.

USAGE
-----
  # Default: walk the schneider_cell ros_ws from this script's location.
  python validate_xml_comments_and_parse.py

  # Override the root:
  python validate_xml_comments_and_parse.py /path/to/ros_ws

EXIT CODE
---------
  0   all checks passed
  1   at least one illegal '--' inside a comment OR one XML parse error

OUTPUT
------
  ERROR: Illegal XML comment token '--' found in <file>, line <N>
        > <the offending source line>
  XML PARSE ERROR (<file>, line N, col M): <python xml.etree message>
"""
from __future__ import print_function
import os
import sys
import xml.etree.ElementTree as ET

DEFAULT_ROOTS = [
    # When the script lives inside the description package, walk three
    # levels up to reach the ros_ws/src directory.
    os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..")),
]
EXTS = (".xacro", ".urdf", ".launch", ".xml")


def line_of(text, idx):
    return text.count("\n", 0, idx) + 1


def line_text(text, idx):
    a = text.rfind("\n", 0, idx) + 1
    b = text.find("\n", idx)
    if b == -1:
        b = len(text)
    return text[a:b].rstrip()


def scan_comments(text):
    """Walk each `<!--` opener in text.  For each one, find the matching
    `-->` and inspect the body for any illegal `--` substring.  Returns
    a list of (line_no, message, offending_line) tuples.
    """
    out = []
    i = 0
    while True:
        a = text.find("<!--", i)
        if a == -1:
            break
        b = text.find("-->", a + 4)
        if b == -1:
            out.append((line_of(text, a),
                        "<!-- never closed",
                        line_text(text, a)))
            break
        body = text[a + 4 : b]
        j = 0
        while True:
            k = body.find("--", j)
            if k == -1:
                break
            abs_idx = a + 4 + k
            out.append((line_of(text, abs_idx),
                        "illegal '--' inside XML comment",
                        line_text(text, abs_idx)))
            j = k + 2
        i = b + 3
    return out


def parse_xml(path):
    try:
        ET.parse(path)
        return None
    except ET.ParseError as e:
        return str(e)


def walk(root):
    for dirpath, _dirs, files in os.walk(root):
        # Skip generated/cache directories
        if "__pycache__" in dirpath or ".git" in dirpath:
            continue
        for f in files:
            if f.lower().endswith(EXTS):
                yield os.path.join(dirpath, f)


def main():
    roots = sys.argv[1:] or DEFAULT_ROOTS
    n_files = 0
    n_comment = 0
    n_parse = 0
    for root in roots:
        if not os.path.isdir(root):
            print("ROOT NOT FOUND: " + root)
            continue
        for p in walk(root):
            n_files += 1
            try:
                with open(p, encoding="utf-8", errors="replace") as fh:
                    txt = fh.read()
            except Exception as e:
                print("READ ERROR ({}): {}".format(p, e))
                n_parse += 1
                continue
            rel = os.path.relpath(p, root)
            offences = scan_comments(txt)
            for ln, msg, line in offences:
                print("ERROR: {} in {}, line {}".format(msg, rel, ln))
                print("      > {}".format(line))
                n_comment += 1
            err = parse_xml(p)
            if err is not None:
                print("XML PARSE ERROR ({}): {}".format(rel, err))
                n_parse += 1
    print()
    print("Audited {} XML-family files.".format(n_files))
    print("  illegal '--' inside XML comments : {}".format(n_comment))
    print("  XML parse errors                 : {}".format(n_parse))
    return 0 if (n_comment == 0 and n_parse == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
