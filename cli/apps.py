"""brio apps [project|all] [filter] - the roster of the apps, from the
sources themselves: one main() per src/apps/<app>.cpp in each build
project (plus each experiment's per-architecture halves), with the
first line of the app's own header comment and the boards its
`// build:` line names. Needs no configure and no board: the roster the
build discovers is the one this prints."""

import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECTS = ("avrdx", "samc21", "stm32g0", "ch32v00x", "ch32vx03", "ch32x035", "rp2040", "rp2350", "stm32f4", "host")


def head_line(path):
    """The first line of the header comment, without its comment marks
    and without the app's own name repeated in front ("blink - ...")."""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("//"):
                text = line.lstrip("/").strip()
            elif line.startswith("/*") or line.startswith("*"):
                text = line.lstrip("/*").strip()
            else:
                return ""
            if not text:
                continue
            name = os.path.splitext(os.path.basename(path))[0]
            for sep in (" - ", ": "):
                if text.startswith(name + sep):
                    text = text[len(name) + len(sep):]
                    break
            return text
    return ""


def groups_line(path):
    """The groups of letters the app's '// build: groups' lines name, or
    None: the images it splits into on a board type its project lists as
    splitting (its CMakeLists.txt's SPLIT_BOARDS) - the plain line - and
    on one board type alone - a 'groups.<board type>' line, shown as
    "<board type>: <groups>"."""
    found = []
    with open(path, encoding="ascii", errors="replace") as f:
        for raw in f:
            m = re.match(r"\s*//\s*build:\s*groups(?:\.([A-Za-z0-9_-]+))?\s*=\s*(.*)", raw)
            if m:
                found.append((m.group(1) + ": " if m.group(1) else "") + m.group(2).strip())
    return "; ".join(found) or None


def boards_line(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            m = re.match(r"\s*//\s*build:\s*boards\s*=\s*(.*)", raw)
            if m:
                return m.group(1).strip()
            if raw.startswith("#include"):
                break
    return ""


def roster(projects):
    rows = []
    for project in projects:
        paths = sorted(glob.glob(os.path.join(ROOT, project, "src", "apps", "*.cpp")))
        paths += sorted(glob.glob(os.path.join(ROOT, "experiments", "*", project, "*.cpp")))
        for p in paths:
            name = os.path.splitext(os.path.basename(p))[0]
            where = os.path.relpath(os.path.dirname(p), ROOT)
            rows.append((project, name, boards_line(p), groups_line(p), head_line(p), where))
    return rows


def main(argv):
    args = argv[1:]
    projects = PROJECTS
    if args and args[0] in PROJECTS:
        projects = (args[0],)
        args = args[1:]
    elif args and args[0] == "all":
        args = args[1:]
    needle = args[0].lower() if args else ""
    rows = [r for r in roster(projects) if needle in r[1].lower() or needle in r[4].lower()]
    if not rows:
        print("brio apps: nothing matches")
        return 1
    width = max(len(r[1]) for r in rows)
    for project, name, boards, groups, head, where in rows:
        tag = project if not where.startswith("experiments") else where.split(os.sep)[1] + "/" + project
        line = "%-8s %-*s" % (tag, width, name)
        if boards:
            line += "  [%s]" % boards
        if groups:
            line += "  [groups %s]" % groups
        if head:
            line += "  " + head
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
