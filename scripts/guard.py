"""Claude Code PreToolUse guard: keep the quality gate out of the agent's reach.

Blocks (exit 2, reason on stderr):
- Edit/Write/NotebookEdit of a protected file,
- Bash commands that modify a protected file,
- `git commit --no-verify` / `-n`, which would skip the git hooks.
Protected files change only by hand, or with the guard switched off in /hooks.
"""

import json
import os
import re
import sys

PROTECTED = ("scripts/check.sh", "scripts/guard.py", ".claude/settings.json", ".githooks/")
WRITE_OPS = re.compile(r"(>|\btee\b|\bsed\s+-i|\brm\b|\bmv\b|\bcp\b|\bchmod\b|\btruncate\b|\bln\b|\bgit\s+(checkout|restore)\b)")
NO_VERIFY = re.compile(r"\bgit\b[^;&|]*\bcommit\b[^;&|]*(--no-verify|\s-[a-zA-Z]*n\b)")


def is_protected(path: str, root: str) -> bool:
    rel = os.path.relpath(os.path.abspath(os.path.join(root, path)), root)
    return any(rel == p or (p.endswith("/") and rel.startswith(p)) for p in PROTECTED)


def block(reason: str) -> None:
    print(f"Blocked by scripts/guard.py: {reason}", file=sys.stderr)
    sys.exit(2)


def main() -> None:
    event = json.load(sys.stdin)
    root = os.environ.get("CLAUDE_PROJECT_DIR") or event.get("cwd") or os.getcwd()
    tool, args = event.get("tool_name", ""), event.get("tool_input", {})

    if tool in ("Edit", "Write", "NotebookEdit"):
        path = args.get("file_path") or args.get("notebook_path") or ""
        if is_protected(path, root):
            block(f"{path} is part of the quality gate; ask the user to change it.")
    elif tool == "Bash":
        cmd = args.get("command", "")
        if NO_VERIFY.search(cmd):
            block("committing with --no-verify skips the quality gate.")
        if WRITE_OPS.search(cmd) and any(p.rstrip("/") in cmd for p in PROTECTED):
            block("this command would modify a quality-gate file; ask the user to change it.")


if __name__ == "__main__":
    main()
