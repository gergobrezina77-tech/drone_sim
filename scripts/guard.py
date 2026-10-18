"""Claude Code PreToolUse guard: keep the quality gate out of the agent's reach.

Blocks (exit 2, reason on stderr):
- Edit/Write/NotebookEdit of a protected file,
- Bash commands that write to a protected file (redirect target, cp/mv/ln/install destination, rm/chmod/sed -i/...
  operand, git checkout/restore of it, or inline interpreter code that mentions it),
- `git commit --no-verify` / `-n`, which would skip the git hooks.
Commands are tokenised with shlex, so quoted text (e.g. a commit message) never triggers a block.
Protected files change only by hand, or with the guard switched off in /hooks.
"""

import json
import os
import re
import shlex
import sys

PROTECTED = ("scripts/check.sh", "scripts/guard.py", ".claude/settings.json", ".githooks/")
SEPARATORS = {";", "&&", "||", "|", "&", "|&", "(", ")"}
REDIRECTS = {">", ">>", ">|", "&>", "&>>", ">&"}
DEST_WRITERS = {"cp", "mv", "ln", "install", "rsync"}  # last operand is written
ANY_WRITERS = {"rm", "chmod", "chown", "truncate", "shred", "unlink", "touch"}  # every operand is written
INTERPRETERS = {"python", "python3", "perl", "ruby", "node", "bash", "sh"}
INLINE_FLAGS = {"-c", "-e"}
COMMIT_VALUE_SHORT = set("mFCct")  # short options of `git commit` that take a value


def is_protected(path: str, root: str) -> bool:
    rel = os.path.relpath(os.path.abspath(os.path.join(root, path)), root)
    return any(rel == p or (p.endswith("/") and (rel + "/").startswith(p)) for p in PROTECTED)


def block(reason: str) -> None:
    print(f"Blocked by scripts/guard.py: {reason}", file=sys.stderr)
    sys.exit(2)


def segments(cmd: str) -> list[list[str]]:
    """Split a shell command into simple commands; quoted text stays a single token."""
    lex = shlex.shlex(cmd, posix=True, punctuation_chars=True)
    lex.whitespace_split = True
    out, cur = [], []
    for tok in lex:
        if tok in SEPARATORS:
            out.append(cur)
            cur = []
        else:
            cur.append(tok)
    out.append(cur)
    return [s for s in out if s]


def skips_hooks(seg: list[str]) -> bool:
    """`git ... commit` with --no-verify or a short-option cluster containing n."""
    if not seg or os.path.basename(seg[0]) != "git" or "commit" not in seg:
        return False
    args = seg[seg.index("commit") + 1 :]
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--":
            break
        if a == "--no-verify":
            return True
        if a.startswith("--"):
            i += 1 if "=" in a or a not in ("--message", "--file", "--reuse-message", "--reedit-message") else 2
            continue
        if a.startswith("-") and len(a) > 1:
            for j, ch in enumerate(a[1:]):
                if ch == "n":
                    return True
                if ch in COMMIT_VALUE_SHORT:  # rest of the cluster (or the next token) is the value
                    if j == len(a) - 2:
                        i += 1
                    break
        i += 1
    return False


def written_paths(seg: list[str]) -> list[str]:
    paths = [seg[k + 1] for k, t in enumerate(seg[:-1]) if t in REDIRECTS]
    words = [t for k, t in enumerate(seg) if t not in REDIRECTS and (k == 0 or seg[k - 1] not in REDIRECTS)]
    if not words:
        return paths
    prog, operands = os.path.basename(words[0]), [w for w in words[1:] if not w.startswith("-")]
    if prog in DEST_WRITERS and operands:
        paths.append(operands[-1])
    elif prog in ANY_WRITERS or prog == "tee" or (prog == "sed" and any(w.startswith("-i") for w in words)):
        paths += operands
    elif prog == "git" and len(words) > 1 and words[1] in ("checkout", "restore", "rm", "mv"):
        paths += operands[1:]
    return paths


def inline_code_mentions_protected(seg: list[str]) -> bool:
    if not seg or os.path.basename(seg[0]).rstrip("0123456789.") not in INTERPRETERS:
        return False
    code = " ".join(seg[k + 1] for k, t in enumerate(seg[:-1]) if t in INLINE_FLAGS)
    return any(p.rstrip("/") in code for p in PROTECTED)


def check_bash(cmd: str, root: str) -> None:
    try:
        segs = segments(cmd)
    except ValueError:  # unbalanced quotes etc.: fall back to a conservative substring check
        if re.search(r"--no-verify", cmd) or any(p.rstrip("/") in cmd for p in PROTECTED):
            block("could not parse this command and it mentions the quality gate.")
        return
    for seg in segs:
        if skips_hooks(seg):
            block("committing with --no-verify skips the quality gate.")
        if any(is_protected(p, root) for p in written_paths(seg)) or inline_code_mentions_protected(seg):
            block("this command would modify a quality-gate file; ask the user to change it.")


def main() -> None:
    event = json.load(sys.stdin)
    root = os.environ.get("CLAUDE_PROJECT_DIR") or event.get("cwd") or os.getcwd()
    tool, args = event.get("tool_name", ""), event.get("tool_input", {})

    if tool in ("Edit", "Write", "NotebookEdit"):
        path = args.get("file_path") or args.get("notebook_path") or ""
        if is_protected(path, root):
            block(f"{path} is part of the quality gate; ask the user to change it.")
    elif tool == "Bash":
        check_bash(args.get("command", ""), root)


if __name__ == "__main__":
    main()
