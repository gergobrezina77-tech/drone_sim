# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

---

# Project workflow (drone-sim)

Architecture: `docs/ARCHITECTURE.md`. Board: `docs/board/BOARD.md`, tickets in `docs/board/tickets/`.

## Ticket loop

1. **Start:** if a ticket is *In progress*, resume it. Otherwise move the first *Ready* ticket to *In progress*
   (board + its `Status:` line). One ticket at a time; `git status` clean apart from its work.
2. **Test first:** turn every acceptance criterion into a test and see it fail.
3. **Implement** only the ticket's scope: its layer package(s), tests, ticket file and board. Anything else becomes a
   new Backlog ticket (same template, next id).
4. **Gate:** `scripts/check.sh` passes. No `skip`/`xfail`, loosened tolerances or weakened tests to get there.
5. **Close:** Notes (decisions, measurements), `Status: Done`, move to *Done*, promote unblocked tickets to *Ready*.
6. **Commit** once: `DS-XXX: <title>`. Board/tooling-only changes use `chore: <text>`.
7. **Report** each criterion with its proving test (or "manual: …"), gate result, commit hash, promoted and new
   tickets. Then stop, unless told to keep going (e.g. "work through M1").

## Stop and ask when

- A criterion is ambiguous or contradicts the architecture, or the ticket needs another layer or a >~400-line diff.
- A lower-layer interface changes (protocol, `WorldSpec` field, API schema), or a new dependency is needed.
- A decision is hard to reverse: record it in `docs/decisions/NNN-title.md` and link it from the ticket.

## Engineering rules

- Extension points are a protocol + a name registry; callers look implementations up by name.
- Fleet data is struct-of-arrays numpy; no per-drone Python loops in the simulation tick.

## Enforcement — never bypass or edit

`scripts/check.sh`, `scripts/guard.py`, `.claude/settings.json`, `.githooks/` and the `[tool.importlinter]` contract
are off limits; changes need the user's approval. Git hooks (`git config core.hooksPath .githooks`) run the gate on
commit and check the message format; Claude Code hooks run the gate on Stop and block edits to these files.
