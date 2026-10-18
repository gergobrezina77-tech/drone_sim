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

Architecture and layer rules: `docs/ARCHITECTURE.md`. Work tracking: `docs/board/BOARD.md` + `docs/board/tickets/`.

## Ticket loop — one ticket at a time

1. **Pick:** take the first ticket in *Ready* (all dependencies Done). Move it to *In progress* in `BOARD.md` and set
   its `Status:` line. Never have more than one ticket in progress.
2. **Implement** only that ticket's scope. Anything else you notice becomes a new Backlog ticket, not a drive-by change.
3. **Test:** add tests that cover every acceptance criterion. Tick the criteria in the ticket as they are met.
4. **Gate:** `scripts/check.sh` must pass (ruff, import-linter layer contract, pytest). The Stop hook runs it too
   and blocks finishing while it fails.
5. **Close:** write decisions and measurements in the ticket's Notes, set `Status: Done`, move it to *Done* in
   `BOARD.md`, promote Backlog tickets whose dependencies are now all Done to *Ready*.
6. **Commit** once per ticket: `DS-XXX: <title>` (code + tests + board update together).

If a ticket turns out to be wrong or too big, stop and say so; split or amend it on the board before coding.

## Engineering rules

- Respect the layers: never add an import that `lint-imports` rejects, and never weaken the contract to make it pass.
- Extension points are a protocol + a name registry; callers look implementations up by name.
- Fleet data is struct-of-arrays numpy; no per-drone Python loops in the simulation tick.
