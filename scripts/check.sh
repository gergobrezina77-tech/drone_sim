#!/usr/bin/env bash
# Quality gate: ruff (lint + format), import-linter (layer contract), pytest.
#   scripts/check.sh          run all checks, exit non-zero on failure
#   scripts/check.sh --hook   Claude Code Stop hook mode: skip when no code changed, exit 2 (block) on failure
set -uo pipefail
cd "$(dirname "$0")/.."

hook=false
[[ "${1:-}" == "--hook" ]] && hook=true

if $hook; then
    input=$(cat)
    # Only gate when code or build config has uncommitted changes.
    if [[ -z "$(git status --porcelain -- src tests web pyproject.toml poetry.lock 2>/dev/null)" ]]; then
        exit 0
    fi
fi

out=$(
    {
        poetry run ruff check src tests &&
            poetry run ruff format --check src tests &&
            poetry run lint-imports &&
            poetry run pytest -q -x
    } 2>&1
)
status=$?

if [[ $status -eq 0 ]]; then
    $hook || echo "$out" | tail -n 5
    exit 0
fi

if $hook; then
    # A second consecutive block would loop forever on an unfixable failure: let it stop, but tell the user.
    if grep -q '"stop_hook_active": *true' <<<"$input"; then
        echo '{"systemMessage": "Quality gate still failing (ruff / import-linter / pytest). Run scripts/check.sh to see why."}'
        exit 0
    fi
    { echo "Quality gate failed. Fix before finishing (the ticket is not done):"; echo "$out" | tail -n 40; } >&2
    exit 2
fi
echo "$out"
exit $status
