## What / why

<!-- What does this change do, and why? Link issues if any. -->

## Checklist

- [ ] Pre-commit hooks installed (`sh scripts/setup-hooks.sh`) and passing
      (fmt, clippy `-D warnings`, tests, ≥90% coverage)
- [ ] New/changed behavior is covered by a test
- [ ] If proptest found a failure: the `*.proptest-regressions` seed is
      committed
- [ ] If this touches plan generation, envelope math, machine validation,
      or the WAL durability path: the PR body states the safety invariant
      preserved and the test that proves it
