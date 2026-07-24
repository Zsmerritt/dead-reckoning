# klippy_plugin — Klipper console plugin for dead-reckoning

Klipper (klippy) extras plugin for dead-reckoning power-loss recovery.
All user interaction happens through `PLR_*` g-code console commands
registered by a `[plr]` config section; the heavy lifting stays in the
Rust daemon (`plrd`).

**Install**: symlink `klippy_plugin/plr` into your Klipper checkout as
`klippy/extras/plr` (full install/usage docs to come).

## Development

- One-time setup: `sh scripts/setup-py.sh` (creates `.venv` at the repo
  root with the pinned dev deps from `requirements-dev.txt`; required —
  the pre-commit hook fails without it).
- Gates (enforced by `.githooks/pre-commit` and CI): `ruff check`,
  `ruff format --check`, and pytest with ≥90% line coverage over `plr/`
  (`scripts/coverage-py.sh`).
- Version split: code under `plr/` must stay **Python 3.7
  syntax-compatible** (it runs inside klippy; Klipper supports 3.7+).
  The dev tooling floor is **Python 3.9** — see `pyproject.toml`.
