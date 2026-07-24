---
name: Bug report
about: Something misbehaved — a daemon error, a wrong scan report, a bad plan
labels: bug
---

## What happened

<!-- What did you observe? Paste the exact message / report lines. -->

## What you expected

## Environment

- dead-reckoning version / commit (`plrd version`):
- Klipper version, printer (moving-bed-Z model, probe type):
- Host (Pi model / OS) and where `wal_dir` lives:

## Evidence

<!--
The more of these the better:
- journalctl -u plrd output around the event
  (docs/operations.md#understanding-plrds-logs explains the messages)
- full `plrd scan --wal <dir>` output
- for parse/reconstruction bugs: the WAL directory (segments are
  greppable JSON-in-frames) and, if possible, the sliced .gcode file
-->
