# dead-reckoning

A power-loss-recovery (PLR) system for Klipper 3D printers. dead-reckoning
maintains a write-ahead log of print motion and, after an unexpected power
loss, reconstructs the printer's last known state so a print can be resumed
in place instead of scrapped. This repository is a Rust workspace; project
scaffolding is in progress and feature crates land on review branches.
