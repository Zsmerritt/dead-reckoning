//! Build script for `plrd`: embeds the current git commit hash into the
//! binary as the `PLRD_GIT_HASH` compile-time env var, so `plrd version`
//! can print `plrd <semver> (<hash>)` and a production log can be tied to
//! an exact tree (the reason this crate has a build script at all).
//!
//! Design constraints:
//!
//! * **Tarball-safe.** A build outside a git checkout (a released source
//!   tarball, a vendored copy) has no `git` and no `.git` — it must still
//!   compile. Every git step is best-effort; the fallback is the literal
//!   `unknown`.
//! * **No spurious rebuilds.** Emitting any `cargo:rerun-if-changed`
//!   opts out of cargo's default "rescan the whole package every build"
//!   behavior. We point it at `.git/logs/HEAD` (plus `.git/HEAD` and the
//!   named ref as extras), resolved via `git rev-parse --git-path` — correct
//!   inside a worktree where `.git` is a file and HEAD/logs live under
//!   `.git/worktrees/<name>/` — so the hash is recomputed only when the
//!   checkout actually moves. See [`rerun_paths`] for why `logs/HEAD` is the
//!   load-bearing one.

use std::process::Command;

fn main() {
    let hash = git_hash().unwrap_or_else(|| "unknown".to_owned());
    println!("cargo:rustc-env=PLRD_GIT_HASH={hash}");

    // Scope the rebuild triggers. Without at least one rerun-if line, cargo
    // reruns this script whenever ANY package file changes; with them, only
    // a real HEAD move recomputes the hash. `build.rs` is always watched
    // by cargo regardless, and naming it keeps a git-less (tarball) build
    // from falling back to the whole-package scan.
    //
    // Only EXISTING paths are emitted: cargo treats a `rerun-if-changed`
    // target that does not exist as perpetually-changed and rebuilds every
    // compile. See [`rerun_paths`] for the watched set and why `logs/HEAD`
    // is the one that actually guarantees freshness.
    println!("cargo:rerun-if-changed=build.rs");
    for path in rerun_paths() {
        println!("cargo:rerun-if-changed={path}");
    }
}

/// `<short-hash>` or `<short-hash>-dirty`, or `None` when git is
/// unavailable / this is not a checkout.
fn git_hash() -> Option<String> {
    let short = git(&["rev-parse", "--short", "HEAD"])?;
    if short.is_empty() {
        return None;
    }
    // `git status --porcelain` prints one line per changed/untracked path
    // and nothing when the tree is clean — the same signal `git describe
    // --dirty` uses, without needing a tag to describe against.
    let dirty = git(&["status", "--porcelain"]).is_some_and(|s| !s.is_empty());
    Some(if dirty {
        format!("{short}-dirty")
    } else {
        short
    })
}

/// Files whose change means HEAD moved.
///
/// **`logs/HEAD` is the load-bearing one** — the invariant the freshness
/// guarantee actually rests on. It is a single per-worktree file that git
/// APPENDS a line to on EVERY ref movement of HEAD (commit, checkout, reset,
/// pull, merge), regardless of whether refs are loose or packed. Watching the
/// ref file alone is not enough: after `git pack-refs --all` the loose ref is
/// deleted, and a subsequent commit writes a BRAND-NEW loose ref that was not
/// in the watched set while leaving `packed-refs` and `HEAD` byte-identical —
/// so cargo would never rerun and the embedded hash would go stale. The
/// appended reflog entry closes that hole.
///
/// `.git/HEAD` and the named ref file are kept as best-effort EXTRAS (they
/// help the rare `core.logAllRefUpdates=false` config where no reflog exists,
/// and `HEAD` itself changes on a branch switch that leaves the old branch's
/// tip untouched). All three are resolved through `git rev-parse --git-path`
/// so they are correct for worktrees and unusual `GIT_DIR` layouts.
///
/// Only paths that EXIST are returned (see the caller); empty when git is
/// unavailable (a tarball build — hash is `unknown`, nothing to watch).
fn rerun_paths() -> Vec<String> {
    let mut paths = Vec::new();
    let Some(head_path) = git(&["rev-parse", "--git-path", "HEAD"]) else {
        return paths;
    };
    // The universal ref-movement signal (see the doc comment): appended to on
    // every HEAD move whether refs are loose or packed.
    if let Some(logs_head) = git(&["rev-parse", "--git-path", "logs/HEAD"]) {
        paths.push(logs_head);
    }
    // Best-effort extras. If HEAD is detached, its content IS the hash and
    // there is no ref file to also watch; watching HEAD alone suffices.
    if let Ok(head) = std::fs::read_to_string(&head_path) {
        if let Some(reference) = head.strip_prefix("ref:") {
            let reference = reference.trim();
            if let Some(ref_path) = git(&["rev-parse", "--git-path", reference]) {
                paths.push(ref_path);
            }
        }
    }
    paths.push(head_path);
    // Drop any that do not exist: cargo rebuilds every compile for a
    // `rerun-if-changed` path it cannot stat (a currently-packed ref, or
    // `logs/HEAD` under `core.logAllRefUpdates=false`).
    paths.retain(|p| std::path::Path::new(p).exists());
    paths
}

/// Runs `git <args>` and returns trimmed stdout, or `None` if git is
/// missing or the command failed (non-zero exit). Never panics — a build
/// outside a checkout must still succeed.
fn git(args: &[&str]) -> Option<String> {
    let output = Command::new("git").args(args).output().ok()?;
    if !output.status.success() {
        return None;
    }
    Some(String::from_utf8_lossy(&output.stdout).trim().to_owned())
}
