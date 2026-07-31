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
//!   behavior. We point it at `.git/HEAD` and the ref HEAD names (resolved
//!   via `git rev-parse --git-path`, which is correct inside a worktree
//!   where `.git` is a file and HEAD lives under `.git/worktrees/<name>/`),
//!   so the hash is recomputed only when the checkout actually moves.

use std::process::Command;

fn main() {
    let hash = git_hash().unwrap_or_else(|| "unknown".to_owned());
    println!("cargo:rustc-env=PLRD_GIT_HASH={hash}");

    // Scope the rebuild triggers. Without at least one rerun-if line, cargo
    // reruns this script whenever ANY package file changes; with them, only
    // a real HEAD/ref move recomputes the hash. `build.rs` is always watched
    // by cargo regardless, and naming it keeps a git-less (tarball) build
    // from falling back to the whole-package scan.
    //
    // Only EXISTING paths are emitted: cargo treats a `rerun-if-changed`
    // target that does not exist as perpetually-changed and rebuilds every
    // compile — so watching e.g. a `packed-refs` that this repo (loose refs)
    // does not have would defeat the whole point. See [`rerun_paths`].
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

/// Files whose change means HEAD moved: `.git/HEAD` and, when HEAD is a
/// symbolic ref, the ref file it points at (plus `packed-refs`, which is
/// where a ref lives after `git gc`/`git pack-refs`). Resolved through
/// `git rev-parse --git-path` so it is correct for worktrees and unusual
/// `GIT_DIR` layouts. Only paths that EXIST are returned (see the caller);
/// empty when git is unavailable.
///
/// The loose-ref file and `packed-refs` are mutually exclusive for a given
/// ref, and packing/unpacking DELETES one and CREATES the other — a change
/// cargo sees on the file it was already watching, so whichever exists now
/// is enough to catch the transition.
fn rerun_paths() -> Vec<String> {
    let mut paths = Vec::new();
    let Some(head_path) = git(&["rev-parse", "--git-path", "HEAD"]) else {
        return paths;
    };
    // If HEAD is detached, its content IS the hash and there is no ref file
    // to also watch; watching HEAD alone suffices.
    if let Ok(head) = std::fs::read_to_string(&head_path) {
        if let Some(reference) = head.strip_prefix("ref:") {
            let reference = reference.trim();
            if let Some(ref_path) = git(&["rev-parse", "--git-path", reference]) {
                paths.push(ref_path);
            }
            if let Some(packed) = git(&["rev-parse", "--git-path", "packed-refs"]) {
                paths.push(packed);
            }
        }
    }
    paths.push(head_path);
    // Drop any that do not exist: cargo rebuilds every compile for a
    // `rerun-if-changed` path it cannot stat (e.g. `packed-refs` in a
    // loose-ref repo, or a ref that is currently packed).
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
