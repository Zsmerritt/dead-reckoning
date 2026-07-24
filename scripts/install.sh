#!/usr/bin/env bash
# dead-reckoning installer: builds and installs the plrd power-loss
# recovery recorder on a Klipper printer host (Debian / Raspberry Pi OS).
#
# One-command install (run as your normal printer user, NOT root):
#
#   curl -sSL https://raw.githubusercontent.com/Zsmerritt/dead-reckoning/main/scripts/install.sh | bash
#
# Equally runnable from a clone:  bash scripts/install.sh
#
# Safety: this script NEVER talks to the Klipper socket and never
# touches the klipper or moonraker services. The only Moonraker-related
# changes (moonraker.conf / moonraker.asvc edits) are opt-in, backed up
# first, and you restart Moonraker yourself. The Klipper console plugin
# is installed as a single symlink into the klipper checkout's
# klippy/extras; printer.cfg is NEVER edited (adding [plr] and
# restarting Klipper are yours to do). plrd itself is a read-only
# observer of Klipper; the installer is equally inert toward the printer.
set -euo pipefail

# --- constants -------------------------------------------------------------

REPO_URL_DEFAULT="https://github.com/Zsmerritt/dead-reckoning.git"
REPO_URL="${PLRD_INSTALL_REPO:-$REPO_URL_DEFAULT}"
BRANCH="${PLRD_INSTALL_BRANCH:-main}"

BIN_PATH="/usr/local/bin/plrd"
CONF_PATH="/etc/plrd.conf"
UNIT_PATH="/etc/systemd/system/plrd.service"
DROPIN_DIR="/etc/systemd/system/plrd.service.d"
DROPIN_PATH="$DROPIN_DIR/50-plrd-refresh.conf"
STATE_DIR="/var/lib/plrd"
WAL_DIR="$STATE_DIR/wal"
BUILD_STAMP="$STATE_DIR/build-head"
BUILD_FAIL_STAMP="$STATE_DIR/build-failed-head"

# Rough floor for a release build plus (if needed) the pinned toolchain.
MIN_DISK_KB=$((2 * 1024 * 1024))     # 2 GiB hard floor
WARN_DISK_KB=$((4 * 1024 * 1024))    # below this, warn
LOW_MEM_KB=$((1600 * 1024))          # below this, build with -j1

# --- options ---------------------------------------------------------------

ASSUME_YES=0
NO_SERVICE=0
WANT_MOONRAKER=0
FORCE_CONFIG=0
REFRESH_MODE=0
REPO_DIR=""
PRINTER_DATA=""
KLIPPER_DIR=""

usage() {
    cat <<'EOF'
dead-reckoning installer (plrd)

USAGE:
    install.sh [OPTIONS]

OPTIONS:
    --dir <path>           Repo checkout location (default: ~/dead-reckoning,
                           or the enclosing repo when run from a clone)
    --printer-data <path>  Klipper printer_data directory
                           (default: ~/printer_data)
    --klipper <path>       Klipper source checkout, used to symlink the
                           console plugin into <path>/klippy/extras
                           (default: auto-detect, ~/klipper first)
    --moonraker            Register plrd with Moonraker's update manager
                           (moonraker.conf + moonraker.asvc, backed up first).
                           Without this flag you are asked interactively;
                           the non-interactive default is "no".
    --force-config         Regenerate /etc/plrd.conf even if it exists
                           (a timestamped backup is made first)
    --no-service           Build and stage only: no sudo, no files installed,
                           no systemd changes
    --yes                  Non-interactive: accept the default answer for
                           every prompt
    --help                 Show this help

ADVANCED:
    --refresh              Internal mode used by the systemd drop-in that
                           Moonraker integration installs: rebuild from the
                           already-updated repo and refresh the installed
                           binary. No prompts, no config or service changes.

The installer never communicates with Klipper and never restarts the
klipper or moonraker services.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dir)
            [[ $# -ge 2 ]] || { echo "error: --dir requires a value" >&2; exit 2; }
            REPO_DIR="$2"; shift 2 ;;
        --printer-data)
            [[ $# -ge 2 ]] || { echo "error: --printer-data requires a value" >&2; exit 2; }
            PRINTER_DATA="$2"; shift 2 ;;
        --klipper)
            [[ $# -ge 2 ]] || { echo "error: --klipper requires a value" >&2; exit 2; }
            KLIPPER_DIR="$2"; shift 2 ;;
        --moonraker)    WANT_MOONRAKER=1; shift ;;
        --force-config) FORCE_CONFIG=1; shift ;;
        --no-service)   NO_SERVICE=1; shift ;;
        --yes|-y)       ASSUME_YES=1; shift ;;
        --refresh)      REFRESH_MODE=1; shift ;;
        --help|-h)      usage; exit 0 ;;
        *)
            echo "error: unknown option '$1' (see --help)" >&2
            exit 2 ;;
    esac
done

# --- logging / prompting helpers -------------------------------------------

if [[ -t 1 ]]; then
    C_STEP=$'\033[1;32m'; C_WARN=$'\033[1;33m'; C_ERR=$'\033[1;31m'; C_OFF=$'\033[0m'
else
    C_STEP=""; C_WARN=""; C_ERR=""; C_OFF=""
fi

log()  { printf '%s==>%s %s\n' "$C_STEP" "$C_OFF" "$*"; }
warn() { printf '%sWARNING:%s %s\n' "$C_WARN" "$C_OFF" "$*" >&2; }
die()  { printf '%sERROR:%s %s\n' "$C_ERR" "$C_OFF" "$*" >&2; exit 1; }

# A controlling terminal lets us prompt even when stdin is the curl pipe.
INTERACTIVE=0
if { : </dev/tty; } 2>/dev/null; then
    INTERACTIVE=1
fi

# confirm <question> <default y|n> — honors --yes and non-interactive runs
# by returning the default.
confirm() {
    local question="$1" default="${2:-n}" suffix ans
    if [[ $ASSUME_YES -eq 1 || $INTERACTIVE -eq 0 ]]; then
        [[ "$default" == y ]]
        return
    fi
    suffix="[y/N]"
    [[ "$default" == y ]] && suffix="[Y/n]"
    read -r -p "$question $suffix " ans </dev/tty || ans=""
    ans="${ans,,}"
    [[ -z "$ans" ]] && ans="$default"
    [[ "$ans" == y || "$ans" == yes ]]
}

have() { command -v "$1" >/dev/null 2>&1; }

run_root() {
    if [[ $(id -u) -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

# --- klippy console plugin symlink ------------------------------------------
#
# The Klipper console plugin (klippy_plugin/plr — the PLR_* commands)
# loads like any other klippy extras module, via one symlink:
#     <klipper>/klippy/extras/plr -> <repo>/klippy_plugin/plr
# Creating it needs no sudo (the klipper checkout belongs to the printer
# user) and is idempotent (ln -sfn re-points our own link; anything that
# is not ours is never touched). printer.cfg is never edited and Klipper
# is never restarted — activating the plugin ([plr] section in
# printer.cfg + RESTART) is deliberately left to you.

PLUGIN_LINK=""    # set once the symlink exists (used by the summary)

# Prints the klipper checkout to use. --klipper wins; otherwise
# <home>/klipper (the standard KIAUH/kiauh-like layout). A candidate
# qualifies only if klippy/extras exists — that is the directory klippy
# scans for extras modules, and its presence is what distinguishes a
# source checkout from, say, a bare printer_data directory.
detect_klipper_dir() {
    local home="$1"
    if [[ -n "$KLIPPER_DIR" ]]; then
        if [[ -d "$KLIPPER_DIR/klippy/extras" ]]; then
            echo "$KLIPPER_DIR"
            return 0
        fi
        warn "--klipper $KLIPPER_DIR has no klippy/extras directory (not a Klipper source checkout?)"
        return 1
    fi
    if [[ -d "$home/klipper/klippy/extras" ]]; then
        echo "$home/klipper"
        return 0
    fi
    return 1
}

# ensure_plugin_symlink <home> [quiet] — create or repair the extras
# symlink. Never fatal: a missing klipper checkout gets manual
# instructions and the install continues ("quiet" suppresses those —
# used by --refresh, which runs on every service restart and must not
# spam the journal). Only OUR link is ever (re)written — an existing
# regular file/directory, or a live symlink resolving outside this
# repo, is warned about and left alone.
ensure_plugin_symlink() {
    local home="$1" quiet="${2:-}" plugin_src="$REPO_DIR/klippy_plugin/plr" klipper link target
    if [[ ! -d "$plugin_src" ]]; then
        [[ -n "$quiet" ]] \
            || warn "no klippy plugin at $plugin_src (checkout predates the console plugin?); skipping the plugin symlink"
        return 0
    fi
    if ! klipper="$(detect_klipper_dir "$home")"; then
        if [[ -z "$quiet" ]]; then
            warn "no Klipper source checkout found (looked for klippy/extras under ${KLIPPER_DIR:-$home/klipper})."
            warn "The PLR_* console commands need the plugin linked into klippy. Once you know the path, run:"
            warn "    ln -sfn $plugin_src <klipper>/klippy/extras/plr"
            warn "or re-run this installer with --klipper <path-to-klipper>."
        fi
        return 0
    fi
    link="$klipper/klippy/extras/plr"
    if [[ -e "$link" && ! -L "$link" ]]; then
        warn "$link exists and is not a symlink — leaving it alone."
        warn "If it is a stale copy of the plugin, remove it, then run: ln -sfn $plugin_src $link"
        return 0
    fi
    if [[ -L "$link" ]]; then
        target="$(readlink "$link")"
        # A live symlink into some other tree is not ours to replace
        # (it may be a different plugin that happens to be named plr).
        # A dangling one, or one pointing anywhere inside this repo, is
        # ours to fix.
        if [[ -e "$link" && "$target" != "$REPO_DIR"/* ]]; then
            warn "$link points to $target (outside $REPO_DIR) — leaving it alone."
            warn "To use this checkout's plugin instead: ln -sfn $plugin_src $link"
            return 0
        fi
    fi
    ln -sfn "$plugin_src" "$link"
    # In --refresh mode this runs as root inside the user's klipper
    # checkout; hand the (new) symlink to the directory's owner.
    if [[ $(id -u) -eq 0 ]]; then
        chown -h --reference="$klipper/klippy/extras" "$link" 2>/dev/null || true
    fi
    PLUGIN_LINK="$link"
    log "klippy plugin linked: $link -> $plugin_src"
}

# --- refresh mode (called from the systemd drop-in, as root) ----------------
#
# Moonraker's update manager pulls the repo and then restarts the plrd
# service (managed_services). It cannot run build steps for git_repo
# entries, so the rebuild is hooked into the service restart via an
# ExecStartPre drop-in that runs `install.sh --refresh`. This mode:
#   * is a fast no-op when the installed binary already matches HEAD,
#   * builds as the repo's owner (never as root inside the checkout),
#   * only replaces /usr/local/bin/plrd — no config, no service actions.
refresh() {
    [[ $(id -u) -eq 0 ]] || die "--refresh is meant to run as root from the plrd.service drop-in"
    [[ -n "$REPO_DIR" ]] || die "--refresh requires --dir <repo>"
    [[ -d "$REPO_DIR/.git" ]] || die "--refresh: $REPO_DIR is not a git checkout"

    local head owner owner_home
    owner="$(stat -c %U "$REPO_DIR")"
    owner_home="$(getent passwd "$owner" | cut -d: -f6)"
    # Keep the klippy plugin symlink healthy across updates (an update
    # that first ships the plugin, or a repaired dangling link). Cheap,
    # best-effort, and independent of whether a rebuild is needed.
    if [[ -n "$owner_home" ]]; then
        ensure_plugin_symlink "$owner_home" quiet
    fi
    # git refuses to read a repo owned by another user ("dubious
    # ownership"), so ask git as the repo's owner.
    if [[ "$owner" != "root" ]]; then
        head="$(runuser -u "$owner" -- git -C "$REPO_DIR" rev-parse HEAD)"
    else
        head="$(git -C "$REPO_DIR" rev-parse HEAD)"
    fi
    if [[ -f "$BUILD_STAMP" && "$(cat "$BUILD_STAMP")" == "$head" ]]; then
        log "plrd refresh: binary already built from $head — nothing to do"
        return 0
    fi
    if [[ -f "$BUILD_FAIL_STAMP" && "$(cat "$BUILD_FAIL_STAMP")" == "$head" ]]; then
        warn "plrd refresh: build of $head failed previously; not retrying automatically."
        warn "Fix the build (see earlier journal entries), then run: sudo rm $BUILD_FAIL_STAMP"
        return 0
    fi

    log "plrd refresh: building $head in $REPO_DIR"
    mkdir -p "$STATE_DIR"
    # Build from inside the repo (rustup discovers rust-toolchain.toml from
    # the working directory) and as the repo's owner, never root-in-checkout.
    # shellcheck disable=SC2016  # expansion happens inside the child bash
    local build_script='cd "$1" && export PATH="$HOME/.cargo/bin:$PATH" && exec cargo build --release -p plrd'
    if [[ "$owner" != "root" ]]; then
        if ! runuser -u "$owner" -- env "HOME=$owner_home" \
                bash -c "$build_script" refresh-build "$REPO_DIR"; then
            printf '%s\n' "$head" >"$BUILD_FAIL_STAMP"
            die "plrd refresh: build failed; the previous binary stays installed"
        fi
    else
        if ! bash -c "$build_script" refresh-build "$REPO_DIR"; then
            printf '%s\n' "$head" >"$BUILD_FAIL_STAMP"
            die "plrd refresh: build failed; the previous binary stays installed"
        fi
    fi

    # Atomic replace: a plain copy over a running binary fails (ETXTBSY).
    install -m755 "$REPO_DIR/target/release/plrd" "$BIN_PATH.new"
    mv -f "$BIN_PATH.new" "$BIN_PATH"
    printf '%s\n' "$head" >"$BUILD_STAMP"
    rm -f "$BUILD_FAIL_STAMP"
    log "plrd refresh: installed new binary ($("$BIN_PATH" version 2>/dev/null || echo unknown))"
}

if [[ $REFRESH_MODE -eq 1 ]]; then
    refresh
    exit 0
fi

# --- preflight --------------------------------------------------------------

preflight() {
    [[ "$(uname -s)" == "Linux" ]] \
        || die "plrd's recorder daemon is Linux-only; this installer must run on the printer host"

    if [[ -r /etc/os-release ]]; then
        # shellcheck source=/dev/null
        . /etc/os-release
        case "${ID:-} ${ID_LIKE:-}" in
            *debian*) : ;;
            *) warn "not a Debian-family OS (${PRETTY_NAME:-unknown}); continuing, but this is untested territory" ;;
        esac
    else
        warn "cannot read /etc/os-release; assuming a Debian-like system"
    fi

    if [[ $(id -u) -eq 0 ]]; then
        warn "running as root: the repo and toolchain will live under /root."
        warn "Recommended: run as your normal printer user (sudo is used only where needed)."
    fi

    if [[ $NO_SERVICE -eq 0 && ! -d /run/systemd/system ]]; then
        die "systemd is not running; use --no-service to build without installing the service"
    fi

    local free_kb
    free_kb="$(df -Pk "$HOME" | awk 'NR==2 {print $4}')"
    if [[ "$free_kb" -lt "$MIN_DISK_KB" ]]; then
        die "only $((free_kb / 1024)) MiB free in $HOME; at least 2 GiB is needed for the toolchain + build"
    elif [[ "$free_kb" -lt "$WARN_DISK_KB" ]]; then
        warn "$((free_kb / 1024)) MiB free in $HOME; the build should fit but it will be tight"
    fi

    local missing=()
    have git  || missing+=(git)
    have curl || missing+=(curl)
    have cc || have gcc || missing+=(build-essential)
    if [[ ${#missing[@]} -gt 0 ]]; then
        if [[ $NO_SERVICE -eq 1 ]]; then
            die "missing build prerequisites: ${missing[*]}.
--no-service promises not to use sudo, so install them first:
    sudo apt-get install -y ${missing[*]}"
        fi
        log "missing build prerequisites: ${missing[*]}"
        require_sudo "installing ${missing[*]}"
        if confirm "Install them now with apt-get?" y; then
            run_root apt-get update -qq
            run_root apt-get install -y "${missing[@]}"
        else
            die "cannot continue without: ${missing[*]}"
        fi
    fi
}

# Explain the sudo situation once, before the first privileged command.
SUDO_CHECKED=0
require_sudo() {
    local why="$1"
    [[ $(id -u) -eq 0 ]] && return 0
    have sudo || die "sudo is required for $why but is not installed"
    [[ $SUDO_CHECKED -eq 1 ]] && return 0
    SUDO_CHECKED=1
    if ! sudo -n true 2>/dev/null; then
        if [[ $INTERACTIVE -eq 1 ]]; then
            log "sudo will prompt for your password (needed for: $why)"
        else
            die "passwordless sudo is not available and there is no terminal to prompt on.
Re-run in an interactive SSH session, or use --no-service to build without installing."
        fi
    fi
}

# --- printer_data / klippy socket detection --------------------------------

SOCKET_PATH=""
SOCKET_FOUND=0

detect_printer_data() {
    if [[ -z "$PRINTER_DATA" ]]; then
        PRINTER_DATA="$HOME/printer_data"
    fi
    if [[ ! -d "$PRINTER_DATA" ]]; then
        warn "printer_data not found at $PRINTER_DATA (override with --printer-data)."
        warn "Continuing; the generated config will use the standard socket path."
    fi

    if [[ -S "$PRINTER_DATA/comms/klippy.sock" ]]; then
        SOCKET_PATH="$PRINTER_DATA/comms/klippy.sock"
        SOCKET_FOUND=1
    elif [[ -S /tmp/klippy_uds ]]; then
        SOCKET_PATH="/tmp/klippy_uds"
        SOCKET_FOUND=1
    else
        SOCKET_PATH="$PRINTER_DATA/comms/klippy.sock"
        warn "no Klipper API socket found (checked $PRINTER_DATA/comms/klippy.sock and /tmp/klippy_uds)."
        warn "Continuing with $SOCKET_PATH; plrd waits for the socket with backoff, so this is fine if Klipper is simply stopped."
    fi
    log "printer_data: $PRINTER_DATA"
    log "klipper socket: $SOCKET_PATH$( [[ $SOCKET_FOUND -eq 0 ]] && echo ' (not present yet)' )"
}

# --- rust toolchain ---------------------------------------------------------

ensure_rust() {
    export PATH="$HOME/.cargo/bin:$PATH"
    if have rustup; then
        log "rustup present; the pinned toolchain in rust-toolchain.toml installs on first build"
        return 0
    fi
    if have cargo; then
        # Distro cargo without rustup cannot honor rust-toolchain.toml.
        local req cur
        req="$(sed -n 's/^channel = "\(.*\)"/\1/p' "$REPO_DIR/rust-toolchain.toml" 2>/dev/null || true)"
        cur="$(rustc --version 2>/dev/null | awk '{print $2}')"
        warn "cargo found without rustup (rustc ${cur:-unknown}); the repo pins Rust ${req:-unknown} via rust-toolchain.toml."
        if [[ -n "$req" && -n "$cur" ]] && \
           [[ "$(printf '%s\n%s\n' "$req" "$cur" | sort -V | head -n1)" == "$req" ]]; then
            warn "system rustc $cur is >= $req; attempting the build with it"
            return 0
        fi
        die "system Rust is too old and rustup is absent. Install rustup: https://rustup.rs (or apt remove the distro rust first)"
    fi
    log "installing rustup (minimal profile, no default toolchain)"
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --profile minimal --default-toolchain none
    export PATH="$HOME/.cargo/bin:$PATH"
    have cargo || die "rustup installation did not put cargo on PATH ($HOME/.cargo/bin)"
}

# --- source checkout --------------------------------------------------------

SKIP_REPO_UPDATE=0

detect_repo_dir() {
    # When executed from inside a clone (not via curl), default to that
    # clone and do not move its branch — a developer's checkout is theirs.
    local script_path="${BASH_SOURCE[0]:-}" script_dir candidate
    if [[ -z "$REPO_DIR" && -n "$script_path" && -f "$script_path" ]]; then
        script_dir="$(cd "$(dirname "$script_path")" && pwd)"
        candidate="$(cd "$script_dir/.." && pwd)"
        if [[ -d "$candidate/.git" && -f "$candidate/crates/plrd/Cargo.toml" ]]; then
            REPO_DIR="$candidate"
            SKIP_REPO_UPDATE=1
            log "running from a checkout; using it as-is: $REPO_DIR"
        fi
    fi
    if [[ -z "$REPO_DIR" ]]; then
        REPO_DIR="$HOME/dead-reckoning"
    fi
    # If the script we are running lives inside the target repo, never
    # pull the rug out from under ourselves (or a developer's branch).
    if [[ $SKIP_REPO_UPDATE -eq 0 && -n "$script_path" && -f "$script_path" ]]; then
        script_dir="$(cd "$(dirname "$script_path")" && pwd)"
        case "$script_dir/" in
            "$REPO_DIR"/*) SKIP_REPO_UPDATE=1 ;;
        esac
    fi
}

fetch_source() {
    if [[ -d "$REPO_DIR/.git" ]]; then
        if [[ $SKIP_REPO_UPDATE -eq 1 ]]; then
            log "using existing checkout at $REPO_DIR (no fetch/checkout: script runs from inside it)"
            return 0
        fi
        log "updating existing checkout at $REPO_DIR"
        if [[ -n "$(git -C "$REPO_DIR" status --porcelain)" ]]; then
            die "$REPO_DIR has local changes. Commit or stash them (or pass --dir for a fresh location); this installer never clobbers local work."
        fi
        local origin
        origin="$(git -C "$REPO_DIR" remote get-url origin 2>/dev/null || true)"
        if [[ "$origin" != "$REPO_URL" ]]; then
            warn "origin is '$origin' (expected $REPO_URL); updating from it anyway"
        fi
        git -C "$REPO_DIR" fetch origin "$BRANCH"
        git -C "$REPO_DIR" checkout "$BRANCH"
        git -C "$REPO_DIR" pull --ff-only origin "$BRANCH" \
            || die "$REPO_DIR has diverged from origin/$BRANCH. Resolve it manually; this installer only fast-forwards."
    elif [[ -e "$REPO_DIR" ]]; then
        die "$REPO_DIR exists but is not a git repository; move it aside or pass --dir"
    else
        log "cloning $REPO_URL (branch $BRANCH) to $REPO_DIR"
        git clone --branch "$BRANCH" "$REPO_URL" "$REPO_DIR"
    fi
}

# --- build ------------------------------------------------------------------

build() {
    local jobs=() mem_kb
    mem_kb="$(awk '/MemTotal/ {print $2}' /proc/meminfo)"
    if [[ "$mem_kb" -lt "$LOW_MEM_KB" ]]; then
        warn "only $((mem_kb / 1024)) MiB RAM; building single-threaded (-j1) to avoid OOM"
        jobs=(-j1)
    fi
    log "building plrd (release). First builds also install the pinned toolchain; on a Pi this can take a while."
    if ! (cd "$REPO_DIR" && cargo build --release -p plrd "${jobs[@]+"${jobs[@]}"}"); then
        echo >&2
        warn "the build failed. Common causes on printer boards:"
        warn "  * out of disk:   $(df -Ph "$HOME" | awk 'NR==2 {print $4}') free in $HOME (need ~2 GiB)"
        warn "  * out of memory: $((mem_kb / 1024)) MiB RAM — retry with: cd $REPO_DIR && cargo build --release -p plrd -j1"
        warn "                   and/or add swap: sudo dphys-swapfile setup (Raspberry Pi OS)"
        warn "  * toolchain download interrupted — retry: rustup toolchain install"
        die "cargo build failed (see output above)"
    fi
    log "built $REPO_DIR/target/release/plrd ($("$REPO_DIR/target/release/plrd" version))"
}

# --- config generation ------------------------------------------------------

# Prints detected z stepper names, comma-joined, by scanning printer.cfg
# plus one level of [include ...] (glob-aware), matching Klipper sections
# [stepper_z], [stepper_z1], ... Falls back to "stepper_z" with a warning.
detect_z_steppers() {
    local cfg_dir="$PRINTER_DATA/config" main_cfg inc files=() f found
    main_cfg="$cfg_dir/printer.cfg"
    if [[ ! -f "$main_cfg" ]]; then
        warn "no printer.cfg at $main_cfg; defaulting z_steppers to 'stepper_z' — edit $CONF_PATH if your machine has more"
        echo "stepper_z"
        return 0
    fi
    files=("$main_cfg")
    while IFS= read -r inc; do
        # Klipper resolves includes relative to the including file's dir
        # and allows wildcards.
        while IFS= read -r f; do
            if [[ -f "$f" ]]; then
                files+=("$f")
            fi
        done < <(compgen -G "$cfg_dir/$inc" || true)
    done < <(sed -n 's/^\[include[[:space:]]\{1,\}\([^]]*\)\].*/\1/p' "$main_cfg")

    found="$(grep -h -o '^\[stepper_z[0-9]*\]' "${files[@]}" 2>/dev/null \
        | tr -d '[]' | sort -u -V | paste -sd, - || true)"
    if [[ -z "$found" ]]; then
        warn "no [stepper_z*] sections found in printer.cfg (or its includes); defaulting to 'stepper_z'"
        echo "stepper_z"
    else
        echo "$found"
    fi
}

GENERATED_CONF=""

generate_config() {
    local z_steppers example tmp
    example="$REPO_DIR/deploy/plrd.conf.example"
    [[ -f "$example" ]] || die "missing $example in the checkout (corrupt clone?)"

    if [[ -e "$CONF_PATH" && $FORCE_CONFIG -eq 0 ]]; then
        log "$CONF_PATH already exists — leaving it untouched (use --force-config to regenerate)"
        return 0
    fi

    z_steppers="$(detect_z_steppers)"
    log "detected z steppers: $z_steppers"

    tmp="$(mktemp)"
    awk -v sock="$SOCKET_PATH" -v zs="$z_steppers" '
        /^klipper_socket = / { print "klipper_socket = " sock; next }
        /^z_steppers = /     { print "z_steppers = " zs; next }
        { print }
    ' "$example" >"$tmp"
    GENERATED_CONF="$tmp"

    echo
    log "generated configuration (destination: $CONF_PATH):"
    sed 's/^/    /' "$tmp"
    echo
}

# --- install ----------------------------------------------------------------

SERVICE_WAS_ACTIVE=0

install_files() {
    require_sudo "installing plrd (binary, config, systemd unit)"

    if systemctl is-active --quiet plrd 2>/dev/null; then
        SERVICE_WAS_ACTIVE=1
    fi

    # Atomic binary replace: `install` straight onto a running binary can
    # hit ETXTBSY; staging + rename never does.
    run_root install -m755 "$REPO_DIR/target/release/plrd" "$BIN_PATH.new"
    run_root mv -f "$BIN_PATH.new" "$BIN_PATH"
    log "installed $BIN_PATH"

    if [[ -n "$GENERATED_CONF" ]]; then
        if [[ -e "$CONF_PATH" ]]; then
            local backup
            backup="$CONF_PATH.bak.$(date +%Y%m%d-%H%M%S)"
            run_root cp -p "$CONF_PATH" "$backup"
            log "backed up existing config to $backup"
        fi
        run_root install -m644 "$GENERATED_CONF" "$CONF_PATH"
        rm -f "$GENERATED_CONF"
        log "installed $CONF_PATH"
    fi

    run_root install -m644 "$REPO_DIR/deploy/plrd.service" "$UNIT_PATH"
    log "installed $UNIT_PATH"

    run_root systemctl daemon-reload
    run_root systemctl enable plrd
    # Restart (not just start): a re-run must swap in the new binary.
    # Restarting plrd is safe at any time — it observes Klipper, it does
    # not control it.
    run_root systemctl restart plrd
    if [[ $SERVICE_WAS_ACTIVE -eq 1 ]]; then
        log "plrd service restarted with the new binary"
    else
        log "plrd service enabled and started"
    fi

    # Record what the installed binary was built from so the Moonraker
    # refresh hook can no-op until the repo actually changes.
    run_root mkdir -p "$STATE_DIR"
    git -C "$REPO_DIR" rev-parse HEAD | run_root tee "$BUILD_STAMP" >/dev/null
    run_root rm -f "$BUILD_FAIL_STAMP"
}

# --- moonraker integration --------------------------------------------------
#
# Two documented pieces (moonraker.readthedocs.io):
#  * [update_manager plrd] git_repo section in moonraker.conf. The section
#    name must equal the systemd unit name: managed_services only accepts
#    the section name, "klipper", or "moonraker".
#  * "plrd" line in <printer_data>/moonraker.asvc — the allow-list of
#    services Moonraker may manage (case-sensitive).
# Moonraker runs no build steps for git_repo entries (install_script is
# deprecated and parse-only), so a systemd drop-in rebuilds via
# `install.sh --refresh` when Moonraker restarts plrd after an update.

moonraker_conf_snippet() {
    local origin
    origin="$(git -C "$REPO_DIR" remote get-url origin 2>/dev/null || echo "$REPO_URL")"
    cat <<EOF

[update_manager plrd]
# Added by dead-reckoning scripts/install.sh on $(date +%Y-%m-%d).
# Remove with scripts/uninstall.sh --moonraker.
type: git_repo
path: $REPO_DIR
origin: $origin
primary_branch: $BRANCH
managed_services: plrd
info_tags:
    desc=dead-reckoning power-loss recovery recorder
EOF
}

setup_moonraker() {
    local mrconf="$PRINTER_DATA/config/moonraker.conf"
    local asvc="$PRINTER_DATA/moonraker.asvc"
    local stamp changed=0
    stamp="$(date +%Y%m%d-%H%M%S)"

    if [[ ! -f "$mrconf" ]]; then
        warn "no moonraker.conf at $mrconf — skipping Moonraker integration."
        warn "Manual setup: see deploy/moonraker-update-manager.conf in the repo."
        return 0
    fi

    if grep -q '^\[update_manager plrd\]' "$mrconf"; then
        log "moonraker.conf already has [update_manager plrd] — not touching it"
    else
        cp -p "$mrconf" "$mrconf.plrd-bak.$stamp"
        log "backed up moonraker.conf to $mrconf.plrd-bak.$stamp"
        moonraker_conf_snippet >>"$mrconf"
        log "appended [update_manager plrd] to $mrconf"
        changed=1
    fi

    # moonraker.asvc: newline-separated allow-list of extra services
    # Moonraker may manage; unit names are case-sensitive.
    if [[ -f "$asvc" ]] && grep -qx 'plrd' "$asvc"; then
        log "moonraker.asvc already allows plrd"
    else
        if [[ -f "$asvc" ]]; then
            cp -p "$asvc" "$asvc.plrd-bak.$stamp"
        else
            warn "$asvc did not exist (old Moonraker or non-standard data path?); creating it"
        fi
        printf 'plrd\n' >>"$asvc"
        log "allow-listed plrd in $asvc"
        changed=1
    fi

    # Rebuild-on-update hook: Moonraker pulls the repo then restarts plrd;
    # this drop-in makes that restart rebuild first. "-" = a failed build
    # never blocks startup (the old binary keeps running); "+" = run
    # outside the unit's sandbox (ProtectSystem=strict would forbid the
    # build). TimeoutStartSec covers slow Pi rebuilds; normal starts are
    # still ready in well under a second.
    require_sudo "installing the plrd rebuild-on-update drop-in"
    local tmp
    tmp="$(mktemp)"
    cat >"$tmp" <<EOF
# Installed by dead-reckoning scripts/install.sh --moonraker.
# Rebuilds plrd from $REPO_DIR when the service restarts after a
# Moonraker update; a fast no-op when nothing changed.
[Service]
ExecStartPre=-+/usr/bin/env bash $REPO_DIR/scripts/install.sh --refresh --dir $REPO_DIR
TimeoutStartSec=3600
EOF
    run_root install -d "$DROPIN_DIR"
    run_root install -m644 "$tmp" "$DROPIN_PATH"
    rm -f "$tmp"
    run_root systemctl daemon-reload
    log "installed $DROPIN_PATH"

    echo
    log "Moonraker integration staged. Restart Moonraker yourself to pick it up:"
    echo "      sudo systemctl restart moonraker    (or use the web UI)"
    echo "    This installer deliberately never restarts moonraker or klipper."
    if [[ $changed -eq 0 ]]; then
        log "(no moonraker file changes were needed this run)"
    fi
}

# --- verification -----------------------------------------------------------

verify() {
    local ok=1 state
    state="$(systemctl is-active plrd 2>/dev/null || true)"
    for _ in 1 2 3 4 5; do
        [[ "$state" == "active" ]] && break
        sleep 1
        state="$(systemctl is-active plrd 2>/dev/null || true)"
    done
    if [[ "$state" == "active" ]]; then
        log "service: active"
    else
        ok=0
        warn "service state is '$state' — check: journalctl -u plrd -n 50"
    fi

    for _ in 1 2 3; do
        if run_root sh -c "ls -A '$WAL_DIR' 2>/dev/null" | grep -q .; then
            break
        fi
        sleep 1
    done
    if run_root sh -c "ls -A '$WAL_DIR' 2>/dev/null" | grep -q .; then
        log "WAL: files present in $WAL_DIR"
    else
        ok=0
        warn "no WAL files in $WAL_DIR after a few seconds — check: journalctl -u plrd -n 50"
    fi

    log "version: $("$BIN_PATH" version)"

    echo
    if [[ $ok -eq 1 ]]; then
        log "install verified."
    else
        warn "install finished with warnings — see above."
    fi
    if [[ $SOCKET_FOUND -eq 0 ]]; then
        echo "    Note: no Klipper socket exists yet, so plrd is retrying the"
        echo "    connection with backoff. That is its normal, healthy state"
        echo "    whenever Klipper is down; it attaches by itself when Klipper"
        echo "    starts. Watch it with: journalctl -u plrd -f"
    fi
}

summary() {
    echo
    echo "==================================================================="
    echo " plrd is installed."
    echo
    echo "   binary   : $BIN_PATH"
    echo "   config   : $CONF_PATH"
    echo "   service  : plrd.service (systemctl status plrd)"
    echo "   WAL      : $WAL_DIR"
    echo "   source   : $REPO_DIR"
    if [[ -n "$PLUGIN_LINK" ]]; then
        echo "   plugin   : $PLUGIN_LINK"
    else
        echo "   plugin   : NOT linked (no Klipper checkout found — see the"
        echo "              warning above for the manual ln -sfn command)"
    fi
    echo
    echo " plrd only *reads* from Klipper (its API socket) and writes its"
    echo " own WAL — it never sends commands, so it is safe to run alongside"
    echo " prints, including right now."
    echo
    echo " Activate the console plugin (the PLR_* commands). This installer"
    echo " never edits printer.cfg and never restarts Klipper, so both"
    echo " steps are yours:"
    echo "   1. add a [plr] section to printer.cfg — minimal starter:"
    echo
    echo "        [plr]"
    echo "        probe_method: tap   # or load_cell; adxl_drag needs accel_chip too"
    echo
    echo "      (commented starter block: examples/printer-plr-section.cfg;"
    echo "       full reference: klippy_plugin/README.md)"
    echo "   2. RESTART Klipper (console RESTART, or your web UI), then"
    echo "   3. type PLR_SETUP in the console and follow its report."
    echo
    echo " Other next steps:"
    echo "   * review the config:        sudo nano $CONF_PATH"
    echo "     (then: sudo systemctl restart plrd — restarting plrd is safe)"
    echo "   * watch it attach:          journalctl -u plrd -f"
    echo "   * after a power loss:       type PLR_RECOVER in the console"
    echo "                               (or: plrd scan --wal $WAL_DIR)"
    if [[ $DID_MOONRAKER -eq 1 ]]; then
        echo "   * restart Moonraker to show plrd in the update UI:"
        echo "                               sudo systemctl restart moonraker"
    else
        echo "   * optional Moonraker update-manager integration:"
        echo "                               re-run this installer with --moonraker"
    fi
    echo "==================================================================="
}

# --- main -------------------------------------------------------------------

DID_MOONRAKER=0

main() {
    local ensure_rust_after_clone
    detect_repo_dir
    preflight
    detect_printer_data
    ensure_rust_after_clone=0
    # rust-toolchain.toml is read from the checkout, so make sure the
    # checkout exists before deciding how to get Rust.
    if [[ ! -f "$REPO_DIR/rust-toolchain.toml" ]]; then
        ensure_rust_after_clone=1
    fi
    if [[ $ensure_rust_after_clone -eq 0 ]]; then
        ensure_rust
    fi
    fetch_source
    if [[ $ensure_rust_after_clone -eq 1 ]]; then
        ensure_rust
    fi
    build
    generate_config

    if [[ $NO_SERVICE -eq 1 ]]; then
        local staged="$REPO_DIR/target/plrd.conf.generated"
        if [[ -n "$GENERATED_CONF" ]]; then
            mv "$GENERATED_CONF" "$staged"
        fi
        echo
        log "--no-service: nothing was installed. Staged artifacts:"
        echo "      binary : $REPO_DIR/target/release/plrd"
        [[ -n "$GENERATED_CONF" ]] && echo "      config : $staged (destination: $CONF_PATH)"
        echo "      unit   : $REPO_DIR/deploy/plrd.service (destination: $UNIT_PATH)"
        echo "    Manual install commands are in the header of deploy/plrd.service."
        echo "    The klippy console plugin was also NOT linked; do it yourself with:"
        echo "      ln -sfn $REPO_DIR/klippy_plugin/plr <klipper>/klippy/extras/plr"
        return 0
    fi

    install_files
    ensure_plugin_symlink "$HOME"

    if [[ $WANT_MOONRAKER -eq 1 ]] || confirm "Register plrd with Moonraker's update manager (edits moonraker.conf + moonraker.asvc, with backups)?" n; then
        setup_moonraker
        DID_MOONRAKER=1
    else
        log "skipping Moonraker integration (re-run with --moonraker to add it later)"
    fi

    verify
    summary
}

main
