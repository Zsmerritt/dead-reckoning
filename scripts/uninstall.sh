#!/usr/bin/env bash
# dead-reckoning uninstaller: removes the plrd recorder service and
# binary. Configuration and the WAL (which holds power-loss RECOVERY
# DATA) are kept unless you explicitly ask for their removal.
#
#   curl -sSL https://raw.githubusercontent.com/Zsmerritt/dead-reckoning/main/scripts/uninstall.sh | bash
#
# Safety: only the plrd service is touched. The klipper and moonraker
# services are never stopped, started, or restarted; moonraker.conf and
# moonraker.asvc are only edited with --moonraker (backed up first) —
# otherwise you get exact manual instructions. The klippy plugin
# symlink (<klipper>/klippy/extras/plr) is removed only when it
# actually points into a dead-reckoning checkout — a foreign plr module
# is never deleted. printer.cfg is never edited: removing the [plr]
# section (and its autosave block) is yours, with a reminder printed.
set -euo pipefail

BIN_PATH="/usr/local/bin/plrd"
CONF_PATH="/etc/plrd.conf"
UNIT_PATH="/etc/systemd/system/plrd.service"
DROPIN_DIR="/etc/systemd/system/plrd.service.d"
STATE_DIR="/var/lib/plrd"
WAL_DIR="$STATE_DIR/wal"

ASSUME_YES=0
PURGE=0
EDIT_MOONRAKER=0
PRINTER_DATA=""
KLIPPER_DIR=""

usage() {
    cat <<'EOF'
dead-reckoning uninstaller (plrd)

USAGE:
    uninstall.sh [OPTIONS]

OPTIONS:
    --printer-data <path>  Klipper printer_data directory, used to locate
                           moonraker.conf / moonraker.asvc
                           (default: ~/printer_data)
    --klipper <path>       Klipper source checkout, used to locate the
                           klippy/extras/plr plugin symlink
                           (default: ~/klipper)
    --moonraker            Also remove the [update_manager plrd] section
                           from moonraker.conf and the plrd line from
                           moonraker.asvc (timestamped backups first).
                           Default: leave them and print manual steps.
    --purge                Also remove /etc/plrd.conf and /var/lib/plrd.
                           WARNING: /var/lib/plrd/wal holds the recovery
                           data for the last print — purging it forfeits
                           any pending power-loss recovery.
    --yes                  Non-interactive: accept the default answer for
                           every prompt (defaults KEEP config and WAL)
    --help                 Show this help

The uninstaller never touches the klipper or moonraker services.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --printer-data)
            [[ $# -ge 2 ]] || { echo "error: --printer-data requires a value" >&2; exit 2; }
            PRINTER_DATA="$2"; shift 2 ;;
        --klipper)
            [[ $# -ge 2 ]] || { echo "error: --klipper requires a value" >&2; exit 2; }
            KLIPPER_DIR="$2"; shift 2 ;;
        --moonraker) EDIT_MOONRAKER=1; shift ;;
        --purge)     PURGE=1; shift ;;
        --yes|-y)    ASSUME_YES=1; shift ;;
        --help|-h)   usage; exit 0 ;;
        *)
            echo "error: unknown option '$1' (see --help)" >&2
            exit 2 ;;
    esac
done

if [[ -t 1 ]]; then
    C_STEP=$'\033[1;32m'; C_WARN=$'\033[1;33m'; C_ERR=$'\033[1;31m'; C_OFF=$'\033[0m'
else
    C_STEP=""; C_WARN=""; C_ERR=""; C_OFF=""
fi

log()  { printf '%s==>%s %s\n' "$C_STEP" "$C_OFF" "$*"; }
warn() { printf '%sWARNING:%s %s\n' "$C_WARN" "$C_OFF" "$*" >&2; }
die()  { printf '%sERROR:%s %s\n' "$C_ERR" "$C_OFF" "$*" >&2; exit 1; }

INTERACTIVE=0
if { : </dev/tty; } 2>/dev/null; then
    INTERACTIVE=1
fi

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

run_root() {
    if [[ $(id -u) -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

require_sudo() {
    [[ $(id -u) -eq 0 ]] && return 0
    command -v sudo >/dev/null 2>&1 || die "sudo is required to remove installed files"
    if ! sudo -n true 2>/dev/null; then
        if [[ $INTERACTIVE -eq 1 ]]; then
            log "sudo will prompt for your password"
        else
            die "passwordless sudo is not available and there is no terminal to prompt on; re-run in an interactive session"
        fi
    fi
}

REMOVED=()
KEPT=()
PLR_CFG_REMINDER=0

main() {
    [[ "$(uname -s)" == "Linux" ]] || die "this uninstaller must run on the printer host (Linux)"
    [[ -z "$PRINTER_DATA" ]] && PRINTER_DATA="$HOME/printer_data"

    require_sudo

    # --- service ------------------------------------------------------------
    if [[ -f "$UNIT_PATH" ]]; then
        log "stopping and disabling plrd.service"
        run_root systemctl disable --now plrd || warn "disable --now plrd failed (continuing)"
        run_root rm -f "$UNIT_PATH"
        REMOVED+=("$UNIT_PATH (service stopped and disabled)")
    else
        log "no $UNIT_PATH — service not installed"
    fi
    if [[ -d "$DROPIN_DIR" ]]; then
        run_root rm -rf "$DROPIN_DIR"
        REMOVED+=("$DROPIN_DIR (rebuild-on-update drop-in)")
    fi
    if [[ -d /run/systemd/system ]]; then
        run_root systemctl daemon-reload
    fi

    # --- binary -------------------------------------------------------------
    if [[ -e "$BIN_PATH" ]]; then
        run_root rm -f "$BIN_PATH"
        REMOVED+=("$BIN_PATH")
    else
        log "no $BIN_PATH — binary not installed"
    fi

    # --- klippy plugin symlink ----------------------------------------------
    # Removed ONLY when it is a symlink pointing into a dead-reckoning
    # checkout (target ends in klippy_plugin/plr). A regular directory,
    # or a symlink to anything else, was not created by our installer
    # and is never deleted — someone else's plr module is theirs.
    local klipper="" plr_link plr_target
    if [[ -n "$KLIPPER_DIR" ]]; then
        klipper="$KLIPPER_DIR"
    elif [[ -d "$HOME/klipper/klippy/extras" ]]; then
        klipper="$HOME/klipper"
    fi
    if [[ -n "$klipper" ]]; then
        plr_link="$klipper/klippy/extras/plr"
        if [[ -L "$plr_link" ]]; then
            plr_target="$(readlink "$plr_link")"
            case "${plr_target%/}" in
                */klippy_plugin/plr)
                    rm -f "$plr_link"
                    REMOVED+=("$plr_link (klippy plugin symlink -> $plr_target)")
                    PLR_CFG_REMINDER=1
                    ;;
                *)
                    KEPT+=("$plr_link (symlink to $plr_target — not a dead-reckoning checkout; never deleting a foreign plugin)")
                    ;;
            esac
        elif [[ -e "$plr_link" ]]; then
            KEPT+=("$plr_link (not a symlink, so not created by our installer)")
        else
            log "no klippy plugin symlink at $plr_link"
        fi
    else
        log "no Klipper checkout at ~/klipper — if the plugin is linked elsewhere, re-run with --klipper <path> (or remove <klipper>/klippy/extras/plr yourself)"
    fi

    # --- config (kept by default) ------------------------------------------
    if [[ -e "$CONF_PATH" ]]; then
        if [[ $PURGE -eq 1 ]] || confirm "Remove $CONF_PATH?" n; then
            run_root rm -f "$CONF_PATH"
            REMOVED+=("$CONF_PATH")
        else
            KEPT+=("$CONF_PATH (config; harmless to keep)")
        fi
    fi

    # --- WAL / state (kept by default: it is the recovery data) -------------
    if [[ -d "$STATE_DIR" ]]; then
        warn "$WAL_DIR contains plrd's power-loss RECOVERY DATA for the last print."
        warn "If a print died with the power, deleting it forfeits recovery."
        if [[ $PURGE -eq 1 ]] || confirm "Remove $STATE_DIR (including the WAL)?" n; then
            run_root rm -rf "$STATE_DIR"
            REMOVED+=("$STATE_DIR (WAL + recovery data)")
        else
            KEPT+=("$STATE_DIR (WAL + recovery data)")
        fi
    fi

    # --- moonraker integration ----------------------------------------------
    local mrconf="$PRINTER_DATA/config/moonraker.conf"
    local asvc="$PRINTER_DATA/moonraker.asvc"
    local stamp mr_left=0
    stamp="$(date +%Y%m%d-%H%M%S)"

    if [[ $EDIT_MOONRAKER -eq 1 ]]; then
        if [[ -f "$mrconf" ]] && grep -q '^\[update_manager plrd\]' "$mrconf"; then
            cp -p "$mrconf" "$mrconf.plrd-bak.$stamp"
            # Drop the section: from its header up to (not including) the
            # next section header.
            awk '
                /^\[update_manager plrd\][[:space:]]*$/ { drop = 1; next }
                /^\[/                                   { drop = 0 }
                !drop
            ' "$mrconf.plrd-bak.$stamp" >"$mrconf"
            REMOVED+=("[update_manager plrd] section in $mrconf (backup: $mrconf.plrd-bak.$stamp)")
        else
            log "no [update_manager plrd] section in $mrconf"
        fi
        if [[ -f "$asvc" ]] && grep -qx 'plrd' "$asvc"; then
            cp -p "$asvc" "$asvc.plrd-bak.$stamp"
            grep -vx 'plrd' "$asvc.plrd-bak.$stamp" >"$asvc" || true
            REMOVED+=("plrd line in $asvc (backup: $asvc.plrd-bak.$stamp)")
        else
            log "no plrd line in $asvc"
        fi
        log "restart Moonraker yourself to apply (we never restart it): sudo systemctl restart moonraker"
    else
        if [[ -f "$mrconf" ]] && grep -q '^\[update_manager plrd\]' "$mrconf"; then
            mr_left=1
        fi
        if [[ -f "$asvc" ]] && grep -qx 'plrd' "$asvc"; then
            mr_left=1
        fi
    fi

    # --- summary ------------------------------------------------------------
    echo
    echo "==================================================================="
    echo " plrd uninstall complete."
    echo
    if [[ ${#REMOVED[@]} -gt 0 ]]; then
        echo " Removed:"
        printf '   - %s\n' "${REMOVED[@]}"
    else
        echo " Removed: nothing (nothing was installed)"
    fi
    if [[ ${#KEPT[@]} -gt 0 ]]; then
        echo
        echo " Deliberately kept:"
        printf '   - %s\n' "${KEPT[@]}"
        echo "   (remove later by re-running with --purge)"
    fi
    if [[ $PLR_CFG_REMINDER -eq 1 ]]; then
        echo
        echo " The [plr] section in printer.cfg was NOT touched (this script"
        echo " never edits printer.cfg). Before the next Klipper restart:"
        echo "   1. delete the [plr] section from printer.cfg, AND"
        echo "   2. delete the autosaved '#*# [plr]' block at the bottom of"
        echo "      the file (self_locking_z, probe_resolution,"
        echo "      noise_floor_*), if present."
        echo " With the plugin unlinked, a leftover [plr] section makes"
        echo " klippy fail its config load at startup."
    fi
    if [[ $mr_left -eq 1 ]]; then
        echo
        echo " Moonraker still references plrd. Either re-run with --moonraker,"
        echo " or remove by hand:"
        echo "   1. Edit $mrconf"
        echo "      and delete the whole [update_manager plrd] section (the"
        echo "      header line and every line below it until the next"
        echo "      [section] header)."
        echo "   2. Edit $asvc"
        echo "      and delete the line that reads exactly: plrd"
        echo "   3. Restart Moonraker (sudo systemctl restart moonraker, or"
        echo "      the web UI). This script never restarts it for you."
    fi
    echo
    echo " The source checkout (e.g. ~/dead-reckoning) was not touched."
    echo "==================================================================="
}

main
