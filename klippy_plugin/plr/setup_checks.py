"""Pre-flight environment and configuration checks.

Backs the planned ``PLR_CHECK_SETUP`` command: verifying that the plrd
daemon is reachable, that the printer objects the plugin depends on
exist, and that optional dependencies are present before any diagnostic
runs.  numpy is optional inside klippy (it is present on installs that
use input_shaper/resonance tooling); commands that need it must degrade
with a clear installation hint instead of a traceback, which
:func:`require_numpy` provides.
"""

import importlib.util

NUMPY_HINT = (
    "numpy is required for this command but is not installed in the "
    "klippy environment; install it the same way Klipper's resonance "
    "tooling does (e.g. ~/klippy-env/bin/pip install numpy) and restart "
    "Klipper"
)


def numpy_available():
    """Return True if numpy can be imported in this environment."""
    return importlib.util.find_spec("numpy") is not None


def require_numpy(error_type=RuntimeError):
    """Raise ``error_type`` with a clear install hint if numpy is absent.

    ``error_type`` lets callers raise klippy's command error (e.g.
    ``gcode.error``) so the message reaches the console, not the log.
    """
    if not numpy_available():
        raise error_type(NUMPY_HINT)
