"""Production-import-mechanics: import plr exactly the way klippy does.

Every other test module in this suite does ``import plr`` (or ``from plr
import ...``) and it just works, because ``pyproject.toml``'s
``pythonpath = [".", "tests"]`` puts ``klippy_plugin/`` itself on
``sys.path`` -- so pytest resolves ``plr`` as an ordinary top-level
package.  Real klippy never does that.  Klippy adds its own ``klippy/``
directory to ``sys.path`` and loads config-section plugins as
``extras.<name>``; the plugin is wired in as a single symlink,
``<klipper>/klippy/extras/plr -> <repo>/klippy_plugin/plr``
(``scripts/install.sh``).  There is no top-level ``plr`` module in that
process at all.

That gap means an absolute self-import buried anywhere inside the
package (``import plr`` / ``from plr import ...``, as opposed to a
relative ``from . import ...``) can pass every one of the other 1071
tests here and still be dead on arrival in production -- which is
exactly what happened: ``calibration_meta.plugin_version()`` did a bare
``import plr`` and halted klippy with ``ModuleNotFoundError: No module
named 'plr'`` the first time anything called it.

This module rebuilds the production layout in a temp directory (a
synthetic ``extras`` package whose ``plr`` submodule IS the real source
tree) with the top-level ``plr`` name scrubbed from ``sys.path`` and
``sys.modules``, then imports ``extras.plr`` and every one of its
submodules exactly as klippy's module loader would.  That kills the
whole bug class: any future absolute self-import anywhere in ``plr/``
fails this test immediately, on both Windows (the pre-commit hook) and
Linux (CI) -- symlinks aren't guaranteed on Windows, so the fixture
falls back to a plain copy there.
"""

import importlib
import shutil
import sys
from pathlib import Path

import pytest

PLR_SRC = Path(__file__).resolve().parent.parent / "plr"


def _place_plr_package(extras_dir):
    """Populate ``extras_dir/plr`` with the real plugin source.

    Prefers a symlink -- the actual production wiring
    (``scripts/install.sh``) -- and falls back to a copy where symlinks
    aren't available (e.g. Windows without Developer Mode / admin
    rights), which is only a fidelity compromise for this test: either
    way ``extras.plr`` ends up backed by the real ``plr/`` source.
    """
    dest = extras_dir / "plr"
    try:
        dest.symlink_to(PLR_SRC, target_is_directory=True)
    except OSError:
        shutil.copytree(
            PLR_SRC, dest, ignore=shutil.ignore_patterns("__pycache__", "*.pyc")
        )


@pytest.fixture
def klippy_style_import(tmp_path, monkeypatch):
    """Arrange sys.path / sys.modules the way klippy's loader does.

    Scrubs ``klippy_plugin/`` (pytest's own ``pythonpath = "."``) out of
    ``sys.path`` and drops any already-imported top-level ``plr`` from
    ``sys.modules`` -- otherwise a cached module from an earlier test in
    this same run would paper over exactly the bug this test exists to
    catch -- then puts a synthetic ``extras`` package (real ``plr``
    source underneath) at the front of ``sys.path`` instead.
    """
    extras_dir = tmp_path / "extras"
    extras_dir.mkdir()
    (extras_dir / "__init__.py").write_text("")
    _place_plr_package(extras_dir)

    pre_existing = set(sys.modules)
    for name in list(sys.modules):
        if name == "plr" or name.startswith("plr."):
            monkeypatch.delitem(sys.modules, name, raising=False)

    scrubbed = [p for p in sys.path if Path(p).resolve() != PLR_SRC.parent]
    monkeypatch.setattr(sys, "path", [str(tmp_path)] + scrubbed)
    importlib.invalidate_caches()

    yield

    # sys.path / the deleted `plr*` entries are restored by monkeypatch;
    # the `extras*` entries this test added are new keys monkeypatch
    # never touched, so drop them ourselves.
    for name in list(sys.modules):
        if name not in pre_existing and (
            name == "extras" or name.startswith("extras.")
        ):
            del sys.modules[name]
    importlib.invalidate_caches()


def test_top_level_plr_is_not_importable(klippy_style_import):
    """Sanity-check the fixture itself.

    If this fails, the scrub is leaky and a pass in the test below would
    prove nothing -- it could just be falling back to a top-level `plr`
    the way pytest normally would.
    """
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("plr")


def test_extras_plr_imports_the_way_klippy_does(klippy_style_import):
    """The class-killer: import the package, and every submodule, the
    way klippy's extras loader actually does.

    Fails on unfixed source with::

        ModuleNotFoundError: No module named 'plr'

    raised out of ``calibration_meta.py``'s ``import plr`` inside
    ``plugin_version()``. Passes once every self-import inside ``plr/``
    names its own package relatively instead of assuming a top-level
    ``plr`` exists.
    """
    package = importlib.import_module("extras.plr")
    assert package.__version__

    # Every submodule, not just calibration_meta -- so a *future*
    # absolute self-import anywhere else in the package fails this test
    # too, not just the one that happened to break this time.
    submodule_names = sorted(
        p.stem for p in PLR_SRC.glob("*.py") if p.stem != "__init__"
    )
    assert submodule_names, "expected plr/ to contain submodules"
    for name in submodule_names:
        importlib.import_module("extras.plr." + name)

    # The actual call site that halted the printer (plugin.py:417):
    # reading the plugin version off the freshly loaded package.
    calibration_meta = sys.modules["extras.plr.calibration_meta"]
    assert calibration_meta.plugin_version() == package.__version__
