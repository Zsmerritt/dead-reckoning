"""The off-reactor call channel itself: its guarantees, one per test.

``AsyncDaemon`` is the only place in the plugin that touches a thread, so
the properties it promises are the ones that keep the fix from being worse
than the bug it fixes:

* one call in flight per channel (no thread storm, no forked conversation);
* nothing from a stale or post-shutdown call is ever delivered — a worker
  thread must never touch a printer that has gone away;
* no exception ever escapes a reactor callback, because klippy turns one
  into a printer shutdown (klippy/klippy.py:170-186);
* worker threads are daemon threads, so a socket read blocked in ``recv``
  cannot stop klippy exiting or restarting.

Real threads throughout: a fake that called the worker inline would test
nothing that matters here.
"""

import threading
import time

import fake_klippy
import pytest

from plr import daemon_link, daemon_worker


class Link:
    """A link whose call can be blocked, made to fail, or made to explode."""

    def __init__(self, response=None, error=None, block=None):
        self.response = response if response is not None else {"ok": True}
        self.error = error
        self.block = block
        self.calls = []
        self.entered = threading.Event()

    def call(self, cmd, args=None, timeout=None):
        self.calls.append((cmd, args, timeout))
        self.entered.set()
        if self.block is not None:
            self.block.wait(5.0)
        if self.error is not None:
            raise self.error
        return self.response


@pytest.fixture
def channel(fake_printer):
    holder = {}

    def build(link):
        holder["link"] = link
        return daemon_worker.AsyncDaemon(fake_printer, lambda: holder["link"], "test")

    return build


def collect():
    results, errors = [], []
    return results, errors, results.append, errors.append


def test_a_result_comes_back_on_the_reactor_not_the_worker(fake_printer, channel):
    link = Link(response={"ok": True, "text": "hi", "data": {}})
    daemon = channel(link)
    results, errors, on_result, on_error = collect()
    assert daemon.call("status", None, 1.0, on_result, on_error) is True
    # The call is in flight and nothing has been delivered: the reactor has
    # not been asked to do anything yet.
    link.entered.wait(2.0)
    assert results == [] and errors == []
    # ...it is waiting in the reactor's async queue, exactly where
    # klippy/reactor.py:199-205 puts it.
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert results == [{"ok": True, "text": "hi", "data": {}}]
    assert errors == []
    assert daemon.is_busy() is False


def test_a_daemon_error_is_delivered_to_the_error_callback(fake_printer, channel):
    error = daemon_link.DaemonError("plrd not reachable at /x")
    daemon = channel(Link(error=error))
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert results == []
    assert errors == [error]


def test_an_unexpected_exception_becomes_a_daemon_error_not_a_lost_thread(
    fake_printer, channel
):
    # A non-DaemonError out of the link is a plugin bug.  It must still
    # reach the operator, not die on a thread nobody is watching.
    daemon = channel(Link(error=ValueError("bad json in a place we forgot")))
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert results == []
    (err,) = errors
    assert isinstance(err, daemon_link.DaemonError)
    assert "internal error talking to plrd (ValueError)" in str(err)


def test_only_one_call_is_in_flight_per_channel(fake_printer, channel):
    release = threading.Event()
    link = Link(block=release)
    daemon = channel(link)
    results, errors, on_result, on_error = collect()
    try:
        assert daemon.call("status", None, 1.0, on_result, on_error) is True
        link.entered.wait(2.0)
        assert daemon.is_busy() is True
        # A second call is REFUSED, not queued and not run: an operator
        # leaning on a button must not be able to fork the conversation or
        # spawn threads against a hung daemon.
        assert daemon.call("status", None, 1.0, on_result, on_error) is False
        assert len(link.calls) == 1
    finally:
        release.set()
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    # ...and the channel is free again once the answer has been delivered.
    assert daemon.is_busy() is False
    assert daemon.call("status", None, 1.0, on_result, on_error) is True
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1


def test_a_cancelled_calls_answer_is_dropped(fake_printer, channel):
    release = threading.Event()
    link = Link(block=release)
    daemon = channel(link)
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    link.entered.wait(2.0)
    daemon.cancel()
    # The slot is NOT freed by cancelling: the orphan still holds a socket
    # until its own deadline, so a second call would be a second thread.
    assert daemon.is_busy() is True
    assert daemon.call("status", None, 1.0, on_result, on_error) is False
    release.set()
    # The worker still finishes and still hands its result to the reactor —
    # it cannot be interrupted mid-recv — but the callback is not run, and
    # THAT is where the slot frees.
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert results == [] and errors == []
    assert daemon.is_busy() is False


def test_cancelling_in_a_loop_cannot_pile_up_workers(fake_printer, channel):
    # The amplifier: `wizard._reset()` cancels on every dismissal, so a
    # looping macro would otherwise start one worker per iteration against a
    # daemon that never answers.
    release = threading.Event()
    link = Link(block=release)
    daemon = channel(link)
    results, errors, on_result, on_error = collect()
    try:
        assert daemon.call("status", None, 1.0, on_result, on_error) is True
        link.entered.wait(2.0)
        started = 1
        for _ in range(200):
            daemon.cancel()
            if daemon.call("status", None, 1.0, on_result, on_error):
                started += 1
        assert started == 1, "200 cancel/start cycles started %d workers" % (started,)
        assert len([t for t in threading.enumerate() if t.name == "plr-test"]) == 1
    finally:
        release.set()
    fake_printer.reactor.pump_async(1, timeout=2.0)


def test_a_shutdown_does_not_close_the_channel(fake_printer, channel):
    # A SHUTDOWN is not teardown: klippy stays up until FIRMWARE_RESTART,
    # and that is exactly when an operator needs PLR_STATUS to still tell
    # them what plrd thinks it is doing.  Treating it as terminal made
    # PLR_STATUS / PLR_RECOVER / PLR_WIZARD_START permanently promise a
    # report that would never arrive — a regression from before this branch.
    daemon = channel(Link(response={"ok": True, "text": "still here", "data": {}}))
    fake_printer.invoke_shutdown("Manual stop (M112)")
    assert daemon.is_closed() is False
    assert daemon.refusal_text("PLR_STATUS") is None
    results, errors, on_result, on_error = collect()
    assert daemon.call("status", None, 1.0, on_result, on_error) is True
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert [r["text"] for r in results] == ["still here"]


def test_nothing_is_delivered_after_klippy_disconnects(fake_printer, channel):
    release = threading.Event()
    link = Link(block=release)
    daemon = channel(link)
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    link.entered.wait(2.0)
    fake_printer.send_event("klippy:disconnect")
    assert daemon.is_closed() is True
    release.set()
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    # A worker thread must never touch a printer that has gone away.
    assert results == [] and errors == []
    # ...and no new call can be started either.
    assert daemon.call("status", None, 1.0, on_result, on_error) is False


def test_a_closed_channel_says_so_instead_of_claiming_it_is_busy(fake_printer, channel):
    # BLOCKER: every call site reads `call() is False` and must be able to
    # tell "closed" from "busy", or it invents an in-flight query.
    daemon = channel(Link())
    fake_printer.send_event("klippy:disconnect")
    text = daemon.refusal_text("PLR_STATUS")
    assert text is not None
    assert "shutting down" in text
    assert "in flight" not in text


def test_a_busy_channel_says_busy(fake_printer, channel):
    release = threading.Event()
    daemon = channel(Link(block=release))
    results, errors, on_result, on_error = collect()
    try:
        daemon.call("status", None, 1.0, on_result, on_error)
        text = daemon.refusal_text("PLR_STATUS")
        assert text is not None and "has not answered yet" in text
    finally:
        release.set()
    fake_printer.reactor.pump_async(1, timeout=2.0)


def test_disconnect_is_idempotent(fake_printer, channel):
    daemon = channel(Link())
    fake_printer.send_event("klippy:disconnect")
    fake_printer.send_event("klippy:disconnect")
    assert daemon.is_closed() is True


def test_an_exception_in_the_callback_never_escapes_the_reactor(fake_printer, channel):
    # klippy/klippy.py:170-186: an exception escaping a reactor callback is
    # logged as "Unhandled exception during run" and turned into a printer
    # shutdown.  Nothing this plugin does may be able to cause that.
    daemon = channel(Link(response={"ok": True, "text": "", "data": {}}))

    def explode(_payload):
        raise RuntimeError("callback exploded")

    daemon.call("status", None, 1.0, explode, explode)
    # pump_async invokes the callback the way the reactor would; a leak
    # would surface here as a raised RuntimeError.
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    gcode = fake_printer.lookup_object("gcode")
    assert any(
        "internal error handling the plrd response" in r for r in gcode.responses
    )


def test_a_failure_to_even_report_the_failure_is_contained(fake_printer, channel):
    # The last line of defence: the callback raised AND the console is
    # unreachable.  Still no exception into klippy's loop.
    daemon = channel(Link())
    fake_printer.objects.pop("gcode")

    def explode(_payload):
        raise RuntimeError("callback exploded")

    daemon.call("status", None, 1.0, explode, explode)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1


def test_a_dead_reactor_cannot_break_a_worker(fake_printer, channel):
    # After klippy finalizes the reactor its wakeup pipe is closed
    # (klippy/reactor.py:337-347).  A worker that hands over a result then
    # must not raise on a thread nobody is watching.
    daemon = channel(Link())

    def broken(*args, **kwargs):
        raise OSError("reactor is gone")

    fake_printer.reactor.register_async_callback = broken
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    for _ in range(400):
        if daemon.is_busy() is False or not threading.enumerate():
            break
        time.sleep(0.005)
    assert results == [] and errors == []


def test_a_thread_that_cannot_start_frees_the_slot_and_refuses_cleanly(
    fake_printer, channel, monkeypatch
):
    # A memory-pressured Pi can fail thread creation.  Leaving `_busy` set
    # would refuse every later attempt for the rest of the session, and
    # letting the RuntimeError escape a g-code handler would make klippy
    # shut the printer down (klippy/gcode.py:231-235).
    daemon = channel(Link())

    def refuse(self):
        raise RuntimeError("can't start new thread")

    monkeypatch.setattr(threading.Thread, "start", refuse)
    results, errors, on_result, on_error = collect()
    assert daemon.call("status", None, 1.0, on_result, on_error) is False
    assert daemon.is_busy() is False
    monkeypatch.undo()
    # ...and the channel still works afterwards.
    assert daemon.call("status", None, 1.0, on_result, on_error) is True
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert len(results) == 1


def test_worker_threads_are_daemon_threads(fake_printer, channel):
    # klippy restarts by re-running in the same process
    # (klippy/klippy.py:186-198) and must be able to exit while a socket
    # read is still blocked in recv.
    release = threading.Event()
    link = Link(block=release)
    daemon = channel(link)
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    link.entered.wait(2.0)
    try:
        workers = [t for t in threading.enumerate() if t.name == "plr-test"]
        assert workers, [t.name for t in threading.enumerate()]
        assert all(t.daemon for t in workers)
    finally:
        release.set()
    fake_printer.reactor.pump_async(1, timeout=2.0)


def test_the_link_is_resolved_per_call_not_captured(fake_printer):
    # The plugin owns the one DaemonLink; binding it at construction time
    # would freeze whatever object existed then (and would quietly defeat
    # every test that swaps it).
    holder = {"link": Link(response={"ok": True, "text": "first", "data": {}})}
    daemon = daemon_worker.AsyncDaemon(fake_printer, lambda: holder["link"], "test")
    results, errors, on_result, on_error = collect()
    daemon.call("status", None, 1.0, on_result, on_error)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    holder["link"] = Link(response={"ok": True, "text": "second", "data": {}})
    daemon.call("status", None, 1.0, on_result, on_error)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1
    assert [r["text"] for r in results] == ["first", "second"]


def test_the_channel_registers_only_for_teardown(fake_printer, channel):
    channel(Link())
    assert fake_printer.event_handlers.get("klippy:disconnect")
    # Deliberately NOT klippy:shutdown — see
    # test_a_shutdown_does_not_close_the_channel.
    assert not fake_printer.event_handlers.get("klippy:shutdown")


def test_the_fake_reactor_pump_reports_what_it_ran(fake_printer):
    # Guard for the harness itself: pump(0) must not silently swallow a
    # delivered callback, or every "nothing was delivered" assertion in the
    # suite would be vacuous.
    reactor = fake_printer.reactor
    ran = []
    reactor.register_async_callback(lambda eventtime: ran.append(eventtime))
    assert reactor.pending_async() == 1
    assert reactor.pump_async(0, timeout=0.0) == 1
    assert len(ran) == 1
    assert reactor.pump_async(0, timeout=0.0) == 0


def test_a_command_error_from_a_callback_is_still_contained(fake_printer, channel):
    # The realistic version of the callback-raises case: plugin code that
    # calls gcmd.error-style raises from a context that has no gcmd.
    daemon = channel(Link())

    def explode(_payload):
        raise fake_klippy.FakeCommandError("refused")

    daemon.call("status", None, 1.0, explode, explode)
    assert fake_printer.reactor.pump_async(1, timeout=2.0) == 1


# --- the structural guard: nothing else may call the link -------------


def _link_call_sites():
    """Every ``<something>.call(...)`` on a daemon link, with its location.

    An AST walk over the whole package: the point is to catch a FUTURE
    change that reintroduces a blocking daemon call somewhere a g-code
    handler can reach, which is exactly how this defect shipped in the
    first place.

    Matched on the receiver's spelling, and only the BLOCKING client's
    spellings: ``plugin.daemon`` / ``self.daemon`` (the one
    :class:`~plr.daemon_link.DaemonLink`), a local named ``link``, and
    ``self.get_link()`` (how the worker resolves it).  The non-blocking
    channels are deliberately NOT matched — ``daemon_query``,
    ``daemon_wizard`` and ``_async`` are
    :class:`~plr.daemon_worker.AsyncDaemon` instances whose ``call``
    starts a worker and returns, which is the safe path this guard exists
    to funnel everything through.
    """
    import os

    from plr import plugin as plugin_module

    package_dir = os.path.dirname(os.path.abspath(plugin_module.__file__))
    sites = []
    for name in sorted(os.listdir(package_dir)):
        if not name.endswith(".py"):
            continue
        with open(os.path.join(package_dir, name), encoding="utf-8") as handle:
            sites.extend((name, fn) for fn in _link_calls_in(handle.read()))
    return sorted(set(sites))


def _blocking_methods():
    """Every blocking public method on the shipped DaemonLink.

    Derived from the class, not listed: ``ping`` is also blocking, the
    earlier guard missed it, and the next method somebody adds must not need
    this test updated in order to be covered.
    """
    return {
        name
        for name in vars(daemon_link.DaemonLink)
        if not name.startswith("__") and callable(getattr(daemon_link.DaemonLink, name))
    }


def _link_calls_in(source):
    """The enclosing function name of every blocking link call in ``source``."""
    import ast

    blocking = _blocking_methods()
    tree = ast.parse(source)
    # Each function's line span, so a call can be attributed to the
    # function that contains it (the innermost one wins).
    scopes = [
        (node.lineno, getattr(node, "end_lineno", node.lineno), node.name)
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    ]
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr not in blocking:
            continue
        receiver = func.value
        if isinstance(receiver, ast.Attribute):
            spelling = receiver.attr
        elif isinstance(receiver, ast.Name):
            spelling = receiver.id
        elif isinstance(receiver, ast.Call):
            inner = receiver.func
            spelling = (
                inner.attr
                if isinstance(inner, ast.Attribute)
                else getattr(inner, "id", "")
            )
        else:
            spelling = ""
        if spelling not in ("daemon", "link", "get_link"):
            continue
        enclosing = sorted(
            (start, fn) for start, end, fn in scopes if start <= node.lineno <= end
        )
        found.append(enclosing[-1][1] if enclosing else "<module>")
    return found


def test_only_the_worker_and_the_detached_abort_call_the_daemon_link():
    # THE GUARD.  A blocking daemon call is only ever safe off the reactor,
    # so exactly two places in the package may make one:
    #
    #   * AsyncDaemon._work — runs on a worker thread by construction;
    #   * RecoverySession._abort_detached (its nested `run`) — its own
    #     detached thread, used from the klippy:shutdown handler where the
    #     AsyncDaemon channel is already closed.
    #
    # Anything else is, or can become, a call inside a g-code handler.  If
    # this list grows, the new site has to prove it is off the reactor.
    assert _link_call_sites() == [
        ("daemon_worker.py", "_work"),
        ("recovery.py", "run"),
    ]


@pytest.mark.parametrize(
    "body",
    [
        pytest.param("    plugin.daemon.call('status', {}, timeout=5.0)", id="call"),
        pytest.param("    self.daemon.call('status')", id="self-call"),
        pytest.param("    link.call('status')", id="local-link"),
        pytest.param("    self.get_link().call('status')", id="get_link"),
        # THE SHAPES THE FIRST VERSION OF THIS GUARD MISSED: ping() blocks
        # exactly as call() does, because it IS a call() underneath.
        pytest.param("    alive = plugin.daemon.ping()", id="ping"),
        pytest.param("    if self.daemon.ping():\n        pass", id="self-ping"),
    ],
)
def test_the_call_site_scan_catches_every_blocking_shape(body):
    # Proof the guard is not vacuous: each shape below, inside a g-code
    # handler, is the defect this branch removed, and each must be found.
    source = "def cmd_PLR_RECOVER(plugin, gcmd):\n%s\n" % (body,)
    assert _link_calls_in(source) == ["cmd_PLR_RECOVER"], source


def test_the_scan_ignores_the_non_blocking_channel():
    # The AsyncDaemon channels also expose `call`, and theirs is the SAFE
    # path: it starts a worker and returns.
    fixed = (
        "def cmd_PLR_RECOVER(plugin, gcmd):\n"
        "    plugin.daemon_query.call('recover_dryrun', None, 1.0, ok, err)\n"
        "    plugin.daemon_wizard.call('status', None, 1.0, ok, err)\n"
        "    self._async.call('recover_confirm', {}, 1.0, ok, err)\n"
    )
    assert _link_calls_in(fixed) == []


def test_ping_is_covered_by_the_derived_method_set():
    # If DaemonLink's blocking surface stops being derivable, the guard
    # silently narrows to nothing.
    methods = _blocking_methods()
    assert {"call", "ping"} <= methods, methods
