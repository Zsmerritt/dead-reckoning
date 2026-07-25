"""Klipper *action prompt* line builders and the one prompt record.

Extracted from :mod:`plr.wizard` so the recovery-confirmation renderer
(:mod:`plr.confirm_ui`) can build dialogs without importing the wizard —
the wizard imports the recovery session, which imports the renderer, so
the builders have to sit below all three.  :mod:`plr.wizard` re-exports
every name here, so ``wizard.action_prompt_begin`` and friends keep
working.

Wire format (Mainsail "Macro Prompts" spec, supported since Mainsail
2.9.0):

    // action:prompt_begin <headline>
    // action:prompt_text <text>
    // action:prompt_button <label>|<gcode?>|<color?>
    // action:prompt_footer_button <label>|<gcode?>|<color?>
    // action:prompt_show
    // action:prompt_end

Colors: primary | secondary | info | warning | error (else a default).
Each line goes out through ``gcmd.respond_info`` / ``gcode.respond_info``,
which klippy prefixes with ``// `` on every line
(klippy/gcode.py:247-252) — byte-identical to what
``RESPOND TYPE=command MSG="action:..."`` puts on the wire
(klippy/extras/respond.py maps TYPE=command to the ``//`` prefix), so
``[respond]`` is NOT required.

PORTABILITY RULE, enforced by review and by tests: every button fires a
plain ``PLR_*`` g-code command and nothing else, and every prompt is
paired with plain-text fallback lines naming those same commands.
Mainsail >= 2.9.0 and KlipperScreen render the full spec;
OctoPrint/OctoApp implement the older Action Command Prompt protocol with
no pipe fields, gcode or colors, so a pipe-delimited button renders there
as inert text and the console fallback is the working path.  No prompt in
this plugin promises an image: no client renders one.
"""

import collections

# The action protocol is LINE-oriented, and ``respond_info`` splits its
# argument on newlines and prefixes each line with ``// ``
# (klippy/gcode.py:250-254).  So a ``\n`` inside a prompt text or a button
# label does not "wrap": it emits a second ``// `` line that no client can
# parse as part of the action, shredding the dialog from that point on.
#
# Prompt content is not all ours — it carries plrd's diagnosis prose
# (``what`` / ``why`` / ``suggested_fix``), and plr/confirm_ui.py
# deliberately renders whatever arrives, including from a future daemon
# whose text is multi-line.  Every string that becomes part of an action
# line therefore passes through :func:`one_line` first.
_LINE_BREAKS = ("\r\n", "\r", "\n", " ", " ")


def one_line(text):
    """Collapse anything that would break an action line into one line."""
    if not isinstance(text, str):
        return text
    for token in _LINE_BREAKS:
        text = text.replace(token, " ")
    # Tabs are legal but make the console output ragged; collapse runs of
    # whitespace so a wrapped source string reads as one sentence.
    return " ".join(text.split())


# One prompt to render: a headline, descriptive text lines, primary
# buttons and footer buttons (each ``(label, gcode, color)``), and the
# plain-text fallback lines that name the advancing console command(s).
Prompt = collections.namedtuple(
    "Prompt", ["title", "texts", "buttons", "footers", "fallbacks"]
)


def button_spec(label, gcode, color):
    """``<label>|<gcode?>|<color?>`` with the pipes the Mainsail spec uses.

    A color forces the middle field (empty gcode defaults to the label
    on the client); gcode alone yields ``label|gcode``; label alone is
    bare.
    """
    label = one_line(label)
    gcode = one_line(gcode)
    if color is not None:
        return "|".join([label, gcode or "", color])
    if gcode is not None:
        return "|".join([label, gcode])
    return label


def action_prompt_begin(title):
    return "action:prompt_begin %s" % (one_line(title),)


def action_prompt_text(text):
    return "action:prompt_text %s" % (one_line(text),)


def action_prompt_button(label, gcode=None, color=None):
    return "action:prompt_button %s" % (button_spec(label, gcode, color),)


def action_prompt_footer_button(label, gcode=None, color=None):
    return "action:prompt_footer_button %s" % (button_spec(label, gcode, color),)


def action_prompt_show():
    return "action:prompt_show"


def action_prompt_end():
    return "action:prompt_end"


def emit_prompt(respond, prompt):
    """Emit one :data:`Prompt` as ordered action lines, then its fallback.

    ``respond`` is ``gcmd.respond_info`` or ``gcode.respond_info`` — a
    plain output call that takes no g-code mutex (klippy/gcode.py:247-252
    walks ``output_callbacks``), which is what makes this safe to call
    from a reactor callback while plrd holds the mutex to drive the
    machine.

    Order is fixed and asserted by tests: begin, text*, button*,
    footer_button*, show, then the plain-text fallback line(s) that name
    the advancing console command.
    """
    respond(action_prompt_begin(prompt.title))
    for text in prompt.texts:
        respond(action_prompt_text(text))
    for label, gcode, color in prompt.buttons:
        respond(action_prompt_button(label, gcode, color))
    for label, gcode, color in prompt.footers:
        respond(action_prompt_footer_button(label, gcode, color))
    respond(action_prompt_show())
    # Fallbacks are plain console text, NOT action lines: a newline in one
    # is harmless there (respond_info just emits two `// ` lines), so they
    # are passed through unmodified.
    for line in prompt.fallbacks:
        respond(line)
