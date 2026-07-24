//! Byte-exact g-code line parsing.
//!
//! The tokenization rules replicate Klipper's dispatcher
//! (`klippy/gcode.py`, `_process_commands`, lines 200-218 of the reference
//! checkout):
//!
//! * a line is stripped of surrounding whitespace, then cut at the first
//!   `;` (the remainder is the comment);
//! * the code part is uppercased and split on the regex
//!   `([A-Z_]+|[A-Z*])` — maximal runs of `A-Z`/`_` plus single `*`;
//! * a leading `N<number>` word (line number) is skipped when determining
//!   the command; the command is the first letter run joined with the text
//!   that follows it (e.g. `G1`, `SET_GCODE_OFFSET`);
//! * remaining letter/value pairs become the parameters; a duplicated key
//!   keeps its last occurrence, matching Python dict construction.
//!
//! Commands that are not "traditional" (letter + number, see
//! `GCodeDispatch.is_traditional_gcode`, gcode.py:125-130) carry their raw
//! parameter text and shell-style `KEY=VALUE` parameters, replicating
//! `GCodeCommand.get_raw_command_parameters` (gcode.py:40-53) and
//! `GCodeDispatch._get_extended_params` (gcode.py:266-281, `shlex` in
//! POSIX mode with `;`/`#` comments).
//!
//! Totality: parsing never fails and never panics, for any byte input.
//! Non-UTF-8 bytes are decoded lossily (replacement character), which
//! diverges from Klipper (`CPython` would raise a decode error) but
//! guarantees the recovery pipeline can always walk a damaged file.
//! Divergences from `CPython` on garbage input (Python `float` accepting
//! `_` digit separators, full Unicode case mapping subtleties) are
//! intentionally not replicated; they cannot occur in slicer output.
//!
//! Every [`Line`] carries the [`ByteSpan`] it occupied in the source
//! stream, including its line terminator, so `span.end` of a line is
//! always the byte offset of the next line — the boundary needed for
//! `M26 S<byte>` resume offsets.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Half-open byte range `[start, end)` of a line in the source stream,
/// including the line terminator (`\n` or `\r\n`) when present.
///
/// `end` of one line equals `start` of the next, so both are valid
/// `M26 S<byte>` resume boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ByteSpan {
    /// Offset of the first byte of the line.
    pub start: u64,
    /// Offset one past the last byte of the line (terminator included).
    pub end: u64,
}

impl ByteSpan {
    /// Number of bytes covered by the span.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.end.saturating_sub(self.start)
    }

    /// True when the span covers no bytes.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// A single classified g-code line together with its source span.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Line {
    /// Location of this line in the source stream.
    pub span: ByteSpan,
    /// Parsed content of the line.
    pub body: LineBody,
}

impl Line {
    /// The command carried by this line, if any.
    #[must_use]
    pub fn command(&self) -> Option<&Command> {
        match &self.body {
            LineBody::Command { command, .. } => Some(command),
            LineBody::Blank | LineBody::Comment(_) => None,
        }
    }

    /// The comment attached to this line, if any.
    #[must_use]
    pub fn comment(&self) -> Option<&Comment> {
        match &self.body {
            LineBody::Comment(c) => Some(c),
            LineBody::Command { comment, .. } => comment.as_ref(),
            LineBody::Blank => None,
        }
    }
}

impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.body {
            LineBody::Blank => Ok(()),
            LineBody::Comment(c) => f.write_str(&c.text),
            LineBody::Command { command, comment } => {
                write!(f, "{command}")?;
                if let Some(c) = comment {
                    write!(f, " {}", c.text)?;
                }
                Ok(())
            }
        }
    }
}

/// The content of a parsed line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum LineBody {
    /// Nothing but whitespace.
    Blank,
    /// Only a comment (`;...`), possibly a slicer annotation.
    Comment(Comment),
    /// A command, optionally followed by a comment.
    ///
    /// For extended (non-traditional) commands the comment is folded into
    /// the raw parameter text instead (Klipper's shlex pass consumes it),
    /// so `comment` is always `None` for those.
    Command {
        /// The command and its parameters.
        command: Command,
        /// Trailing comment, traditional commands only.
        comment: Option<Comment>,
    },
}

/// A comment (starting at `;`, text kept verbatim).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Comment {
    /// Verbatim comment text, including the leading `;`.
    pub text: String,
}

impl Comment {
    /// Classify well-known slicer annotations.
    ///
    /// Recognized: `;TYPE:<name>` (Prusa/Orca feature type blocks),
    /// `;LAYER_CHANGE` (Prusa/Orca), `;LAYER:<n>` (Cura style) and
    /// `;Z:<height>` (Prusa). Returns `None` for anything else.
    #[must_use]
    pub fn annotation(&self) -> Option<Annotation> {
        let body = self.text.strip_prefix(';').unwrap_or(&self.text).trim();
        if let Some(t) = body.strip_prefix("TYPE:") {
            return Some(Annotation::FeatureType(t.trim().to_string()));
        }
        if body == "LAYER_CHANGE" {
            return Some(Annotation::LayerChange);
        }
        if let Some(l) = body.strip_prefix("LAYER:") {
            return Some(Annotation::Layer(l.trim().to_string()));
        }
        if let Some(z) = body.strip_prefix("Z:") {
            return z.trim().parse::<f64>().ok().map(Annotation::Z);
        }
        None
    }
}

/// A recognized slicer annotation carried by a comment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Annotation {
    /// `;TYPE:<name>` — extrusion feature type (e.g. `External perimeter`,
    /// `Internal infill`, `Outer wall`).
    FeatureType(String),
    /// `;LAYER_CHANGE` marker.
    LayerChange,
    /// `;LAYER:<n>` marker (value kept verbatim).
    Layer(String),
    /// `;Z:<height>` marker.
    Z(f64),
}

/// A parsed command word plus parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Command {
    /// Uppercased dispatch name exactly as Klipper would look it up
    /// (`G1`, `M220`, `SET_GCODE_OFFSET`, ...). May be empty for
    /// degenerate lines such as a bare line number (Klipper treats those
    /// as silent no-ops).
    pub name: String,
    /// The command's parameters.
    pub params: CommandParams,
}

/// Parameter storage; the layout differs between traditional
/// (letter+number) commands and extended commands.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CommandParams {
    /// Traditional command: uppercase letter-run keys with whitespace
    /// trimmed values, in source order. The pair that forms the command
    /// itself, and a leading `N<line number>` pair, are not stored (no
    /// Klipper handler reads them).
    Traditional {
        /// `(key, value)` pairs; duplicate keys resolve last-wins.
        pairs: Vec<(String, String)>,
    },
    /// Extended command: raw parameter text (verbatim, original case)
    /// plus shell-style parsed `KEY=VALUE` pairs.
    Extended {
        /// Raw parameter region, exactly as Klipper's
        /// `get_raw_command_parameters` would return it.
        raw: String,
        /// Parsed pairs; `None` when the raw text is malformed under
        /// shlex rules (unbalanced quote, missing `=`), in which case
        /// Klipper raises "Malformed command" for registered handlers.
        pairs: Option<Vec<(String, String)>>,
    },
}

impl Command {
    /// All parameter pairs, or `None` for a malformed extended command.
    #[must_use]
    pub fn pairs(&self) -> Option<&[(String, String)]> {
        match &self.params {
            CommandParams::Traditional { pairs } => Some(pairs),
            CommandParams::Extended { pairs, .. } => pairs.as_deref(),
        }
    }

    /// Look up a parameter by (uppercase) key, last occurrence winning —
    /// the same resolution as Python dict construction in Klipper.
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&str> {
        self.pairs()?
            .iter()
            .rev()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    /// True when this is an extended command whose parameter text failed
    /// shlex parsing.
    #[must_use]
    pub fn is_malformed_extended(&self) -> bool {
        matches!(&self.params, CommandParams::Extended { pairs: None, .. })
    }
}

impl fmt::Display for Command {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.params {
            CommandParams::Traditional { pairs } => {
                f.write_str(&self.name)?;
                for (k, v) in pairs {
                    write!(f, " {k}{v}")?;
                }
                Ok(())
            }
            CommandParams::Extended { raw, .. } => {
                if self.name.is_empty() {
                    f.write_str(raw)
                } else {
                    f.write_str(&self.name)?;
                    if !raw.is_empty() {
                        write!(f, " {raw}")?;
                    }
                    Ok(())
                }
            }
        }
    }
}

/// Iterator over the lines of a byte buffer, yielding parsed [`Line`]s
/// with stream-absolute byte spans.
///
/// Handles both LF and CRLF terminators; a final line without terminator
/// is still yielded. The concatenation of all yielded spans exactly tiles
/// `base_offset..base_offset + data.len()`.
#[derive(Debug, Clone)]
pub struct LineIter<'a> {
    data: &'a [u8],
    pos: usize,
    base: u64,
}

impl<'a> LineIter<'a> {
    /// Iterate over `data`, reporting spans relative to `base_offset`
    /// (the stream offset at which `data` begins).
    #[must_use]
    pub fn new(data: &'a [u8], base_offset: u64) -> Self {
        Self {
            data,
            pos: 0,
            base: base_offset,
        }
    }
}

/// Lossless-on-64-bit conversion from in-memory index to stream offset.
fn offset_u64(v: usize) -> u64 {
    u64::try_from(v).unwrap_or(u64::MAX)
}

impl Iterator for LineIter<'_> {
    type Item = Line;

    fn next(&mut self) -> Option<Line> {
        let rest = self.data.get(self.pos..)?;
        if rest.is_empty() {
            return None;
        }
        let (content_len, term_len) = match rest.iter().position(|&b| b == b'\n') {
            Some(nl) => (nl, 1),
            None => (rest.len(), 0),
        };
        let raw = rest.get(..content_len).unwrap_or(&[]);
        let start = self.base.saturating_add(offset_u64(self.pos));
        let end = start.saturating_add(offset_u64(content_len + term_len));
        self.pos += content_len + term_len;
        Some(parse_line(raw, ByteSpan { start, end }))
    }
}

/// Parse a single line (without its terminator) into a [`Line`].
///
/// Total: never fails, never panics, for any byte content.
#[must_use]
pub fn parse_line(bytes: &[u8], span: ByteSpan) -> Line {
    let text = String::from_utf8_lossy(bytes);
    let stripped = text.trim();
    if stripped.is_empty() {
        return Line {
            span,
            body: LineBody::Blank,
        };
    }
    // Comment split (gcode.py:206-208). `;` is ASCII so the index is a
    // char boundary.
    let (code, comment) = match stripped.find(';') {
        Some(cpos) => (
            stripped.get(..cpos).unwrap_or(""),
            Some(Comment {
                text: stripped.get(cpos..).unwrap_or("").to_string(),
            }),
        ),
        None => (stripped, None),
    };
    let upper = code.to_uppercase();
    let parts = split_args(&upper);
    // Command word extraction (gcode.py:210-215).
    let n_prefix = parts.len() >= 2
        && parts.first().is_some_and(String::is_empty)
        && parts.get(1).is_some_and(|p| p == "N");
    let joined = if n_prefix {
        format!(
            "{}{}",
            parts.get(3).map_or("", String::as_str),
            parts.get(4).map_or("", String::as_str)
        )
    } else {
        let mut s = String::new();
        for p in parts.iter().take(3) {
            s.push_str(p);
        }
        s
    };
    let name = joined.trim().to_string();
    if name.is_empty() && code.trim().is_empty() {
        // Comment-only line.
        return Line {
            span,
            body: match comment {
                Some(c) => LineBody::Comment(c),
                None => LineBody::Blank,
            },
        };
    }
    if is_traditional_command(&name) {
        let mut pairs = pairs_from_parts(&parts);
        // Drop the line-number pair and the pair that formed the command
        // word; no Klipper handler ever reads them.
        let drop_n = if n_prefix { 2 } else { 1 };
        pairs.drain(..drop_n.min(pairs.len()));
        Line {
            span,
            body: LineBody::Command {
                command: Command {
                    name,
                    params: CommandParams::Traditional { pairs },
                },
                comment,
            },
        }
    } else {
        // Extended command: parameters come from the raw (original-case)
        // line; the shlex pass owns comment handling (gcode.py:266-281),
        // so no separate comment is extracted.
        let raw = raw_command_parameters(stripped, &name);
        let pairs = shlex_kv(&raw);
        Line {
            span,
            body: LineBody::Command {
                command: Command {
                    name,
                    params: CommandParams::Extended { raw, pairs },
                },
                comment: None,
            },
        }
    }
}

/// Split per Klipper's `args_r = re.compile('([A-Z_]+|[A-Z*])')`
/// (gcode.py:201): returns the `re.split` result — alternating non-match
/// text and match tokens, always odd in length, starting and ending with
/// (possibly empty) text.
fn split_args(s: &str) -> Vec<String> {
    let mut parts = Vec::new();
    let mut text = String::new();
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c.is_ascii_uppercase() || c == '_' {
            let mut tok = String::new();
            tok.push(c);
            while let Some(&n) = chars.peek() {
                if n.is_ascii_uppercase() || n == '_' {
                    tok.push(n);
                    chars.next();
                } else {
                    break;
                }
            }
            parts.push(std::mem::take(&mut text));
            parts.push(tok);
        } else if c == '*' {
            parts.push(std::mem::take(&mut text));
            parts.push("*".to_string());
        } else {
            text.push(c);
        }
    }
    parts.push(text);
    parts
}

/// Klipper's parameter-dict construction (gcode.py:217-218): pairs of
/// (match token, following text stripped).
fn pairs_from_parts(parts: &[String]) -> Vec<(String, String)> {
    let mut out = Vec::new();
    let mut it = parts.iter().skip(1);
    while let Some(k) = it.next() {
        let v = it.next().map_or_else(String::new, |s| s.trim().to_string());
        out.push((k.clone(), v));
    }
    out
}

/// `GCodeDispatch.is_traditional_gcode` (gcode.py:125-133): first
/// whitespace token; uppercase letter followed by a digit, with the tail
/// parseable as a float.
fn is_traditional_command(cmd: &str) -> bool {
    let Some(token) = cmd.split_whitespace().next() else {
        return false;
    };
    let mut chars = token.chars();
    let (Some(first), Some(second)) = (chars.next(), chars.next()) else {
        return false;
    };
    let tail_ok = token.get(1..).is_some_and(|t| t.parse::<f64>().is_ok());
    tail_ok && first.is_ascii_uppercase() && second.is_ascii_digit()
}

/// `GCodeCommand.get_raw_command_parameters` (gcode.py:40-53).
///
/// `origline` is the stripped original-case line (comment included);
/// `command` is the uppercase dispatch name. On pathological inputs where
/// byte arithmetic falls off a char boundary, this degrades to an empty
/// string instead of panicking (unreachable for slicer output).
fn raw_command_parameters(origline: &str, command: &str) -> String {
    let mut param_start = command.len();
    let mut param_end = origline.len();
    let head_matches = origline
        .get(..param_start)
        .is_some_and(|h| h.to_uppercase() == command);
    if !head_matches {
        // Skip any gcode line-number and ignore any trailing checksum.
        let up = origline.to_uppercase();
        match up.find(command) {
            Some(idx) => param_start += idx,
            // Python `str.find` returns -1 here; replicate the resulting
            // offset without underflow.
            None => param_start = param_start.saturating_sub(1),
        }
        if let Some(star) = origline.rfind('*') {
            let digits = origline
                .get(star + 1..)
                .is_some_and(|s| !s.is_empty() && s.chars().all(|c| c.is_ascii_digit()));
            if digits {
                param_end = star;
            }
        }
    }
    if let Some(rest) = origline.get(param_start..) {
        if let Some(first) = rest.chars().next() {
            if first.is_whitespace() {
                param_start += first.len_utf8();
            }
        }
    }
    origline
        .get(param_start..param_end)
        .unwrap_or_default()
        .to_string()
}

/// Emulation of `shlex.shlex(raw, posix=True)` with
/// `whitespace_split = True` and `commenters = '#;'`, followed by the
/// `KEY=VALUE` split of `_get_extended_params` (gcode.py:266-281).
///
/// Returns `None` where `CPython` would raise (unbalanced quote, trailing
/// escape, token without `=`).
fn shlex_kv(raw: &str) -> Option<Vec<(String, String)>> {
    let mut tokens: Vec<String> = Vec::new();
    let mut cur = String::new();
    let mut has_token = false;
    let mut chars = raw.chars();
    'outer: while let Some(c) = chars.next() {
        if c.is_whitespace() {
            if has_token {
                tokens.push(std::mem::take(&mut cur));
                has_token = false;
            }
        } else if c == ';' || c == '#' {
            // Comment: terminates the current token and the line.
            if has_token {
                tokens.push(std::mem::take(&mut cur));
            }
            break 'outer;
        } else if c == '\'' {
            has_token = true;
            loop {
                match chars.next() {
                    None => return None, // No closing quotation.
                    Some('\'') => break,
                    Some(ch) => cur.push(ch),
                }
            }
        } else if c == '"' {
            has_token = true;
            loop {
                match chars.next() {
                    None => return None, // No closing quotation.
                    Some('"') => break,
                    Some('\\') => match chars.next() {
                        None => return None, // No escaped character.
                        // Inside double quotes POSIX shlex only unescapes
                        // the quote char and the escape char itself.
                        Some(e) if e == '"' || e == '\\' => cur.push(e),
                        Some(e) => {
                            cur.push('\\');
                            cur.push(e);
                        }
                    },
                    Some(ch) => cur.push(ch),
                }
            }
        } else if c == '\\' {
            has_token = true;
            // A trailing escape is "No escaped character" in `CPython`.
            cur.push(chars.next()?);
        } else {
            has_token = true;
            cur.push(c);
        }
    }
    if has_token {
        tokens.push(cur);
    }
    let mut pairs = Vec::with_capacity(tokens.len());
    for tok in tokens {
        let (k, v) = tok.split_once('=')?;
        pairs.push((k.to_uppercase(), v.to_string()));
    }
    Some(pairs)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(s: &str) -> Line {
        parse_line(
            s.as_bytes(),
            ByteSpan {
                start: 0,
                end: offset_u64(s.len()),
            },
        )
    }

    fn cmd(line: &Line) -> &Command {
        line.command().expect("expected a command")
    }

    #[test]
    fn basic_g1() {
        let l = parse("G1 X10.5 Y-3 E.42 F9000");
        let c = cmd(&l);
        assert_eq!(c.name, "G1");
        assert_eq!(c.get("X"), Some("10.5"));
        assert_eq!(c.get("Y"), Some("-3"));
        assert_eq!(c.get("E"), Some(".42"));
        assert_eq!(c.get("F"), Some("9000"));
        assert_eq!(c.get("Z"), None);
    }

    #[test]
    fn no_spaces_and_lowercase() {
        let l = parse("g1x5y3e0.1");
        let c = cmd(&l);
        assert_eq!(c.name, "G1");
        assert_eq!(c.get("X"), Some("5"));
        assert_eq!(c.get("Y"), Some("3"));
        assert_eq!(c.get("E"), Some("0.1"));
    }

    #[test]
    fn duplicate_key_last_wins() {
        let l = parse("G1 X1 X2");
        assert_eq!(cmd(&l).get("X"), Some("2"));
    }

    #[test]
    fn line_number_and_checksum() {
        let l = parse("N123 G1 X0*57");
        let c = cmd(&l);
        assert_eq!(c.name, "G1");
        assert_eq!(c.get("X"), Some("0"));
        // The checksum survives as a `*` parameter, as in Klipper.
        assert_eq!(c.get("*"), Some("57"));
        assert_eq!(c.get("N"), None, "line-number pair is dropped");
    }

    #[test]
    fn bare_line_number_is_empty_command() {
        let l = parse("N123");
        let c = cmd(&l);
        assert_eq!(c.name, "");
    }

    #[test]
    fn comment_only_and_annotations() {
        let l = parse("  ;TYPE:External perimeter");
        assert!(matches!(l.body, LineBody::Comment(_)));
        assert_eq!(
            l.comment().and_then(Comment::annotation),
            Some(Annotation::FeatureType("External perimeter".to_string()))
        );
        assert_eq!(
            parse(";LAYER_CHANGE")
                .comment()
                .and_then(Comment::annotation),
            Some(Annotation::LayerChange)
        );
        assert_eq!(
            parse(";LAYER:7").comment().and_then(Comment::annotation),
            Some(Annotation::Layer("7".to_string()))
        );
        assert_eq!(
            parse(";Z:0.6").comment().and_then(Comment::annotation),
            Some(Annotation::Z(0.6))
        );
        assert_eq!(
            parse("; just a note")
                .comment()
                .and_then(Comment::annotation),
            None
        );
        assert_eq!(
            parse(";Z:oops").comment().and_then(Comment::annotation),
            None
        );
    }

    #[test]
    fn trailing_comment_preserved() {
        let l = parse("G1 X1 ; wipe");
        assert_eq!(cmd(&l).get("X"), Some("1"));
        assert_eq!(l.comment().map(|c| c.text.as_str()), Some("; wipe"));
    }

    #[test]
    fn blank_lines() {
        assert!(matches!(parse("").body, LineBody::Blank));
        assert!(matches!(parse("   \t ").body, LineBody::Blank));
    }

    #[test]
    fn extended_command_basic() {
        let l = parse("SET_GCODE_OFFSET Z=0.2 MOVE=1");
        let c = cmd(&l);
        assert_eq!(c.name, "SET_GCODE_OFFSET");
        assert_eq!(c.get("Z"), Some("0.2"));
        assert_eq!(c.get("MOVE"), Some("1"));
    }

    #[test]
    fn extended_command_case_and_comment() {
        // Keys are uppercased, values keep case; ;-comment is consumed.
        let l = parse("save_gcode_state name=MyState ; midprint");
        let c = cmd(&l);
        assert_eq!(c.name, "SAVE_GCODE_STATE");
        assert_eq!(c.get("NAME"), Some("MyState"));
        assert_eq!(l.comment(), None, "comment folded into raw params");
    }

    #[test]
    fn extended_command_quoting() {
        let l = parse(r#"SET_THING VALUE="a b;c" OTHER='x y'"#);
        let c = cmd(&l);
        assert_eq!(c.get("VALUE"), Some("a b;c"));
        assert_eq!(c.get("OTHER"), Some("x y"));
    }

    #[test]
    fn extended_command_escapes() {
        let l = parse(r"SET_THING VALUE=a\ b");
        assert_eq!(cmd(&l).get("VALUE"), Some("a b"));
        // Inside double quotes only \" and \\ are unescaped.
        let l = parse(r#"SET_THING VALUE="a\"b\\c\d""#);
        assert_eq!(cmd(&l).get("VALUE"), Some(r#"a"b\c\d"#));
    }

    #[test]
    fn extended_malformed() {
        for s in [
            "SET_THING VALUE=\"unterminated",
            "SET_THING NOEQUALS",
            r"SET_THING TRAILING=\",
        ] {
            let l = parse(s);
            assert!(cmd(&l).is_malformed_extended(), "should be malformed: {s}");
            assert_eq!(cmd(&l).get("VALUE"), None);
        }
    }

    #[test]
    fn extended_after_line_number() {
        let l = parse("N5 SET_GCODE_OFFSET Z=1");
        let c = cmd(&l);
        assert_eq!(c.name, "SET_GCODE_OFFSET");
        assert_eq!(c.get("Z"), Some("1"));
    }

    #[test]
    fn unknown_word_is_extended() {
        // Klipper's command word is `''.join(parts[:3])` — for
        // "FOO-BAR" that is "FOO-" (the dispatch dict then misses).
        let l = parse("FOO-BAR");
        let c = cmd(&l);
        assert_eq!(c.name, "FOO-");
        // A plain extended name keeps itself intact.
        let l = parse("MY_MACRO A=1");
        assert_eq!(cmd(&l).name, "MY_MACRO");
        assert!(!cmd(&l).is_malformed_extended());
    }

    #[test]
    fn m117_multiword() {
        let l = parse("M117 HELLO WORLD");
        let c = cmd(&l);
        assert_eq!(c.name, "M117");
        assert_eq!(c.get("HELLO"), Some(""));
        assert_eq!(c.get("WORLD"), Some(""));
    }

    #[test]
    fn spans_tile_the_buffer() {
        let data = b"G1 X0\nG1 Y1\r\n\n; end";
        let lines: Vec<Line> = LineIter::new(data, 100).collect();
        assert_eq!(lines.len(), 4);
        assert_eq!(
            lines[0].span,
            ByteSpan {
                start: 100,
                end: 106
            }
        );
        assert_eq!(
            lines[1].span,
            ByteSpan {
                start: 106,
                end: 113
            }
        );
        assert_eq!(
            lines[2].span,
            ByteSpan {
                start: 113,
                end: 114
            }
        );
        assert_eq!(
            lines[3].span,
            ByteSpan {
                start: 114,
                end: 119
            }
        );
        assert!(matches!(lines[2].body, LineBody::Blank));
        // CRLF content parses identically to LF content.
        assert_eq!(lines[1].command().map(|c| c.name.as_str()), Some("G1"));
        assert_eq!(u64::from(u32::from(lines[3].span.is_empty())), 0);
    }

    #[test]
    fn final_line_without_terminator() {
        let lines: Vec<Line> = LineIter::new(b"G28", 0).collect();
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].span, ByteSpan { start: 0, end: 3 });
        assert_eq!(lines[0].command().map(|c| c.name.as_str()), Some("G28"));
    }

    #[test]
    fn empty_input_yields_nothing() {
        assert_eq!(LineIter::new(b"", 0).count(), 0);
    }

    #[test]
    fn non_utf8_is_lossy_not_fatal() {
        let mut data = b"G1 X".to_vec();
        data.extend_from_slice(&[0xff, 0xfe]);
        data.extend_from_slice(b"\nG1 Y2\n");
        let lines: Vec<Line> = LineIter::new(&data, 0).collect();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[1].command().and_then(|c| c.get("Y")), Some("2"));
    }

    #[test]
    fn display_round_trips_content() {
        for s in [
            "G1 X10.5 Y-3 E.42 F9000",
            "g1x5y3",
            "N123 G1 X0*57",
            "SET_GCODE_OFFSET Z=0.2 MOVE=1",
            "M117 HELLO WORLD",
            "; a comment",
            "",
            "G1 X1 ; wipe",
            "M204 S3000",
        ] {
            let l1 = parse(s);
            let out = l1.to_string();
            let l2 = parse(&out);
            assert_eq!(l1.body, l2.body, "unstable round trip for {s:?}");
        }
    }

    #[test]
    fn byte_span_len() {
        let s = ByteSpan { start: 5, end: 9 };
        assert_eq!(s.len(), 4);
        assert!(!s.is_empty());
        // Degenerate spans saturate instead of underflowing.
        let bad = ByteSpan { start: 9, end: 5 };
        assert_eq!(bad.len(), 0);
    }
}
