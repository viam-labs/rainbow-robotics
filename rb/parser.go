package rb

import (
	"strconv"
	"strings"
)

// ACKMessage is the line the control box sends after executing a script line.
const ACKMessage = "The command was executed"

// NotAllowedMessage is the line the control box sends for an unrecognised first command.
const NotAllowedMessage = "The command is not allowed"

// LineType classifies a line received on the command port.
type LineType int

// Line types, mirroring rbpodo's Response::Type.
const (
	LineUnknown LineType = iota
	LineACK
	LineInfo
	LineWarn
	LineError
	LineNotAllowed
)

func (t LineType) String() string {
	switch t {
	case LineACK:
		return "ack"
	case LineInfo:
		return "info"
	case LineWarn:
		return "warn"
	case LineError:
		return "error"
	case LineNotAllowed:
		return "not-allowed"
	default:
		return "unknown"
	}
}

// Line is one parsed line from the command port, e.g. info[motion_changed][0].
type Line struct {
	Raw      string
	Type     LineType
	Category string
	Msg      string
}

// IsACK reports whether the line acknowledges a command. The "not allowed"
// reply counts: the box sends exactly one of the two per script line.
func (l Line) IsACK() bool { return l.Type == LineACK || l.Type == LineNotAllowed }

// ErrorCode returns the numeric code of an error[code][N] line.
func (l Line) ErrorCode() (int, bool) {
	if l.Type != LineError {
		return 0, false
	}
	n, err := strconv.Atoi(strings.TrimSpace(l.Msg))
	if err != nil {
		return 0, false
	}
	return n, true
}

// IsMotionChanged reports whether the line is a motion_changed event and, if so,
// whether it signals motion starting (true) or ending (false).
func (l Line) IsMotionChanged() (started, ok bool) {
	if l.Type != LineInfo || l.Category != "motion_changed" {
		return false, false
	}
	return strings.TrimSpace(l.Msg) != "0", true
}

// IsPrint reports whether the line is the output of a print() call.
func (l Line) IsPrint() bool {
	return l.Type == LineInfo && strings.HasPrefix(l.Category, "print")
}

// ParseLine classifies a raw line the same way rbpodo's Response::parse does:
// the ACK string is matched exactly; otherwise the text before the first '[' is
// the type, the first bracketed group (bracket-balanced) the category, and the
// second the message.
func ParseLine(raw string) Line {
	raw = strings.TrimRight(raw, "\r\n")
	raw = strings.TrimSpace(raw)
	l := Line{Raw: raw}
	switch raw {
	case ACKMessage:
		l.Type = LineACK
		return l
	case NotAllowedMessage:
		l.Type = LineNotAllowed
		return l
	}

	i := strings.IndexByte(raw, '[')
	if i < 0 {
		return l
	}
	switch raw[:i] {
	case "info":
		l.Type = LineInfo
	case "warn":
		l.Type = LineWarn
	case "error":
		l.Type = LineError
	default:
		return l
	}
	rest := raw[i:]
	var ok bool
	l.Category, rest, ok = extractBracket(rest)
	if !ok {
		return l
	}
	l.Msg, _, _ = extractBracket(rest)
	return l
}

// extractBracket returns the contents of the first bracket-balanced [...] group
// in s, the remainder after it, and whether the group was closed.
func extractBracket(s string) (content, rest string, ok bool) {
	start := strings.IndexByte(s, '[')
	if start < 0 {
		return "", s, false
	}
	depth := 0
	for i := start; i < len(s); i++ {
		switch s[i] {
		case '[':
			depth++
		case ']':
			depth--
			if depth == 0 {
				return s[start+1 : i], s[i+1:], true
			}
		}
	}
	return s[start+1:], "", false
}
