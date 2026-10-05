package rb

import (
	"math"
	"testing"
)

func mathFloat32bits(v float32) uint32 { return math.Float32bits(v) }

func TestParseLine(t *testing.T) {
	cases := []struct {
		raw      string
		typ      LineType
		category string
		msg      string
	}{
		{"The command was executed", LineACK, "", ""},
		{"The command was executed\r\n", LineACK, "", ""},
		{"The command is not allowed", LineNotAllowed, "", ""},
		{"info[motion_changed][0]", LineInfo, "motion_changed", "0"},
		{"info[motion_changed][1]", LineInfo, "motion_changed", "1"},
		{"error[code][186]", LineError, "code", "186"},
		{"warn[something][x]", LineWarn, "something", "x"},
		{"info[print_x][#ab12c_[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]", LineInfo, "print_x", "#ab12c_[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]"},
		{"info[activation][done]", LineInfo, "activation", "done"},
		{"info[controlbox][8.3.1,10]", LineInfo, "controlbox", "8.3.1,10"},
		{"info[unterminated", LineInfo, "unterminated", ""},
		{"something else entirely", LineUnknown, "", ""},
		{"", LineUnknown, "", ""},
	}
	for _, c := range cases {
		l := ParseLine(c.raw)
		if l.Type != c.typ || l.Category != c.category || l.Msg != c.msg {
			t.Errorf("ParseLine(%q) = {%v %q %q}, want {%v %q %q}", c.raw, l.Type, l.Category, l.Msg, c.typ, c.category, c.msg)
		}
	}
}

func TestLineHelpers(t *testing.T) {
	l := ParseLine("error[code][186]")
	if code, ok := l.ErrorCode(); !ok || code != 186 {
		t.Errorf("ErrorCode = %d %v", code, ok)
	}
	if started, ok := ParseLine("info[motion_changed][2]").IsMotionChanged(); !ok || !started {
		t.Error("expected motion started")
	}
	if started, ok := ParseLine("info[motion_changed][0]").IsMotionChanged(); !ok || started {
		t.Error("expected motion ended")
	}
	if _, ok := ParseLine("info[other][0]").IsMotionChanged(); ok {
		t.Error("unexpected motion_changed")
	}
	if !ParseLine("info[print_1][x]").IsPrint() || ParseLine("info[load][done]").IsPrint() {
		t.Error("IsPrint wrong")
	}
	if !ParseLine("The command is not allowed").IsACK() {
		t.Error("not-allowed should count as an ack")
	}
}

func TestErrorTable(t *testing.T) {
	m, ok := LookupError(186)
	if !ok {
		t.Fatal("code 186 missing from table")
	}
	if m.En == "" || m.Ko == "" {
		t.Errorf("empty messages: %+v", m)
	}
	if got := ErrorText(204); got != "204: Direct-Teaching Mode: This function is only available at Real-Robot mode" {
		t.Errorf("ErrorText(204) = %q", got)
	}
	if got := ErrorText(99999); got != "99999: unknown controller error" {
		t.Errorf("ErrorText(unknown) = %q", got)
	}
	if got := ErrorText(3); got != "3: Program is Stopped because of Collision-Detection" {
		t.Errorf("ErrorText(3) = %q", got)
	}
}
