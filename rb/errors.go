package rb

import (
	"bufio"
	"bytes"
	_ "embed"
	"errors"
	"fmt"
	"strconv"
	"strings"
	"sync"
	"time"
)

// errorCodeCSV is rbpodo's res/error_code.csv (Apache-2.0): tab-separated
// code, Korean message, English message.
//
//go:embed error_code.csv
var errorCodeCSV []byte

// ErrorMessage is the controller's description of an error code.
type ErrorMessage struct {
	Ko string
	En string
}

var (
	errorTableOnce sync.Once
	errorTable     map[int]ErrorMessage
)

func loadErrorTable() {
	errorTable = make(map[int]ErrorMessage, 640)
	sc := bufio.NewScanner(bytes.NewReader(errorCodeCSV))
	sc.Buffer(make([]byte, 0, 64*1024), 1024*1024)
	for sc.Scan() {
		parts := strings.SplitN(sc.Text(), "\t", 3)
		if len(parts) != 3 {
			continue
		}
		code, err := strconv.Atoi(strings.TrimSpace(parts[0]))
		if err != nil {
			continue
		}
		errorTable[code] = ErrorMessage{Ko: parts[1], En: parts[2]}
	}
}

// LookupError returns the message for a controller error code.
func LookupError(code int) (ErrorMessage, bool) {
	errorTableOnce.Do(loadErrorTable)
	m, ok := errorTable[code]
	return m, ok
}

// ErrorText returns "code: english message" or "code: unknown error".
func ErrorText(code int) string {
	if m, ok := LookupError(code); ok {
		return fmt.Sprintf("%d: %s", code, strings.ReplaceAll(m.En, `\n`, " "))
	}
	return fmt.Sprintf("%d: unknown controller error", code)
}

// ControllerError is an error[code][N] line received from the control box.
type ControllerError struct {
	Code int
	At   time.Time
	Raw  string
}

func (e *ControllerError) Error() string {
	return "controller error " + ErrorText(e.Code)
}

// Sentinel errors returned by the client.
var (
	ErrNotConnected = errors.New("rb: not connected to control box")
	ErrTimeout      = errors.New("rb: timed out waiting for control box")
	ErrClosed       = errors.New("rb: client closed")
	ErrNoState      = errors.New("rb: no state received from control box yet")
)
