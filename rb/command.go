package rb

import (
	"crypto/rand"
	"fmt"
	"strconv"
	"strings"
)

// Script formatters. Each returns exactly the byte sequence rbpodo sends (without
// the trailing newline the connection appends). Numbers are formatted with
// strconv 'f' and shortest precision so a value is never written in exponent
// notation, which the controller's script parser does not accept.

// Num formats a float for the wire.
func Num(v float64) string {
	return strconv.FormatFloat(v, 'f', -1, 64)
}

// Jnt formats a joint vector as jnt[a, b, c, d, e, f].
func Jnt(j [6]float64) string {
	parts := make([]string, 6)
	for i, v := range j {
		parts[i] = Num(v)
	}
	return "jnt[" + strings.Join(parts, ", ") + "]"
}

// Pnt formats a Cartesian point as pnt[x, y, z, rx, ry, rz].
func Pnt(p [6]float64) string {
	parts := make([]string, 6)
	for i, v := range p {
		parts[i] = Num(v)
	}
	return "pnt[" + strings.Join(parts, ", ") + "]"
}

// Activate powers and initialises all joints; wait for info[*][done].
func Activate() string { return "mc jall init" }

// Shutdown powers the arm down (rbpodo's shutdown()). Verify on hardware.
func Shutdown() string { return "AvadaKedavra()" }

// ControlBoxInfo requests info[controlbox][<version>,<type>].
func ControlBoxInfo() string { return "get_control_box_info()" }

// PGMode selects real or simulation operation.
func PGMode(real bool) string {
	if real {
		return "pgmode real"
	}
	return "pgmode simulation"
}

// SetSpeedBar sets the global speed multiplier 0-1.
func SetSpeedBar(v float64) string { return "set_speed_bar(" + Num(v) + ")" }

// MoveJ is a non-blocking joint move (deg, deg/s, deg/s^2).
func MoveJ(j [6]float64, speed, accel float64) string {
	return "move_j(" + Jnt(j) + "," + Num(speed) + "," + Num(accel) + ")"
}

// MoveL is a non-blocking straight-line TCP move to p (x,y,z mm; rx,ry,rz deg)
// at speed mm/s and accel mm/s^2, with the controller doing the IK.
func MoveL(p [6]float64, speed, accel float64) string {
	return "move_l(" + Pnt(p) + "," + Num(speed) + "," + Num(accel) + ")"
}

// MoveJL is a non-blocking joint-interpolated move to the TCP pose p (same
// units as MoveL) at joint speed deg/s and accel deg/s^2.
func MoveJL(p [6]float64, speed, accel float64) string {
	return "move_jl(" + Pnt(p) + "," + Num(speed) + "," + Num(accel) + ")"
}

// MoveJB2Clear clears the blended joint batch.
func MoveJB2Clear() string { return "move_jb2_clear()" }

// MoveJB2Add appends a waypoint to the blended joint batch.
func MoveJB2Add(j [6]float64, speed, accel, blend float64) string {
	return "move_jb2_add(" + Jnt(j) + "," + Num(speed) + "," + Num(accel) + ",0," + Num(blend) + ")"
}

// MoveJB2Run executes the blended joint batch.
func MoveJB2Run() string { return "move_jb2_run()" }

// MoveServoJ streams a joint position setpoint: reach j in t1 s, hold t2 s.
func MoveServoJ(j [6]float64, t1, t2, gain, alpha float64) string {
	return "move_servo_j(" + Jnt(j) + "," + Num(t1) + "," + Num(t2) + "," + Num(gain) + "," + Num(alpha) + ")"
}

// MoveSpeedJ streams a joint velocity setpoint (deg/s). All zeros ends a servo stream.
func MoveSpeedJ(dj [6]float64, t1, t2, gain, alpha float64) string {
	return "move_speed_j(" + Jnt(dj) + "," + Num(t1) + "," + Num(t2) + "," + Num(gain) + "," + Num(alpha) + ")"
}

// TaskStop halts motion.
func TaskStop() string { return "task stop" }

// TaskPause pauses the running task.
func TaskPause() string { return "task pause" }

// TaskResume resumes a paused task; afterCollision selects resume_b.
func TaskResume(afterCollision bool) string {
	if afterCollision {
		return "task resume_b"
	}
	return "task resume_a"
}

// Freedrive turns direct teaching on or off (real mode only).
func Freedrive(on bool) string {
	if on {
		return "freedrive_teach_on()"
	}
	return "freedrive_teach_off()"
}

// SetCollisionOnOff enables or disables collision detection.
func SetCollisionOnOff(on bool) string {
	if on {
		return "set_collision_onoff(1)"
	}
	return "set_collision_onoff(0)"
}

// SetCollisionThreshold sets collision sensitivity 0-1.
func SetCollisionThreshold(th float64) string { return "set_collision_th(" + Num(th) + ")" }

// SetCollisionMode sets the collision reaction mode.
func SetCollisionMode(mode int) string { return "set_collision_mode(" + strconv.Itoa(mode) + ")" }

// SetCollisionAfter sets the post-collision behaviour.
func SetCollisionAfter(mode int) string { return "set_collision_after(" + strconv.Itoa(mode) + ")" }

// SetTCPInfo sets the tool centre point offset (mm, deg).
func SetTCPInfo(p [6]float64) string {
	parts := make([]string, 6)
	for i, v := range p {
		parts[i] = Num(v)
	}
	return "set_tcp_info(" + strings.Join(parts, ",") + ")"
}

// SetPayloadInfo sets the payload mass (kg) and centre of mass (mm).
func SetPayloadInfo(kg, cx, cy, cz float64) string {
	return "set_payload_info(" + Num(kg) + "," + Num(cx) + "," + Num(cy) + "," + Num(cz) + ")"
}

// ToolOut sets the tool flange voltage (0, 12, 24) and two digital outputs.
func ToolOut(voltage, s0, s1 int) string {
	return fmt.Sprintf("tool_out %d,%d,%d", voltage, s0, s1)
}

// SetBoxDOut sets a control box digital output: -1 leave, 0 low, 1 high.
func SetBoxDOut(port, mode int) string {
	return fmt.Sprintf("set_box_dout(%d,%d)", port, mode)
}

// SetBoxAOut sets a control box analog output voltage.
func SetBoxAOut(port int, volts float64) string {
	return fmt.Sprintf("set_box_aout(%d,%s)", port, Num(volts))
}

// Print asks the box to print expr wrapped in prefix and suffix; the reply is
// info[print_*][<prefix><value><suffix>]. Use Query on the client for the
// tag-correlated form.
func Print(expr, prefix, suffix string) string {
	return "print(" + expr + ",\"" + prefix + "\", \"" + suffix + "\")"
}

// PrintTagged is the tagged print form rbpodo uses for queries:
// print(EXPR,"#tag_[", "]") which prints #tag_[value].
func PrintTagged(expr, tag string) string {
	return "print(" + expr + ",\"" + tag + "[\", \"]\")"
}

// CalcFKTCP returns the expression for controller-side forward kinematics.
func CalcFKTCP(j [6]float64) string {
	parts := make([]string, 6)
	for i, v := range j {
		parts[i] = Num(v)
	}
	return "calc_fk_tcp(" + strings.Join(parts, ",") + ")"
}

// NewTag returns a random print tag of the form #xxxxx_ like rbpodo's randstr.
func NewTag() string {
	const alphabet = "0123456789abcdefghijklmnopqrstuvwxyz"
	var b [5]byte
	_, _ = rand.Read(b[:])
	for i := range b {
		b[i] = alphabet[int(b[i])%len(alphabet)]
	}
	return "#" + string(b[:]) + "_"
}

// ParseFloats parses a comma-separated list of numbers such as a printed jnt or
// pnt value ("1.0, 2.0, 3.0").
func ParseFloats(s string) ([]float64, error) {
	s = strings.TrimSpace(s)
	s = strings.TrimPrefix(s, "jnt[")
	s = strings.TrimPrefix(s, "pnt[")
	s = strings.TrimSuffix(s, "]")
	if s == "" {
		return nil, nil
	}
	parts := strings.Split(s, ",")
	out := make([]float64, 0, len(parts))
	for _, p := range parts {
		v, err := strconv.ParseFloat(strings.TrimSpace(p), 64)
		if err != nil {
			return nil, fmt.Errorf("rb: cannot parse %q as number: %w", p, err)
		}
		out = append(out, v)
	}
	return out, nil
}
