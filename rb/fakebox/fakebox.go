// Package fakebox is an in-process imitation of a Rainbow Robotics control box.
// It listens on two ephemeral TCP ports, accepts script lines on the command
// port (replying with the ACK and broadcasting events), serves systemSTAT
// packets on the data port, and simulates joint motion for move_j, move_jb2
// batches and move_servo_j setpoints. Tests use it to drive the whole client
// and arm stack without hardware.
package fakebox

import (
	"bufio"
	"fmt"
	"math"
	"net"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/viam-labs/rainbow-robotics/rb"
)

// Options tune the fake box.
type Options struct {
	// Activated starts the box with init_state_info = 6.
	Activated bool
	// Real starts the box in real mode instead of the boot default (simulation).
	Real bool
	// Tick is the motion simulation period (default 2ms).
	Tick time.Duration
	// ActivationDuration is how long "mc jall init" takes (default 50ms).
	ActivationDuration time.Duration
	// FK computes calc_fk_tcp replies (x,y,z mm, rx,ry,rz deg). Zeros if nil.
	FK func(j [6]float64) [6]float64
	// FragmentPackets splits every state packet into two writes with a pause.
	FragmentPackets bool
	// Version is reported by get_control_box_info().
	Version string
	// EchoUnknownAsNotAllowed makes unrecognised commands answer
	// "The command is not allowed" instead of the ACK (the real box does this
	// for the first command on a connection).
	EchoUnknownAsNotAllowed bool
	// CommandPort and DataPort pin the listening ports; zero picks ephemeral ones.
	CommandPort int
	DataPort    int
	// Addr is the interface to listen on (default 127.0.0.1).
	Addr string
}

// Box is a running fake control box.
type Box struct {
	opts Options

	cmdLn  net.Listener
	dataLn net.Listener

	mu      sync.Mutex
	st      rb.SystemState
	clients map[net.Conn]struct{}
	cmds    []string
	closed  bool

	// motion simulation
	target      [6]float64
	moving      bool
	speed       float64 // deg/s for the current move
	queue       []jb2Entry
	jb2Building []jb2Entry
	servoActive bool
	servoUntil  time.Time
	printSeq    int
	servoCount  int
	activating  bool

	done chan struct{}
	wg   sync.WaitGroup
}

type jb2Entry struct {
	j     [6]float64
	speed float64
}

// Start listens on 127.0.0.1 ephemeral ports.
func Start(opts Options) (*Box, error) {
	if opts.Tick == 0 {
		opts.Tick = 2 * time.Millisecond
	}
	if opts.ActivationDuration == 0 {
		opts.ActivationDuration = 50 * time.Millisecond
	}
	if opts.Version == "" {
		opts.Version = "fakebox-1.0"
	}
	if opts.Addr == "" {
		opts.Addr = "127.0.0.1"
	}
	cmdLn, err := net.Listen("tcp", net.JoinHostPort(opts.Addr, strconv.Itoa(opts.CommandPort)))
	if err != nil {
		return nil, err
	}
	dataLn, err := net.Listen("tcp", net.JoinHostPort(opts.Addr, strconv.Itoa(opts.DataPort)))
	if err != nil {
		_ = cmdLn.Close()
		return nil, err
	}
	b := &Box{
		opts:    opts,
		cmdLn:   cmdLn,
		dataLn:  dataLn,
		clients: map[net.Conn]struct{}{},
		done:    make(chan struct{}),
	}
	b.st.RobotState = rb.RobotStateIdle
	b.st.TaskState = rb.TaskStateIdle
	b.st.DefaultSpeed = 1
	b.st.InformationChunk1 = rb.InfoBit48VIn | rb.InfoBit48VOut | rb.InfoBit24VIn
	if opts.Activated {
		b.st.InitStateInfo = rb.InitStateActivated
		b.st.InformationChunk1 |= rb.InfoBitArmPower
	}
	if opts.Real {
		b.st.RealVsSimulationMode = rb.OperationModeReal
	} else {
		b.st.RealVsSimulationMode = rb.OperationModeSimulation
	}
	b.wg.Add(3)
	go func() { defer b.wg.Done(); b.accept(cmdLn, b.serveCommand) }()
	go func() { defer b.wg.Done(); b.accept(dataLn, b.serveData) }()
	go func() { defer b.wg.Done(); b.simulate() }()
	return b, nil
}

// Host returns the address to dial.
func (b *Box) Host() string { return b.opts.Addr }

// CommandPort returns the command port.
func (b *Box) CommandPort() int { return b.cmdLn.Addr().(*net.TCPAddr).Port }

// DataPort returns the data port.
func (b *Box) DataPort() int { return b.dataLn.Addr().(*net.TCPAddr).Port }

// Close stops the box.
func (b *Box) Close() {
	b.mu.Lock()
	if b.closed {
		b.mu.Unlock()
		return
	}
	b.closed = true
	close(b.done)
	for c := range b.clients {
		_ = c.Close()
	}
	b.mu.Unlock()
	_ = b.cmdLn.Close()
	_ = b.dataLn.Close()
	b.wg.Wait()
}

// DropConnections closes every client connection but keeps listening, to
// exercise reconnect logic.
func (b *Box) DropConnections() {
	b.mu.Lock()
	defer b.mu.Unlock()
	for c := range b.clients {
		_ = c.Close()
	}
}

// Commands returns every script line received so far.
func (b *Box) Commands() []string {
	b.mu.Lock()
	defer b.mu.Unlock()
	return append([]string(nil), b.cmds...)
}

// ClearCommands forgets the received command log.
func (b *Box) ClearCommands() {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.cmds = nil
}

// State returns a copy of the current state.
func (b *Box) State() rb.SystemState {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.st
}

// Mutate applies f to the state under the lock.
func (b *Box) Mutate(f func(st *rb.SystemState)) {
	b.mu.Lock()
	defer b.mu.Unlock()
	f(&b.st)
}

// Joints returns the current commanded joint angles (deg).
func (b *Box) Joints() [6]float64 {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.st.Joints()
}

// SetJoints teleports the arm.
func (b *Box) SetJoints(j [6]float64) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.setJointsLocked(j)
	b.target = j
}

// Moving reports whether the simulated arm is executing a move.
func (b *Box) Moving() bool {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.st.Moving()
}

// SetEStop asserts or clears e-stop input 1, stopping any motion.
func (b *Box) SetEStop(on bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if on {
		b.st.InformationChunk1 |= rb.InfoBitEStop1
		b.stopMotionLocked()
		b.broadcastLocked("error[code][0]")
	} else {
		b.st.InformationChunk1 &^= rb.InfoBitEStop1
	}
}

// SetCollision asserts or clears the collision-detected flag, stopping motion.
func (b *Box) SetCollision(on bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if on {
		b.st.OpStatCollisionOccur = 1
		b.stopMotionLocked()
		b.broadcastLocked("error[code][3]")
	} else {
		b.st.OpStatCollisionOccur = 0
	}
}

// Broadcast sends a raw line to every command client.
func (b *Box) Broadcast(line string) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.broadcastLocked(line)
}

// ServoSetpoints returns how many move_servo_j commands have been received.
func (b *Box) ServoSetpoints() int {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.servoCount
}

// ---------------------------------------------------------------------------

func (b *Box) accept(ln net.Listener, serve func(net.Conn)) {
	for {
		conn, err := ln.Accept()
		if err != nil {
			return
		}
		b.mu.Lock()
		if b.closed {
			b.mu.Unlock()
			_ = conn.Close()
			return
		}
		b.clients[conn] = struct{}{}
		b.mu.Unlock()
		b.wg.Add(1)
		go func() {
			defer b.wg.Done()
			serve(conn)
			b.mu.Lock()
			delete(b.clients, conn)
			b.mu.Unlock()
			_ = conn.Close()
		}()
	}
}

func (b *Box) serveData(conn net.Conn) {
	sc := bufio.NewScanner(conn)
	for sc.Scan() {
		if strings.TrimSpace(sc.Text()) != "reqdata" {
			continue
		}
		b.mu.Lock()
		pkt := rb.EncodeSystemState(b.st)
		b.mu.Unlock()
		if b.opts.FragmentPackets {
			cut := len(pkt) / 3
			if _, err := conn.Write(pkt[:cut]); err != nil {
				return
			}
			time.Sleep(time.Millisecond)
			if _, err := conn.Write(pkt[cut:]); err != nil {
				return
			}
			continue
		}
		if _, err := conn.Write(pkt); err != nil {
			return
		}
	}
}

func (b *Box) broadcastLocked(line string) {
	for c := range b.clients {
		if c.LocalAddr().(*net.TCPAddr).Port != b.CommandPort() {
			continue
		}
		_ = c.SetWriteDeadline(time.Now().Add(time.Second))
		_, _ = c.Write([]byte(line + "\n"))
	}
}

var (
	reMoveJ       = regexp.MustCompile(`^move_j\(jnt\[([^\]]*)\],([^,]+),([^)]+)\)$`)
	reMoveJB2Add  = regexp.MustCompile(`^move_jb2_add\(jnt\[([^\]]*)\],([^,]+),([^,]+),([^,]+),([^)]+)\)$`)
	reMoveServoJ  = regexp.MustCompile(`^move_servo_j\(jnt\[([^\]]*)\],([^,]+),([^,]+),([^,]+),([^)]+)\)$`)
	reMoveSpeedJ  = regexp.MustCompile(`^move_speed_j\(jnt\[([^\]]*)\],([^,]+),([^,]+),([^,]+),([^)]+)\)$`)
	rePrint       = regexp.MustCompile(`^print\((.*),"([^"]*)",\s*"([^"]*)"\)$`)
	reSetSpeedBar = regexp.MustCompile(`^set_speed_bar\(([^)]+)\)$`)
	reCalcFK      = regexp.MustCompile(`^calc_fk_tcp\(([^)]*)\)$`)
	reCollOnOff   = regexp.MustCompile(`^set_collision_onoff\(([01])\)$`)
	reCollTh      = regexp.MustCompile(`^set_collision_th\(([^)]+)\)$`)
	reCollMode    = regexp.MustCompile(`^set_collision_mode\((\d+)\)$`)
	reCollAfter   = regexp.MustCompile(`^set_collision_after\((\d+)\)$`)
	rePayload     = regexp.MustCompile(`^set_payload_info\(([^)]*)\)$`)
	reTCPInfo     = regexp.MustCompile(`^set_tcp_info\(([^)]*)\)$`)
	reBoxDOut     = regexp.MustCompile(`^set_box_dout\((\d+),(-?\d+)\)$`)
	reBoxAOut     = regexp.MustCompile(`^set_box_aout\((\d+),([^)]+)\)$`)
	reToolOut     = regexp.MustCompile(`^tool_out (\d+),(\d+),(\d+)$`)
)

func (b *Box) serveCommand(conn net.Conn) {
	sc := bufio.NewScanner(conn)
	sc.Buffer(make([]byte, 0, 64*1024), 1024*1024)
	for sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" {
			continue
		}
		reply, ok := b.handle(line)
		var out string
		switch {
		case !ok && b.opts.EchoUnknownAsNotAllowed:
			out = rb.NotAllowedMessage + "\n"
		default:
			if reply != "" {
				out = reply + "\n"
			}
			out += rb.ACKMessage + "\n"
		}
		_ = conn.SetWriteDeadline(time.Now().Add(time.Second))
		if _, err := conn.Write([]byte(out)); err != nil {
			return
		}
	}
}

// handle executes one script line. It returns any immediate (unicast) reply
// line to send before the ACK and whether the command was recognised.
func (b *Box) handle(line string) (string, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.cmds = append(b.cmds, line)

	switch {
	case line == "mc jall init":
		if b.st.InitStateInfo != rb.InitStateActivated && !b.activating {
			b.activating = true
			go b.activate()
		} else if b.st.InitStateInfo == rb.InitStateActivated {
			b.broadcastLocked("info[activation][done]")
		}
		return "", true
	case line == "pgmode real":
		if b.st.InitStateInfo != rb.InitStateActivated {
			b.broadcastLocked("error[code][186]")
			return "", true
		}
		b.st.RealVsSimulationMode = rb.OperationModeReal
		return "", true
	case line == "pgmode simulation":
		b.st.RealVsSimulationMode = rb.OperationModeSimulation
		return "", true
	case line == "task stop":
		b.stopMotionLocked()
		return "", true
	case line == "task pause":
		b.st.TaskState = rb.TaskStatePaused
		return "", true
	case line == "task resume_a", line == "task resume_b":
		b.st.TaskState = rb.TaskStateIdle
		return "", true
	case line == "freedrive_teach_on()":
		if b.st.Simulation() {
			b.broadcastLocked("error[code][204]")
			return "", true
		}
		b.st.IsFreedriveMode = 1
		return "", true
	case line == "freedrive_teach_off()":
		b.st.IsFreedriveMode = 0
		return "", true
	case line == "move_jb2_clear()":
		b.jb2Building = nil
		return "", true
	case line == "move_jb2_run()":
		if len(b.jb2Building) == 0 {
			return "", true
		}
		b.queue = append([]jb2Entry(nil), b.jb2Building...)
		b.jb2Building = nil
		b.startNextLocked()
		return "", true
	case line == "AvadaKedavra()":
		b.st.InitStateInfo = 0
		b.st.InformationChunk1 &^= rb.InfoBitArmPower
		b.stopMotionLocked()
		b.broadcastLocked("error[code][0]")
		return "", true
	case line == "get_control_box_info()":
		return fmt.Sprintf("info[controlbox][%s,%d]", b.opts.Version, 10), true
	}

	if m := reMoveJ.FindStringSubmatch(line); m != nil {
		j, err := parse6(m[1])
		if err != nil {
			return "", false
		}
		speed, _ := strconv.ParseFloat(strings.TrimSpace(m[2]), 64)
		b.queue = []jb2Entry{{j: j, speed: speed}}
		b.startNextLocked()
		return "", true
	}
	if m := reMoveJB2Add.FindStringSubmatch(line); m != nil {
		j, err := parse6(m[1])
		if err != nil {
			return "", false
		}
		speed, _ := strconv.ParseFloat(strings.TrimSpace(m[2]), 64)
		b.jb2Building = append(b.jb2Building, jb2Entry{j: j, speed: speed})
		return "", true
	}
	if m := reMoveServoJ.FindStringSubmatch(line); m != nil {
		j, err := parse6(m[1])
		if err != nil {
			return "", false
		}
		t1, _ := strconv.ParseFloat(strings.TrimSpace(m[2]), 64)
		t2, _ := strconv.ParseFloat(strings.TrimSpace(m[3]), 64)
		b.servoCount++
		b.servoActive = true
		b.servoUntil = time.Now().Add(time.Duration((t1 + t2) * float64(time.Second)))
		b.queue = nil
		b.moving = false
		b.setJointsLocked(j)
		b.target = j
		if b.st.RobotState != rb.RobotStateMoving {
			b.st.RobotState = rb.RobotStateMoving
			b.broadcastLocked("info[motion_changed][1]")
		}
		return "", true
	}
	if m := reMoveSpeedJ.FindStringSubmatch(line); m != nil {
		dj, err := parse6(m[1])
		if err != nil {
			return "", false
		}
		allZero := true
		for _, v := range dj {
			if v != 0 {
				allZero = false
			}
		}
		if allZero {
			b.servoActive = false
			if b.st.RobotState == rb.RobotStateMoving {
				b.st.RobotState = rb.RobotStateIdle
				b.broadcastLocked("info[motion_changed][0]")
			}
		}
		return "", true
	}
	if m := rePrint.FindStringSubmatch(line); m != nil {
		b.printSeq++
		value := b.evalLocked(strings.TrimSpace(m[1]))
		reply := fmt.Sprintf("info[print_%d][%s%s%s]", b.printSeq, m[2], value, m[3])
		return reply, true
	}
	if m := reSetSpeedBar.FindStringSubmatch(line); m != nil {
		v, _ := strconv.ParseFloat(m[1], 64)
		b.st.DefaultSpeed = float32(v)
		return "", true
	}
	if m := reCollOnOff.FindStringSubmatch(line); m != nil {
		v, _ := strconv.Atoi(m[1])
		b.st.CollisionDetectOnOff = int32(v)
		return "", true
	}
	if m := reBoxDOut.FindStringSubmatch(line); m != nil {
		port, _ := strconv.Atoi(m[1])
		mode, _ := strconv.Atoi(m[2])
		if port >= 0 && port < 16 && mode >= 0 {
			b.st.DigitalOut[port] = int32(mode)
		}
		return "", true
	}
	if m := reBoxAOut.FindStringSubmatch(line); m != nil {
		port, _ := strconv.Atoi(m[1])
		v, _ := strconv.ParseFloat(m[2], 64)
		if port >= 0 && port < 4 {
			b.st.AnalogOut[port] = float32(v)
		}
		return "", true
	}
	if m := reToolOut.FindStringSubmatch(line); m != nil {
		v, _ := strconv.Atoi(m[1])
		s0, _ := strconv.Atoi(m[2])
		s1, _ := strconv.Atoi(m[3])
		b.st.TFBVoltageOut = float32(v)
		b.st.TFBDigitalOut = [2]int32{int32(s0), int32(s1)}
		return "", true
	}
	if reCollTh.MatchString(line) || reCollMode.MatchString(line) || reCollAfter.MatchString(line) ||
		rePayload.MatchString(line) || reTCPInfo.MatchString(line) {
		return "", true
	}
	return "", false
}

func (b *Box) evalLocked(expr string) string {
	if strings.HasPrefix(expr, `"`) && strings.HasSuffix(expr, `"`) && len(expr) >= 2 {
		return expr[1 : len(expr)-1]
	}
	if m := reCalcFK.FindStringSubmatch(expr); m != nil {
		j, err := parse6(m[1])
		if err != nil {
			return "0"
		}
		var p [6]float64
		if b.opts.FK != nil {
			p = b.opts.FK(j)
		}
		parts := make([]string, 6)
		for i, v := range p {
			parts[i] = strconv.FormatFloat(v, 'f', 3, 64)
		}
		return strings.Join(parts, ", ")
	}
	switch expr {
	case "SD_ROBOT_STATE":
		return strconv.Itoa(int(b.st.RobotState))
	case "SD_INIT_STATE_INFO":
		return strconv.Itoa(int(b.st.InitStateInfo))
	case "SD_PG_MODE":
		return strconv.Itoa(int(b.st.RealVsSimulationMode))
	case "SD_IS_FREE_DRIVE_MODE":
		return strconv.Itoa(int(b.st.IsFreedriveMode))
	case "SD_DEFAULT_SPEED":
		return strconv.FormatFloat(float64(b.st.DefaultSpeed), 'f', -1, 32)
	case "get_tcp_info()", "get_tfc_info()":
		return "0, 0, 0, 0, 0, 0"
	}
	return "0"
}

func (b *Box) activate() {
	steps := 6
	for i := 1; i <= steps; i++ {
		select {
		case <-b.done:
			return
		case <-time.After(b.opts.ActivationDuration / time.Duration(steps)):
		}
		b.mu.Lock()
		b.st.InitStateInfo = int32(i)
		if i == steps {
			b.st.InformationChunk1 |= rb.InfoBitArmPower
			b.activating = false
			b.broadcastLocked("info[activation][done]")
		}
		b.mu.Unlock()
	}
}

func (b *Box) setJointsLocked(j [6]float64) {
	for i := range j {
		b.st.JntRef[i] = float32(j[i])
		if !b.st.Simulation() {
			b.st.JntAng[i] = float32(j[i])
		}
	}
}

func (b *Box) startNextLocked() {
	if len(b.queue) == 0 {
		if b.moving {
			b.moving = false
			b.st.RobotState = rb.RobotStateIdle
			b.broadcastLocked("info[motion_changed][0]")
		}
		return
	}
	next := b.queue[0]
	b.queue = b.queue[1:]
	b.target = next.j
	b.speed = next.speed
	if b.speed <= 0 {
		b.speed = 60
	}
	b.servoActive = false
	if !b.moving {
		b.moving = true
		b.st.RobotState = rb.RobotStateMoving
		b.broadcastLocked("info[motion_changed][1]")
	}
}

func (b *Box) stopMotionLocked() {
	wasMoving := b.st.RobotState == rb.RobotStateMoving
	b.queue = nil
	b.jb2Building = nil
	b.moving = false
	b.servoActive = false
	b.target = b.st.Joints()
	b.st.RobotState = rb.RobotStateIdle
	if wasMoving {
		b.broadcastLocked("info[motion_changed][0]")
	}
}

func (b *Box) simulate() {
	ticker := time.NewTicker(b.opts.Tick)
	defer ticker.Stop()
	last := time.Now()
	for {
		select {
		case <-b.done:
			return
		case now := <-ticker.C:
			dt := now.Sub(last).Seconds()
			last = now
			b.mu.Lock()
			b.st.Time += float32(dt)
			if b.servoActive && now.After(b.servoUntil) {
				b.servoActive = false
				if b.st.RobotState == rb.RobotStateMoving {
					b.st.RobotState = rb.RobotStateIdle
					b.broadcastLocked("info[motion_changed][0]")
				}
			}
			if b.moving {
				cur := b.st.Joints()
				step := b.speed * dt
				reached := true
				for i := range cur {
					d := b.target[i] - cur[i]
					if math.Abs(d) <= step {
						cur[i] = b.target[i]
					} else {
						reached = false
						if d > 0 {
							cur[i] += step
						} else {
							cur[i] -= step
						}
					}
				}
				b.setJointsLocked(cur)
				if reached {
					b.startNextLocked()
				}
			}
			b.mu.Unlock()
		}
	}
}

func parse6(s string) ([6]float64, error) {
	var out [6]float64
	vals, err := rb.ParseFloats(s)
	if err != nil {
		return out, err
	}
	if len(vals) != 6 {
		return out, fmt.Errorf("expected 6 values, got %d", len(vals))
	}
	copy(out[:], vals)
	return out, nil
}
