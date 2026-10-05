package arm

import (
	"context"
	"errors"
	"fmt"
	"sort"
	"strings"
	"time"

	"github.com/viam-labs/rainbow-robotics/rb"
)

// DoCommand dispatches on key presence, e.g. {"set_speed": 30}. Unknown keys
// return an error naming the key.
func (a *rb10) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	if len(cmd) == 0 {
		return nil, errors.New("empty command; supported keys: " + strings.Join(a.commandKeys(), ", "))
	}
	out := map[string]any{}
	for key, arg := range cmd {
		h, ok := a.commandHandlers()[key]
		if !ok {
			return nil, fmt.Errorf("unknown command %q; supported keys: %s", key, strings.Join(a.commandKeys(), ", "))
		}
		res, err := h(ctx, arg)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", key, err)
		}
		for k, v := range res {
			out[k] = v
		}
	}
	return out, nil
}

// Status returns the same decoded state as DoCommand get_state.
func (a *rb10) Status(ctx context.Context) (map[string]any, error) {
	return a.getState(), nil
}

type cmdHandler func(ctx context.Context, arg any) (map[string]any, error)

func (a *rb10) commandKeys() []string {
	keys := make([]string, 0, 24)
	for k := range a.commandHandlers() {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

func (a *rb10) commandHandlers() map[string]cmdHandler {
	return map[string]cmdHandler{
		"set_speed":        a.cmdSetSpeed,
		"set_acceleration": a.cmdSetAcceleration,
		"get_speed": func(context.Context, any) (map[string]any, error) {
			s, _ := a.sessionSpeeds()
			return map[string]any{"speed_degs_per_sec": s}, nil
		},
		"get_acceleration": func(context.Context, any) (map[string]any, error) {
			_, ac := a.sessionSpeeds()
			return map[string]any{"acceleration_degs_per_sec_per_sec": ac}, nil
		},
		"get_state": func(context.Context, any) (map[string]any, error) { return a.getState(), nil },
		"get_state_description": func(context.Context, any) (map[string]any, error) {
			return map[string]any{"description": a.describeState()}, nil
		},
		"activate":           a.cmdActivate,
		"set_operation_mode": a.cmdSetOperationMode,
		"set_speed_bar":      a.cmdSetSpeedBar,
		"set_collision":      a.cmdSetCollision,
		"clear_error":        a.cmdClearError,
		"set_payload":        a.cmdSetPayload,
		"power_off":          a.cmdPowerOff,
		"get_io":             func(context.Context, any) (map[string]any, error) { return a.getIO() },
		"set_box_dout":       a.cmdSetBoxDOut,
		"set_box_aout":       a.cmdSetBoxAOut,
		"set_tool_out":       a.cmdSetToolOut,
		"get_ft":             func(context.Context, any) (map[string]any, error) { return a.getFT() },
		"calc_fk":            a.cmdCalcFK,
		"eval":               a.cmdEval,
	}
}

func toFloat(v any) (float64, error) {
	switch n := v.(type) {
	case float64:
		return n, nil
	case float32:
		return float64(n), nil
	case int:
		return float64(n), nil
	case int32:
		return float64(n), nil
	case int64:
		return float64(n), nil
	}
	return 0, fmt.Errorf("expected a number, got %T", v)
}

func toBool(v any) (bool, error) {
	if b, ok := v.(bool); ok {
		return b, nil
	}
	return false, fmt.Errorf("expected a boolean, got %T", v)
}

func toMap(v any) (map[string]any, error) {
	if m, ok := v.(map[string]any); ok {
		return m, nil
	}
	return nil, fmt.Errorf("expected an object, got %T", v)
}

func toFloats(v any) ([]float64, error) {
	list, ok := v.([]any)
	if !ok {
		return nil, fmt.Errorf("expected a list of numbers, got %T", v)
	}
	out := make([]float64, len(list))
	for i, e := range list {
		f, err := toFloat(e)
		if err != nil {
			return nil, err
		}
		out[i] = f
	}
	return out, nil
}

func (a *rb10) cmdSetSpeed(_ context.Context, arg any) (map[string]any, error) {
	v, err := toFloat(arg)
	if err != nil {
		return nil, err
	}
	if v <= 0 {
		return nil, errors.New("speed must be > 0")
	}
	v = a.clampSpeed(v)
	a.mu.Lock()
	a.speed = v
	a.mu.Unlock()
	return map[string]any{"speed_degs_per_sec": v}, nil
}

func (a *rb10) cmdSetAcceleration(_ context.Context, arg any) (map[string]any, error) {
	v, err := toFloat(arg)
	if err != nil {
		return nil, err
	}
	if v <= 0 {
		return nil, errors.New("acceleration must be > 0")
	}
	v = a.clampAccel(v)
	a.mu.Lock()
	a.accel = v
	a.mu.Unlock()
	return map[string]any{"acceleration_degs_per_sec_per_sec": v}, nil
}

func (a *rb10) cmdActivate(ctx context.Context, _ any) (map[string]any, error) {
	if snap, ok := a.client.Snapshot(); ok && snap.State.Activated() {
		return map[string]any{"activated": true}, nil
	}
	if err := a.activate(ctx); err != nil {
		return nil, err
	}
	// Re-run the rest of the startup sequence so mode and settings are applied.
	if err := a.startup(ctx); err != nil {
		return nil, err
	}
	return map[string]any{"activated": true}, nil
}

func (a *rb10) cmdSetOperationMode(ctx context.Context, arg any) (map[string]any, error) {
	mode, ok := arg.(string)
	if !ok || (mode != ModeReal && mode != ModeSimulation) {
		return nil, fmt.Errorf("expected %q or %q", ModeReal, ModeSimulation)
	}
	if moving, _ := a.IsMoving(ctx); moving {
		return nil, errors.New("refused while the arm is moving")
	}
	t := time.Now()
	if err := a.client.Send(ctx, rb.PGMode(mode == ModeReal)); err != nil {
		return nil, err
	}
	if _, err := a.client.WaitState(ctx, 2*time.Second, func(st *rb.SystemState) bool { return st.Simulation() == (mode == ModeSimulation) }); err != nil {
		if ce := a.client.LastErrorSince(t); ce != nil {
			return nil, ce
		}
		return nil, fmt.Errorf("controller did not switch mode: %w", err)
	}
	a.mu.Lock()
	a.cfg.OperationMode = mode
	a.mu.Unlock()
	return map[string]any{"operation_mode": mode}, nil
}

func (a *rb10) cmdSetSpeedBar(ctx context.Context, arg any) (map[string]any, error) {
	v, err := toFloat(arg)
	if err != nil {
		return nil, err
	}
	if v < 0 || v > 1 {
		return nil, errors.New("speed_bar must be 0-1")
	}
	if err := a.client.Send(ctx, rb.SetSpeedBar(v)); err != nil {
		return nil, err
	}
	return map[string]any{"speed_bar": v}, nil
}

func (a *rb10) cmdSetCollision(ctx context.Context, arg any) (map[string]any, error) {
	m, err := toMap(arg)
	if err != nil {
		return nil, err
	}
	out := map[string]any{}
	if v, ok := m["enabled"]; ok {
		b, err := toBool(v)
		if err != nil {
			return nil, err
		}
		if err := a.client.Send(ctx, rb.SetCollisionOnOff(b)); err != nil {
			return nil, err
		}
		out["enabled"] = b
	}
	if v, ok := m["sensitivity"]; ok {
		f, err := toFloat(v)
		if err != nil {
			return nil, err
		}
		if f < 0 || f > 1 {
			return nil, errors.New("sensitivity must be 0-1")
		}
		if err := a.client.Send(ctx, rb.SetCollisionThreshold(f)); err != nil {
			return nil, err
		}
		out["sensitivity"] = f
	}
	if v, ok := m["mode"]; ok {
		f, err := toFloat(v)
		if err != nil {
			return nil, err
		}
		if err := a.client.Send(ctx, rb.SetCollisionMode(int(f))); err != nil {
			return nil, err
		}
		out["mode"] = int(f)
	}
	if v, ok := m["after"]; ok {
		f, err := toFloat(v)
		if err != nil {
			return nil, err
		}
		if err := a.client.Send(ctx, rb.SetCollisionAfter(int(f))); err != nil {
			return nil, err
		}
		out["after"] = int(f)
	}
	if len(out) == 0 {
		return nil, errors.New("expected at least one of enabled, sensitivity, mode, after")
	}
	return out, nil
}

// cmdClearError attempts recovery after a collision or soft e-stop. The exact
// recovery script is not documented in rbpodo; this sends "task stop" and then
// reports which fault conditions remain. Verify on hardware.
func (a *rb10) cmdClearError(ctx context.Context, _ any) (map[string]any, error) {
	if err := a.client.Send(ctx, rb.TaskStop()); err != nil {
		return nil, err
	}
	snap, err := a.client.NextSnapshot(ctx, time.Now(), time.Second)
	if err != nil {
		return nil, err
	}
	reason := a.faultReason(&snap.State)
	if reason == "" {
		a.setState(stateReady)
	}
	return map[string]any{"cleared": reason == "", "remaining_fault": reason}, nil
}

func (a *rb10) cmdSetPayload(ctx context.Context, arg any) (map[string]any, error) {
	m, err := toMap(arg)
	if err != nil {
		return nil, err
	}
	kg, err := toFloat(m["kg"])
	if err != nil {
		return nil, fmt.Errorf("kg: %w", err)
	}
	if kg < 0 {
		return nil, errors.New("kg must be >= 0")
	}
	var cog [3]float64
	if v, ok := m["cog_mm"]; ok {
		f, err := toFloats(v)
		if err != nil {
			return nil, fmt.Errorf("cog_mm: %w", err)
		}
		if len(f) != 3 {
			return nil, errors.New("cog_mm must have 3 values")
		}
		copy(cog[:], f)
	}
	if err := a.client.Send(ctx, rb.SetPayloadInfo(kg, cog[0], cog[1], cog[2])); err != nil {
		return nil, err
	}
	return map[string]any{"kg": kg, "cog_mm": cog[:]}, nil
}

func (a *rb10) cmdPowerOff(ctx context.Context, _ any) (map[string]any, error) {
	if moving, _ := a.IsMoving(ctx); moving {
		return nil, errors.New("refused while the arm is moving")
	}
	if err := a.client.Send(ctx, rb.Shutdown()); err != nil {
		return nil, err
	}
	return map[string]any{"power_off": true}, nil
}

func (a *rb10) cmdSetBoxDOut(ctx context.Context, arg any) (map[string]any, error) {
	m, err := toMap(arg)
	if err != nil {
		return nil, err
	}
	port, err := toFloat(m["port"])
	if err != nil {
		return nil, fmt.Errorf("port: %w", err)
	}
	value, err := toFloat(m["value"])
	if err != nil {
		return nil, fmt.Errorf("value: %w", err)
	}
	if port < 0 || port > 15 || value < -1 || value > 1 {
		return nil, errors.New("port must be 0-15 and value -1, 0 or 1")
	}
	if err := a.client.Send(ctx, rb.SetBoxDOut(int(port), int(value))); err != nil {
		return nil, err
	}
	return map[string]any{"port": int(port), "value": int(value)}, nil
}

func (a *rb10) cmdSetBoxAOut(ctx context.Context, arg any) (map[string]any, error) {
	m, err := toMap(arg)
	if err != nil {
		return nil, err
	}
	port, err := toFloat(m["port"])
	if err != nil {
		return nil, fmt.Errorf("port: %w", err)
	}
	volts, err := toFloat(m["value"])
	if err != nil {
		return nil, fmt.Errorf("value: %w", err)
	}
	if port < 0 || port > 3 || volts < 0 || volts > 10 {
		return nil, errors.New("port must be 0-3 and value 0-10 volts")
	}
	if err := a.client.Send(ctx, rb.SetBoxAOut(int(port), volts)); err != nil {
		return nil, err
	}
	return map[string]any{"port": int(port), "value": volts}, nil
}

func (a *rb10) cmdSetToolOut(ctx context.Context, arg any) (map[string]any, error) {
	m, err := toMap(arg)
	if err != nil {
		return nil, err
	}
	volts, err := toFloat(m["voltage"])
	if err != nil {
		return nil, fmt.Errorf("voltage: %w", err)
	}
	if volts != 0 && volts != 12 && volts != 24 {
		return nil, errors.New("voltage must be 0, 12 or 24")
	}
	s0, err := toFloat(m["out0"])
	if err != nil {
		return nil, fmt.Errorf("out0: %w", err)
	}
	s1, err := toFloat(m["out1"])
	if err != nil {
		return nil, fmt.Errorf("out1: %w", err)
	}
	if err := a.client.Send(ctx, rb.ToolOut(int(volts), int(s0), int(s1))); err != nil {
		return nil, err
	}
	return map[string]any{"voltage": int(volts), "out0": int(s0), "out1": int(s1)}, nil
}

func (a *rb10) cmdCalcFK(ctx context.Context, arg any) (map[string]any, error) {
	f, err := toFloats(arg)
	if err != nil {
		return nil, err
	}
	if len(f) != JointCount {
		return nil, fmt.Errorf("expected %d joint angles in degrees", JointCount)
	}
	var j [6]float64
	copy(j[:], f)
	v, err := a.client.Query(ctx, rb.CalcFKTCP(j))
	if err != nil {
		return nil, err
	}
	vals, err := rb.ParseFloats(v)
	if err != nil {
		return nil, err
	}
	out := map[string]any{"controller_tcp": vals}
	if len(vals) == 6 {
		var p [6]float64
		copy(p[:], vals)
		pose := RainbowPoseToSpatial(p)
		out["controller_pose"] = poseToMap(pose)
	}
	if modelPose, err := a.model.Transform(degsToInputs(j)); err == nil {
		out["model_pose"] = poseToMap(modelPose)
	}
	return out, nil
}

func (a *rb10) cmdEval(ctx context.Context, arg any) (map[string]any, error) {
	if !a.cfg.AllowRawCommands {
		return nil, errors.New("raw script commands are disabled; set allow_raw_commands: true to enable")
	}
	script, ok := arg.(string)
	if !ok || strings.TrimSpace(script) == "" {
		return nil, errors.New("expected a script string")
	}
	if strings.ContainsAny(script, "\r\n") {
		return nil, errors.New("script must be a single line")
	}
	t := time.Now()
	if err := a.client.Send(ctx, script); err != nil {
		return nil, err
	}
	out := map[string]any{"acked": true}
	if ce := a.client.LastErrorSince(t); ce != nil {
		out["controller_error"] = ce.Error()
	}
	return out, nil
}

// getState decodes the latest snapshot into a map.
func (a *rb10) getState() map[string]any {
	snap, ok := a.client.Snapshot()
	cmdOK, dataOK := a.client.Connected()
	a.mu.Lock()
	ready, startupErr, lastState := a.ready, a.startupErr, a.lastState
	a.mu.Unlock()
	out := map[string]any{
		"description":        a.describeState(),
		"module_state":       string(lastState),
		"ready":              ready,
		"command_connected":  cmdOK,
		"data_connected":     dataOK,
		"operation_mode_cfg": a.cfg.OperationMode,
	}
	if startupErr != nil {
		out["startup_error"] = startupErr.Error()
	}
	if !ok {
		out["snapshot_age_ms"] = nil
		return out
	}
	st := &snap.State
	mode := ModeReal
	if st.Simulation() {
		mode = ModeSimulation
	}
	out["snapshot_age_ms"] = float64(snap.Age().Milliseconds())
	out["joints_deg"] = f32s(st.JntRef[:])
	out["joints_encoder_deg"] = f32s(st.JntAng[:])
	out["tcp_ref"] = f32s(st.TCPRef[:])
	out["tcp_pos"] = f32s(st.TCPPos[:])
	out["robot_state"] = float64(st.RobotState)
	out["task_state"] = float64(st.TaskState)
	out["moving"] = st.Moving()
	out["operation_mode"] = mode
	out["init_state_info"] = float64(st.InitStateInfo)
	out["init_error"] = float64(st.InitError)
	out["activated"] = st.Activated()
	out["estop"] = st.EStop()
	out["arm_power"] = st.ArmPowered()
	out["freedrive"] = st.Freedrive()
	out["collision_detection"] = st.CollisionDetectOnOff == 1
	out["collision_occurred"] = st.CollisionStopped()
	out["soft_estop"] = st.OpStatSoftEstopOccur != 0
	out["self_collision"] = st.OpStatSelfCollision != 0
	out["ems_flag"] = float64(st.OpStatEMSFlag)
	out["sos_flag"] = float64(st.OpStatSOSFlag)
	out["speed_bar"] = float64(st.DefaultSpeed)
	out["information_chunk_1"] = float64(st.InformationChunk1)
	out["joint_info"] = i32s(st.JntInfo[:])
	out["joint_faults"] = st.JointFaults()
	out["joint_temperature_c"] = f32s(st.JntTemperature[:])
	out["joint_current_a"] = f32s(st.JntCur[:])
	out["eft"] = map[string]any{
		"fx": float64(st.EFTFx), "fy": float64(st.EFTFy), "fz": float64(st.EFTFz),
		"mx": float64(st.EFTMx), "my": float64(st.EFTMy), "mz": float64(st.EFTMz),
	}
	return out
}

// describeState returns one human string, e.g. "real / activated / idle".
func (a *rb10) describeState() string {
	snap, ok := a.client.Snapshot()
	if !ok {
		return "no state from control box"
	}
	st := &snap.State
	parts := []string{}
	if st.Simulation() {
		parts = append(parts, "simulation")
	} else {
		parts = append(parts, "real")
	}
	if st.Activated() {
		parts = append(parts, "activated")
	} else {
		parts = append(parts, fmt.Sprintf("not activated (%d/6)", st.InitStateInfo))
	}
	if st.Moving() {
		parts = append(parts, "moving")
	} else {
		parts = append(parts, "idle")
	}
	if st.Freedrive() {
		parts = append(parts, "freedrive")
	}
	if reason := a.faultReason(st); reason != "" {
		parts = append(parts, reason)
	}
	if snap.Age() > snapshotMaxAge {
		parts = append(parts, fmt.Sprintf("stale (%v)", snap.Age().Round(time.Millisecond)))
	}
	return strings.Join(parts, " / ")
}

func (a *rb10) getIO() (map[string]any, error) {
	snap, err := a.snapshot()
	if err != nil {
		return nil, err
	}
	st := &snap.State
	return map[string]any{
		"box_digital_in":   i32s(st.DigitalIn[:]),
		"box_digital_out":  i32s(st.DigitalOut[:]),
		"box_analog_in":    f32s(st.AnalogIn[:]),
		"box_analog_out":   f32s(st.AnalogOut[:]),
		"tool_digital_in":  i32s(st.TFBDigitalIn[:]),
		"tool_digital_out": i32s(st.TFBDigitalOut[:]),
		"tool_analog_in":   f32s(st.TFBAnalogIn[:]),
		"tool_voltage_out": float64(st.TFBVoltageOut),
	}, nil
}

func (a *rb10) getFT() (map[string]any, error) {
	snap, err := a.snapshot()
	if err != nil {
		return nil, err
	}
	st := &snap.State
	return map[string]any{
		"fx": float64(st.EFTFx), "fy": float64(st.EFTFy), "fz": float64(st.EFTFz),
		"mx": float64(st.EFTMx), "my": float64(st.EFTMy), "mz": float64(st.EFTMz),
	}, nil
}

func f32s(v []float32) []any {
	out := make([]any, len(v))
	for i, x := range v {
		out[i] = float64(x)
	}
	return out
}

func i32s(v []int32) []any {
	out := make([]any, len(v))
	for i, x := range v {
		out[i] = float64(x)
	}
	return out
}
