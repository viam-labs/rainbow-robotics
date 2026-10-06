// Package arm implements the Viam arm API for the Rainbow Robotics RB10-1300
// over the control box's native TCP script protocol.
package arm

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	commonpb "go.viam.com/api/common/v1"
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/rainbow-robotics/rb"
)

// Namespace and family shared by every model in this module.
const (
	Namespace = "viam"
	Family    = "rainbow-robotics"
)

// Model is the rb10-1300 arm model.
var Model = resource.NewModel(Namespace, Family, "rb10-1300")

func init() {
	resource.RegisterComponent(rdkarm.API, Model, resource.Registration[rdkarm.Arm, *Config]{
		Constructor: newRB10,
	})
}

// state describes the module's view of the controller, for logs and get_state_description.
type state string

const (
	stateConnecting state = "connecting"
	stateActivating state = "activating"
	stateReady      state = "ready"
	stateFaulted    state = "faulted"
	stateManual     state = "manual"
	stateClosed     state = "closed"
)

type rb10 struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	cfg    Config
	client *rb.Client
	model  referenceframe.Model
	motion motion.Service // nil when the optional dependency did not resolve
	opMgr  *operation.SingleOperationManager

	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup
	closed atomic.Bool

	mu          sync.Mutex // guards the fields below
	speed       float64    // session joint speed, deg/s
	accel       float64    // session joint acceleration, deg/s^2
	manualTimer *time.Timer
	ready       bool
	startupErr  error
	lastState   state

	streaming atomic.Bool
	// stopRequested is set by Stop while a move is running so the interrupted
	// move can say why its context ended.
	stopRequested atomic.Bool
}

func newRB10(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (rdkarm.Arm, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	return NewRB10(ctx, deps, conf.ResourceName(), *cfg, conf.Frame != nil, logger)
}

// NewRB10 constructs the arm: it parses the kinematics, dials both control box
// ports, waits for the first state packet and runs the startup sequence
// (activation, operation mode, speed bar, payload, collision settings).
// frameConfigured says whether the component has a frame in the machine
// config; without one the motion service cannot address the arm, so
// MoveToPosition plans in-process instead.
func NewRB10(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, frameConfigured bool, logger logging.Logger) (rdkarm.Arm, error) {
	cfg := conf.withDefaults()

	model, err := buildModel(name.ShortName(), cfg.UseURDF, cfg.MeshDecimationRatios, cfg.SpeedDegsPerSec, cfg.AccelerationDegsPerSec2)
	if err != nil {
		return nil, err
	}

	a := &rb10{
		Named:     name.AsNamed(),
		logger:    logger,
		cfg:       cfg,
		model:     model,
		opMgr:     operation.NewSingleOperationManager(),
		speed:     cfg.SpeedDegsPerSec,
		accel:     cfg.AccelerationDegsPerSec2,
		lastState: stateConnecting,
	}
	a.ctx, a.cancel = context.WithCancel(context.Background())

	switch {
	case deps == nil:
	case !frameConfigured:
		logger.Infow("no frame configured for this arm; MoveToPosition will plan in-process instead of using the motion service")
	default:
		if ms, err := motion.FromProvider(deps, cfg.Motion); err == nil {
			a.motion = ms
		} else {
			logger.Infow("motion service not available; MoveToPosition will plan in-process", "motion", cfg.Motion, "error", err)
		}
	}

	client, err := rb.Dial(ctx, rb.Config{
		Host:           cfg.Host,
		CommandPort:    cfg.CommandPort,
		DataPort:       cfg.DataPort,
		CommandTimeout: time.Duration(cfg.CommandTimeoutSecs * float64(time.Second)),
		PollInterval:   time.Second / time.Duration(cfg.StatePollHz),
		Logger:         logger,
	})
	if err != nil {
		a.cancel()
		return nil, err
	}
	a.client = client
	client.OnReconnect(func() {
		a.wg.Add(1)
		defer a.wg.Done()
		if err := a.startup(a.ctx); err != nil {
			a.logger.Errorw("startup after reconnect failed", "error", err)
		}
	})

	if _, err := client.NextSnapshot(ctx, time.Time{}, 3*time.Second); err != nil {
		_ = client.Close()
		a.cancel()
		return nil, fmt.Errorf("no state packet from control box within 3s: %w", err)
	}
	if err := a.startup(ctx); err != nil {
		_ = client.Close()
		a.cancel()
		return nil, err
	}
	return a, nil
}

// startup brings the controller to the configured state. It is run from the
// constructor and again after every command-connection reconnect.
func (a *rb10) startup(ctx context.Context) error {
	a.setReady(false, nil)
	a.setState(stateActivating)
	err := a.runStartup(ctx)
	if err != nil {
		a.setReady(false, err)
		a.setState(stateFaulted)
		return err
	}
	a.setReady(true, nil)
	a.setState(stateReady)
	a.logger.Infow("control box ready", "mode", a.cfg.OperationMode)
	return nil
}

func (a *rb10) runStartup(ctx context.Context) error {
	c := a.client
	if v, err := c.Query(ctx, "SD_INIT_STATE_INFO"); err == nil {
		a.logger.Debugw("controller reachable", "init_state_info", v)
	}

	snap, ok := c.Snapshot()
	if !ok {
		s, err := c.NextSnapshot(ctx, time.Time{}, 3*time.Second)
		if err != nil {
			return fmt.Errorf("no state from control box: %w", err)
		}
		snap = s
	}

	if !snap.State.Activated() {
		if !*a.cfg.ActivateOnStart {
			return fmt.Errorf("arm is not activated (init_state_info=%d) and activate_on_start is false", snap.State.InitStateInfo)
		}
		if err := a.activate(ctx); err != nil {
			return err
		}
	}

	real := a.cfg.OperationMode == ModeReal
	t := time.Now()
	if err := c.Send(ctx, rb.PGMode(real)); err != nil {
		return fmt.Errorf("setting operation mode: %w", err)
	}
	if _, err := c.WaitState(ctx, 2*time.Second, func(st *rb.SystemState) bool { return st.Simulation() == !real }); err != nil {
		if ce := c.LastErrorSince(t); ce != nil {
			return fmt.Errorf("controller refused %s mode: %w", a.cfg.OperationMode, ce)
		}
		return fmt.Errorf("controller did not switch to %s mode: %w", a.cfg.OperationMode, err)
	}

	if a.cfg.SpeedBar != nil {
		if err := c.Send(ctx, rb.SetSpeedBar(*a.cfg.SpeedBar)); err != nil {
			return fmt.Errorf("setting speed bar: %w", err)
		}
	}
	if a.cfg.PayloadKg != nil {
		var cog [3]float64
		copy(cog[:], a.cfg.PayloadCoGMM)
		if err := c.Send(ctx, rb.SetPayloadInfo(*a.cfg.PayloadKg, cog[0], cog[1], cog[2])); err != nil {
			return fmt.Errorf("setting payload: %w", err)
		}
	}
	if a.cfg.CollisionDetection != nil {
		if err := c.Send(ctx, rb.SetCollisionOnOff(*a.cfg.CollisionDetection)); err != nil {
			return fmt.Errorf("setting collision detection: %w", err)
		}
	}
	if a.cfg.CollisionSensitivity != nil {
		if err := c.Send(ctx, rb.SetCollisionThreshold(*a.cfg.CollisionSensitivity)); err != nil {
			return fmt.Errorf("setting collision sensitivity: %w", err)
		}
	}
	return nil
}

// activate sends "mc jall init" and waits for init_state_info to reach 6.
func (a *rb10) activate(ctx context.Context) error {
	c := a.client
	a.logger.Infow("activating arm (mc jall init)")
	t := time.Now()
	if err := c.Send(ctx, rb.Activate()); err != nil {
		return fmt.Errorf("sending activation: %w", err)
	}
	timeout := time.Duration(a.cfg.ActivateTimeoutSecs * float64(time.Second))
	_, err := c.WaitState(ctx, timeout, func(st *rb.SystemState) bool { return st.Activated() })
	if err != nil {
		if ce := c.LastErrorSince(t); ce != nil {
			return fmt.Errorf("activation failed: %w", ce)
		}
		snap, _ := c.Snapshot()
		return fmt.Errorf("activation did not complete within %v (init_state_info=%d, init_error=%d): %w",
			timeout, snap.State.InitStateInfo, snap.State.InitError, err)
	}
	a.logger.Infow("arm activated")
	return nil
}

func (a *rb10) setReady(ready bool, err error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.ready = ready
	a.startupErr = err
}

func (a *rb10) setState(s state) {
	a.mu.Lock()
	prev := a.lastState
	a.lastState = s
	a.mu.Unlock()
	if prev != s {
		a.logger.Infow("arm state", "from", prev, "to", s)
	}
}

func (a *rb10) sessionSpeeds() (speed, accel float64) {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.speed, a.accel
}

// clampSpeed bounds a joint speed request (deg/s) to the configured ceiling.
func (a *rb10) clampSpeed(v float64) float64 {
	if v <= 0 {
		return a.cfg.SpeedDegsPerSec
	}
	return min(v, a.cfg.MaxSpeedDegsPerSec)
}

// clampAccel bounds a joint acceleration request (deg/s^2) to the configured ceiling.
func (a *rb10) clampAccel(v float64) float64 {
	if v <= 0 {
		return a.cfg.AccelerationDegsPerSec2
	}
	if a.cfg.MaxAccelerationDegsPerSec2 > 0 {
		return min(v, a.cfg.MaxAccelerationDegsPerSec2)
	}
	return v
}

// snapshot returns the latest fresh state or an error naming why it is unusable.
func (a *rb10) snapshot() (rb.Snapshot, error) {
	if a.closed.Load() {
		return rb.Snapshot{}, errors.New("arm is closed")
	}
	snap, ok := a.client.Snapshot()
	if !ok {
		return snap, errors.New("joint positions not currently known: no state received from control box")
	}
	if age := snap.Age(); age > snapshotMaxAge {
		return snap, fmt.Errorf("joint positions not currently known: control box state is %v old", age.Round(time.Millisecond))
	}
	return snap, nil
}

// faultReason returns a non-empty string when st reports a condition under
// which the module must not command motion.
func (a *rb10) faultReason(st *rb.SystemState) string {
	switch {
	case !st.Activated():
		return fmt.Sprintf("arm is not activated (init_state_info=%d)", st.InitStateInfo)
	case st.EStop():
		return "e-stop is engaged"
	case st.CollisionStopped():
		return "collision stop is active (run clear_error)"
	case st.OpStatSoftEstopOccur != 0:
		return "controller is paused by a soft e-stop"
	case st.OpStatEMSFlag != 0:
		return "kinematic emergency stop: " + st.EMSDescription()
	case st.OpStatSOSFlag != 0:
		return "arm device error: " + st.SOSDescription()
	}
	if faults := st.JointFaults(); len(faults) > 0 {
		return "joint fault: " + strings.Join(faults, "; ")
	}
	return ""
}

// ensureControllable is the whole safety story: it refuses motion unless the
// module has just observed that the arm is activated, in the configured mode,
// not e-stopped, not collision-stopped and not in freedrive.
func (a *rb10) ensureControllable() (rb.Snapshot, error) {
	snap, err := a.snapshot()
	if err != nil {
		return snap, err
	}
	if cmdOK, _ := a.client.Connected(); !cmdOK {
		return snap, errors.New("command connection to control box is down")
	}
	a.mu.Lock()
	ready, startupErr := a.ready, a.startupErr
	a.mu.Unlock()
	if !ready {
		if startupErr != nil {
			return snap, fmt.Errorf("controller startup failed: %w", startupErr)
		}
		return snap, errors.New("controller startup sequence has not completed")
	}
	st := &snap.State
	if reason := a.faultReason(st); reason != "" {
		return snap, errors.New("cannot move: " + reason)
	}
	wantSim := a.cfg.OperationMode == ModeSimulation
	if st.Simulation() != wantSim {
		have := ModeReal
		if st.Simulation() {
			have = ModeSimulation
		}
		return snap, fmt.Errorf("cannot move: controller is in %s mode but operation_mode is %s", have, a.cfg.OperationMode)
	}
	if st.Freedrive() {
		return snap, errors.New("cannot move: arm is in manual (freedrive) mode")
	}
	return snap, nil
}

// --- simple API methods ----------------------------------------------------

func (a *rb10) JointPositions(ctx context.Context, extra map[string]any) ([]referenceframe.Input, error) {
	snap, err := a.snapshot()
	if err != nil {
		return nil, err
	}
	return degsToInputs(snap.State.Joints()), nil
}

func (a *rb10) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return a.JointPositions(ctx, nil)
}

func (a *rb10) EndPosition(ctx context.Context, extra map[string]any) (spatialmath.Pose, error) {
	inputs, err := a.JointPositions(ctx, extra)
	if err != nil {
		return nil, err
	}
	return a.model.Transform(inputs)
}

func (a *rb10) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return a.model, nil
}

func (a *rb10) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	inputs, err := a.JointPositions(ctx, extra)
	if err != nil {
		return nil, err
	}
	gif, err := a.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (a *rb10) Get3DModels(ctx context.Context, extra map[string]any) (map[string]*commonpb.Mesh, error) {
	return load3DModels(a.logger), nil
}

func (a *rb10) Properties(ctx context.Context, extra map[string]any) (rdkarm.Properties, error) {
	return rdkarm.Properties{SupportManualMode: true, SupportCartesianCommands: true}, nil
}

func (a *rb10) IsMoving(ctx context.Context) (bool, error) {
	if a.opMgr.OpRunning() {
		return true, nil
	}
	snap, ok := a.client.Snapshot()
	if !ok {
		return false, errors.New("no state received from control box")
	}
	return snap.State.Moving(), nil
}

func (a *rb10) ManualMode(ctx context.Context, extra map[string]any) (bool, error) {
	snap, err := a.snapshot()
	if err != nil {
		return false, err
	}
	return snap.State.Freedrive(), nil
}

// SetManualMode toggles the controller's freedrive (direct teaching) mode.
func (a *rb10) SetManualMode(ctx context.Context, manualMode bool, enabledFor time.Duration, extra map[string]any) error {
	a.mu.Lock()
	if a.manualTimer != nil {
		a.manualTimer.Stop()
		a.manualTimer = nil
	}
	a.mu.Unlock()

	if !manualMode {
		t := time.Now()
		if err := a.client.Send(ctx, rb.Freedrive(false)); err != nil {
			return err
		}
		if _, err := a.client.WaitState(ctx, 2*time.Second, func(st *rb.SystemState) bool { return !st.Freedrive() }); err != nil {
			if ce := a.client.LastErrorSince(t); ce != nil {
				return ce
			}
			return fmt.Errorf("controller did not leave freedrive: %w", err)
		}
		a.setState(stateReady)
		return nil
	}

	if a.cfg.OperationMode == ModeSimulation {
		return errors.New("manual mode requires operation_mode \"real\" (controller error 204: freedrive is only available in real mode)")
	}
	snap, err := a.snapshot()
	if err != nil {
		return err
	}
	if reason := a.faultReason(&snap.State); reason != "" {
		return errors.New("cannot enter manual mode: " + reason)
	}
	if snap.State.Moving() {
		return errors.New("cannot enter manual mode while the arm is moving")
	}
	t := time.Now()
	if err := a.client.Send(ctx, rb.Freedrive(true)); err != nil {
		return err
	}
	if _, err := a.client.WaitState(ctx, 2*time.Second, func(st *rb.SystemState) bool { return st.Freedrive() }); err != nil {
		if ce := a.client.LastErrorSince(t); ce != nil {
			return ce
		}
		return fmt.Errorf("controller did not enter freedrive: %w", err)
	}
	a.setState(stateManual)
	if enabledFor > 0 {
		a.mu.Lock()
		a.manualTimer = time.AfterFunc(enabledFor, func() {
			ctx, cancel := context.WithTimeout(a.ctx, 5*time.Second)
			defer cancel()
			if err := a.SetManualMode(ctx, false, 0, nil); err != nil {
				a.logger.Warnw("failed to leave manual mode after timer", "error", err)
			}
		})
		a.mu.Unlock()
	}
	return nil
}

// Close stops any motion, optionally powers the arm down, and closes both
// sockets. It is idempotent and never powers the arm down unless configured.
func (a *rb10) Close(ctx context.Context) error {
	if !a.closed.CompareAndSwap(false, true) {
		return nil
	}
	a.setState(stateClosed)
	a.mu.Lock()
	if a.manualTimer != nil {
		a.manualTimer.Stop()
	}
	a.mu.Unlock()

	a.opMgr.CancelRunning(ctx)
	a.cancel()

	stopCtx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	if a.streaming.Load() {
		a.sendStreamStop()
	}
	if snap, ok := a.client.Snapshot(); ok && snap.State.Moving() {
		if err := a.client.Send(stopCtx, rb.TaskStop()); err != nil {
			a.logger.Warnw("task stop during close failed", "error", err)
		}
	}
	if a.cfg.PowerOffOnClose {
		if err := a.client.Send(stopCtx, rb.Shutdown()); err != nil {
			a.logger.Warnw("power off during close failed", "error", err)
		}
	}
	err := a.client.Close()

	done := make(chan struct{})
	go func() { a.wg.Wait(); close(done) }()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		a.logger.Warnw("timed out waiting for background goroutines")
	}
	return err
}
