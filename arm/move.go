package arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"

	"github.com/viam-labs/rainbow-robotics/rb"
)

const (
	moveStartTimeout  = time.Second
	stopSettleTimeout = 2 * time.Second
	dedupToleranceRad = 1e-3
)

// errStopped is returned when the controller went idle without reaching the target.
var errStopped = errors.New("stopped before reaching target")

// speedsFromExtra lets a caller override the session speed for one move.
func (a *rb10) speedsFromExtra(extra map[string]any) (speed, accel float64) {
	speed, accel = a.sessionSpeeds()
	if v, ok := numberFromExtra(extra, "speed_degs_per_sec"); ok {
		speed = a.clampSpeed(v)
	}
	if v, ok := numberFromExtra(extra, "acceleration_degs_per_sec_per_sec"); ok {
		accel = a.clampAccel(v)
	}
	return speed, accel
}

// speedsFromOptions clamps MoveOptions (radians) to the config ceilings and
// the session values; per-joint arrays take the minimum since the controller
// accepts one speed for the whole batch.
func (a *rb10) speedsFromOptions(opts *rdkarm.MoveOptions, extra map[string]any) (speed, accel float64) {
	speed, accel = a.speedsFromExtra(extra)
	if opts == nil {
		return speed, accel
	}
	if opts.MaxVelRads > 0 {
		speed = min(speed, a.clampSpeed(utils.RadToDeg(opts.MaxVelRads)))
	}
	for _, v := range opts.MaxVelRadsJoints {
		if v > 0 {
			speed = min(speed, a.clampSpeed(utils.RadToDeg(v)))
		}
	}
	if opts.MaxAccRads > 0 {
		accel = min(accel, a.clampAccel(utils.RadToDeg(opts.MaxAccRads)))
	}
	for _, v := range opts.MaxAccRadsJoints {
		if v > 0 {
			accel = min(accel, a.clampAccel(utils.RadToDeg(v)))
		}
	}
	return speed, accel
}

func numberFromExtra(extra map[string]any, key string) (float64, bool) {
	if extra == nil {
		return 0, false
	}
	v, ok := extra[key]
	if !ok {
		return 0, false
	}
	switch n := v.(type) {
	case float64:
		return n, true
	case float32:
		return float64(n), true
	case int:
		return float64(n), true
	case int64:
		return float64(n), true
	}
	return 0, false
}

// MoveToJointPositions sends one move_j and waits for the controller to reach it.
func (a *rb10) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]any) error {
	if err := rdkarm.CheckDesiredJointPositions(ctx, a, positions); err != nil {
		return err
	}
	target, err := inputsToDegs(positions)
	if err != nil {
		return err
	}
	if _, err := a.ensureControllable(); err != nil {
		return err
	}
	ctx, done := a.opMgr.New(ctx)
	defer done()

	speed, accel := a.speedsFromExtra(extra)
	a.logger.Debugw("move_j", "target_deg", target, "speed", speed, "accel", accel)
	if err := a.client.Send(ctx, rb.MoveJ(target, speed, accel)); err != nil {
		return err
	}
	return a.waitForMove(ctx, target)
}

// MoveThroughJointPositions runs the waypoints as one controller-blended
// move_jb2 batch (chunked if joint_batch_chunk_size is set).
func (a *rb10) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *rdkarm.MoveOptions, extra map[string]any) error {
	if len(positions) == 0 {
		return nil
	}
	waypoints := make([][6]float64, 0, len(positions))
	var prev []referenceframe.Input
	for i, wp := range positions {
		if err := rdkarm.CheckDesiredJointPositions(ctx, a, wp); err != nil {
			return fmt.Errorf("waypoint %d: %w", i, err)
		}
		deg, err := inputsToDegs(wp)
		if err != nil {
			return fmt.Errorf("waypoint %d: %w", i, err)
		}
		if prev != nil && linfRad(prev, wp) < dedupToleranceRad {
			continue
		}
		prev = wp
		waypoints = append(waypoints, deg)
	}
	snap, err := a.ensureControllable()
	if err != nil {
		return err
	}
	if d := maxAbsDiffDeg(snap.State.Joints(), waypoints[0]); d > *a.cfg.RejectMoveRequestThresholdDeg {
		return fmt.Errorf("first waypoint is %.1f deg from the current position (limit %.1f); refusing move",
			d, *a.cfg.RejectMoveRequestThresholdDeg)
	}

	ctx, done := a.opMgr.New(ctx)
	defer done()

	speed, accel := a.speedsFromOptions(options, extra)
	chunk := a.cfg.JointBatchChunkSize
	if chunk <= 0 {
		chunk = len(waypoints)
	}
	for start := 0; start < len(waypoints); start += chunk {
		end := min(start+chunk, len(waypoints))
		batch := waypoints[start:end]
		if len(batch) == 1 {
			if err := a.client.Send(ctx, rb.MoveJ(batch[0], speed, accel)); err != nil {
				return err
			}
		} else {
			if err := a.client.Send(ctx, rb.MoveJB2Clear()); err != nil {
				return err
			}
			for _, wp := range batch {
				if err := a.client.Send(ctx, rb.MoveJB2Add(wp, speed, accel, *a.cfg.JointBatchBlend)); err != nil {
					return err
				}
			}
			if err := a.client.Send(ctx, rb.MoveJB2Run()); err != nil {
				return err
			}
		}
		if err := a.waitForMove(ctx, batch[len(batch)-1]); err != nil {
			return err
		}
	}
	return nil
}

func linfRad(a, b []referenceframe.Input) float64 {
	var m float64
	for i := range a {
		if i < len(b) {
			m = math.Max(m, math.Abs(a[i]-b[i]))
		}
	}
	return m
}

// waitForMove waits for a joint move: see waitForArrival.
func (a *rb10) waitForMove(ctx context.Context, target [6]float64) error {
	return a.waitForArrival(ctx,
		func(st *rb.SystemState) bool { return maxAbsDiffDeg(st.Joints(), target) <= arrivalToleranceDeg },
		func(st *rb.SystemState) string {
			return fmt.Sprintf("controller idle at %v, target %v", roundJoints(st.Joints()), roundJoints(target))
		})
}

// waitForTCPMove waits for a controller-side Cartesian move (move_l /
// move_jl): arrival is the TCP reference within 1 mm and 0.5 deg of target.
func (a *rb10) waitForTCPMove(ctx context.Context, target [6]float64) error {
	return a.waitForArrival(ctx,
		func(st *rb.SystemState) bool { return tcpWithin(st.TCPRef, target, 1.0, 0.5) },
		func(st *rb.SystemState) string {
			return fmt.Sprintf("controller idle with tcp_ref %v, target %v", roundJoints(f32to64(st.TCPRef)), roundJoints(target))
		})
}

func f32to64(v [6]float32) [6]float64 {
	var out [6]float64
	for i := range v {
		out[i] = float64(v[i])
	}
	return out
}

// tcpWithin compares a TCP tuple (x,y,z mm; rx,ry,rz deg) against target,
// wrapping the angle differences into [-180, 180).
func tcpWithin(have [6]float32, want [6]float64, posTolMM, angTolDeg float64) bool {
	for i := range 3 {
		if math.Abs(float64(have[i])-want[i]) > posTolMM {
			return false
		}
	}
	for i := 3; i < 6; i++ {
		d := math.Mod(float64(have[i])-want[i]+540, 360) - 180
		if math.Abs(d) > angTolDeg {
			return false
		}
	}
	return true
}

// waitForArrival waits for the controller to start moving (up to 1 s; a move
// that is already at its target may never report motion), then to go idle, and
// finally checks arrived(). It watches the fault bits on every snapshot and
// sends "task stop" if ctx is cancelled.
func (a *rb10) waitForArrival(ctx context.Context, arrived func(*rb.SystemState) bool, describe func(*rb.SystemState) string) error {
	gen := a.client.Generation()
	startDeadline := time.Now().Add(moveStartTimeout)
	var last time.Time
	sawMoving := false
	for {
		snap, err := a.client.NextSnapshot(ctx, last, snapshotMaxAge)
		if err != nil {
			if ctx.Err() != nil {
				a.emergencyStop("move cancelled")
				return a.cancelErr(ctx)
			}
			a.emergencyStop("state stream stalled")
			return fmt.Errorf("lost control box state during move: %w", err)
		}
		last = snap.At
		st := &snap.State
		if a.client.Generation() != gen {
			return errors.New("connection to arm lost during move")
		}
		if reason := a.faultReason(st); reason != "" {
			a.setState(stateFaulted)
			return fmt.Errorf("%w: %s", errStopped, reason)
		}
		if st.Moving() {
			sawMoving = true
			continue
		}
		if arrived(st) {
			return nil
		}
		if sawMoving || time.Now().After(startDeadline) {
			return fmt.Errorf("%w: %s", errStopped, describe(st))
		}
	}
}

// maxLinearSpeedMMs caps controller-side Cartesian moves. 250 mm/s is the
// ISO/TS 15066 hand-guiding speed limit and a sane ceiling for a driver.
const maxLinearSpeedMMs = 250.0

// moveTCP sends one move_l or move_jl to the controller's own IK and waits
// for arrival. pose is the controller's TCP tuple (x,y,z mm; rx,ry,rz deg in
// its own Euler convention), i.e. the same numbers get_state reports as
// tcp_ref, so no frame conversion happens here.
func (a *rb10) moveTCP(ctx context.Context, linear bool, pose [6]float64, speed, accel float64) error {
	if _, err := a.ensureControllable(); err != nil {
		return err
	}
	ctx, done := a.opMgr.New(ctx)
	defer done()

	var script string
	if linear {
		a.logger.Debugw("move_l", "target", pose, "speed_mm_s", speed, "accel_mm_s2", accel)
		script = rb.MoveL(pose, speed, accel)
	} else {
		a.logger.Debugw("move_jl", "target", pose, "speed_deg_s", speed, "accel_deg_s2", accel)
		script = rb.MoveJL(pose, speed, accel)
	}
	if err := a.client.Send(ctx, script); err != nil {
		return err
	}
	return a.waitForTCPMove(ctx, pose)
}

func roundJoints(j [6]float64) [6]float64 {
	for i := range j {
		j[i] = math.Round(j[i]*100) / 100
	}
	return j
}

// cancelErr explains why a move's context ended before the target was reached.
// The context error stays wrapped so errors.Is(err, context.Canceled) holds.
func (a *rb10) cancelErr(ctx context.Context) error {
	err := ctx.Err()
	switch {
	case a.stopRequested.Swap(false):
		return fmt.Errorf("move interrupted by Stop: %w", err)
	case errors.Is(err, context.DeadlineExceeded):
		return fmt.Errorf("move timed out before reaching target: %w", err)
	default:
		return fmt.Errorf("move cancelled by caller before reaching target: %w", err)
	}
}

// emergencyStop halts the controller on a fresh context because the move's
// context is already cancelled.
func (a *rb10) emergencyStop(why string) {
	ctx, cancel := context.WithTimeout(context.Background(), stopSettleTimeout)
	defer cancel()
	if a.streaming.Load() {
		a.sendStreamStop()
	}
	if err := a.client.Send(ctx, rb.TaskStop()); err != nil {
		a.logger.Warnw("task stop failed", "reason", why, "error", err)
		return
	}
	a.logger.Infow("sent task stop", "reason", why)
}

// Stop cancels the running operation, halts the controller and waits (bounded)
// for it to report idle. It returns nil if the arm was already idle.
func (a *rb10) Stop(ctx context.Context, extra map[string]any) error {
	if a.opMgr.OpRunning() {
		a.stopRequested.Store(true)
	}
	a.opMgr.CancelRunning(ctx)
	if a.closed.Load() {
		return nil
	}
	if a.streaming.Load() {
		a.sendStreamStop()
	}
	if err := a.client.Send(ctx, rb.TaskStop()); err != nil {
		return err
	}
	if _, err := a.client.WaitState(ctx, stopSettleTimeout, func(st *rb.SystemState) bool { return st.Idle() }); err != nil {
		return fmt.Errorf("controller still moving after stop: %w", err)
	}
	return nil
}

// errNoMotionService is returned by MoveToPosition when the motion service did
// not resolve. Planning is deliberately not done in-process: that would pull
// the nlopt/cgo solver into this binary, which then fails to start on machines
// without libnlopt. viam-server's builtin motion service ships with it.
var errNoMotionService = errors.New("MoveToPosition needs the motion service: give this arm a \"frame\" " +
	"in the machine config (at minimum a parent such as \"world\") so the motion service can address it")

// MoveToPosition hands the Cartesian goal to the motion service, which plans
// against this arm's Kinematics and Geometries and executes through
// MoveThroughJointPositions.
func (a *rb10) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]any) error {
	if _, err := a.ensureControllable(); err != nil {
		return err
	}
	if a.motion == nil {
		return errNoMotionService
	}
	_, err := a.motion.Move(ctx, motion.MoveReq{
		ComponentName: a.Name().ShortName(),
		Destination:   referenceframe.NewPoseInFrame(a.Name().ShortName()+"_origin", pose),
		Extra:         extra,
	})
	return err
}

func (a *rb10) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return a.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}
