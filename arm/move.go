package arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan/armplanning"
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

// waitForMove waits for the controller to start moving (up to 1 s; a move
// that is already at its target may never report motion), then to go idle, and
// finally checks the joints are within tolerance of target. It watches the
// fault bits on every snapshot and sends "task stop" if ctx is cancelled.
func (a *rb10) waitForMove(ctx context.Context, target [6]float64) error {
	gen := a.client.Generation()
	startDeadline := time.Now().Add(moveStartTimeout)
	var last time.Time
	sawMoving := false
	for {
		snap, err := a.client.NextSnapshot(ctx, last, snapshotMaxAge)
		if err != nil {
			if ctx.Err() != nil {
				a.emergencyStop("move cancelled")
				return ctx.Err()
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
		atTarget := maxAbsDiffDeg(st.Joints(), target) <= arrivalToleranceDeg
		if atTarget {
			return nil
		}
		if sawMoving || time.Now().After(startDeadline) {
			return fmt.Errorf("%w: controller idle at %v, target %v", errStopped, roundJoints(st.Joints()), roundJoints(target))
		}
	}
}

func roundJoints(j [6]float64) [6]float64 {
	for i := range j {
		j[i] = math.Round(j[i]*100) / 100
	}
	return j
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

// MoveToPosition hands the Cartesian goal to the motion service when it
// resolved, else plans in-process; both execute through MoveThroughJointPositions.
func (a *rb10) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]any) error {
	if _, err := a.ensureControllable(); err != nil {
		return err
	}
	if a.motion != nil {
		_, err := a.motion.Move(ctx, motion.MoveReq{
			ComponentName: a.Name().ShortName(),
			Destination:   referenceframe.NewPoseInFrame(a.Name().ShortName()+"_origin", pose),
			Extra:         extra,
		})
		return err
	}
	return armplanning.MoveArm(ctx, a.logger, a, pose)
}

func (a *rb10) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return a.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}
