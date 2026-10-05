package arm

import (
	"context"
	"errors"
	"fmt"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"

	"github.com/viam-labs/rainbow-robotics/rb"
)

const minServoT1 = 0.002 // controller minimum for move_servo_j t1 (s)

// servoParams returns the move_servo_j parameters from config.
func (a *rb10) servoParams() (t1Default, t2, gain, alpha float64) {
	return 1 / float64(a.cfg.MoveHz), *a.cfg.ServoHoldSecs, *a.cfg.ServoGain, *a.cfg.ServoAlpha
}

// sendStreamStop ends a servo stream with a zero velocity setpoint.
func (a *rb10) sendStreamStop() {
	_, t2, gain, alpha := a.servoParams()
	if err := a.client.SendNoAck(rb.MoveSpeedJ([6]float64{}, minServoT1, t2, gain, alpha)); err != nil {
		a.logger.Warnw("failed to send stream stop", "error", err)
	}
}

// MoveThroughJointPositionsStreamed is the real-time executor. Point i is sent
// at t0 + point[i-1].Time as move_servo_j with t1 = point[i].Time -
// point[i-1].Time, so the controller reaches each point at its scheduled time.
// Setpoints are written without waiting for acknowledgements; the barrier at
// the end resynchronises. Velocities and accelerations in Constraints are
// ignored: the controller tracks position setpoints. If the executor falls
// more than one t1+t2 behind schedule it stops the stream and returns an error
// rather than letting the controller finish the last setpoint on its own.
func (a *rb10) MoveThroughJointPositionsStreamed(
	ctx context.Context,
	batches <-chan []rdkarm.TrajectoryPoint,
	responses chan<- rdkarm.Response,
	extra map[string]any,
) error {
	snap, err := a.ensureControllable()
	if err != nil {
		return err
	}
	ctx, done := a.opMgr.New(ctx)
	defer done()

	a.streaming.Store(true)
	defer a.streaming.Store(false)

	t1Default, t2, gain, alpha := a.servoParams()
	current := snap.State.Joints()
	gen := a.client.Generation()

	var (
		t0       time.Time
		havePrev bool
		prevTime time.Duration
	)
	sent := 0
	fail := func(err error) error {
		a.sendStreamStop()
		return err
	}

	for {
		var batch []rdkarm.TrajectoryPoint
		var ok bool
		select {
		case batch, ok = <-batches:
		case <-ctx.Done():
			return fail(ctx.Err())
		}
		if !ok {
			break
		}
		for _, p := range batch {
			target, err := inputsToDegs(p.Positions)
			if err != nil {
				return fail(err)
			}
			if err := rdkarm.CheckDesiredJointPositions(ctx, a, p.Positions); err != nil {
				return fail(err)
			}
			if !havePrev {
				if d := maxAbsDiffDeg(current, target); d > *a.cfg.RejectMoveRequestThresholdDeg {
					return fmt.Errorf("first trajectory point is %.1f deg from the current position (limit %.1f); refusing stream",
						d, *a.cfg.RejectMoveRequestThresholdDeg)
				}
				t0 = time.Now()
			}
			// Point i is sent at the scheduled time of point i-1 and asked to be
			// reached t1 later. The first point (Time 0) is where we already are.
			sendAt := t0.Add(prevTime)
			t1 := t1Default
			if havePrev {
				t1 = (p.Time - prevTime).Seconds()
				if t1 <= 0 {
					return fail(fmt.Errorf("trajectory point times must be strictly increasing (got %v after %v)", p.Time, prevTime))
				}
			}
			t1 = max(t1, minServoT1)
			if !sleepUntil(ctx, sendAt) {
				return fail(ctx.Err())
			}
			if lag := time.Since(sendAt).Seconds(); lag > t1+t2 && havePrev {
				return fail(fmt.Errorf("streaming executor fell %.0f ms behind schedule (limit %.0f ms); stream stopped",
					lag*1000, (t1+t2)*1000))
			}
			if a.client.Generation() != gen {
				return errors.New("connection to arm lost during streamed move")
			}
			if s, ok := a.client.Snapshot(); ok {
				if reason := a.faultReason(&s.State); reason != "" {
					a.setState(stateFaulted)
					return fail(fmt.Errorf("%w: %s", errStopped, reason))
				}
			}
			if err := a.client.SendNoAck(rb.MoveServoJ(target, t1, t2, gain, alpha)); err != nil {
				return err
			}
			sent++
			havePrev = true
			prevTime = p.Time
		}
		select {
		case responses <- rdkarm.Response{}:
		case <-ctx.Done():
			return fail(ctx.Err())
		}
	}

	if sent == 0 {
		return nil
	}
	// Let the last setpoint play out, then end the stream and resynchronise
	// with a barrier command that waits for its own acknowledgement.
	sleepUntil(ctx, t0.Add(prevTime))
	a.sendStreamStop()
	if err := a.client.Send(ctx, rb.Print(`"stream_end"`, "", "")); err != nil {
		return fmt.Errorf("stream barrier: %w", err)
	}
	if _, err := a.client.WaitState(ctx, stopSettleTimeout, func(st *rb.SystemState) bool { return st.Idle() }); err != nil {
		return fmt.Errorf("controller did not go idle after stream: %w", err)
	}
	return nil
}

// sleepUntil waits until t or ctx ends; it returns false if ctx ended.
func sleepUntil(ctx context.Context, t time.Time) bool {
	d := time.Until(t)
	if d <= 0 {
		return ctx.Err() == nil
	}
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-timer.C:
		return true
	case <-ctx.Done():
		return false
	}
}
