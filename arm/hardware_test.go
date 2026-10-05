package arm

import (
	"context"
	"math"
	"math/rand"
	"os"
	"testing"
	"time"

	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/rainbow-robotics/rb"
)

// Hardware tests talk to a real control box and are skipped unless RB_HOST is
// set, e.g. RB_HOST=10.0.2.7 go test -run Hardware -v ./arm/. They only ever
// put the controller in simulation mode and never move the physical arm.

func hardwareArm(t *testing.T) *rb10 {
	t.Helper()
	host := os.Getenv("RB_HOST")
	if host == "" {
		t.Skip("RB_HOST not set; skipping hardware test")
	}
	cfg := Config{Host: host, OperationMode: ModeSimulation, SpeedDegsPerSec: 30}
	ctx, cancel := context.WithTimeout(context.Background(), 90*time.Second)
	defer cancel()
	a, err := NewRB10(ctx, nil, rdkarm.Named("rb10"), cfg, false, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = a.Close(context.Background()) })
	return a.(*rb10)
}

// TestHardwareControlBoxInfo records the controller software version.
func TestHardwareControlBoxInfo(t *testing.T) {
	a := hardwareArm(t)
	ctx := context.Background()
	w := a.client.NewWaiter(func(l rb.Line) bool { return l.Type == rb.LineInfo && l.Category == "controlbox" })
	defer w.Close()
	if err := a.client.Send(ctx, rb.ControlBoxInfo()); err != nil {
		t.Fatal(err)
	}
	l, err := w.Wait(ctx, 2*time.Second)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("control box: %s", l.Msg)
	t.Logf("state: %s", a.describeState())
}

// TestHardwareFKCrossCheck is the single most valuable hardware test: it
// compares the SVA model against the controller's own forward kinematics for
// 50 random joint vectors without moving the arm. Set the TCP offset on the
// controller to zero first. Pass is position within 1 mm and orientation
// within 0.5 degrees.
func TestHardwareFKCrossCheck(t *testing.T) {
	a := hardwareArm(t)
	ctx := context.Background()
	rng := rand.New(rand.NewSource(1))
	limits := a.model.DoF()
	var worstPos, worstAng float64
	for i := 0; i < 50; i++ {
		var j [6]float64
		for k := range j {
			lo, hi := limits[k].Min*180/math.Pi, limits[k].Max*180/math.Pi
			// Keep away from the multi-turn extremes; the controller's FK is periodic anyway.
			lo, hi = math.Max(lo, -179), math.Min(hi, 179)
			j[k] = lo + rng.Float64()*(hi-lo)
		}
		v, err := a.client.Query(ctx, rb.CalcFKTCP(j))
		if err != nil {
			t.Fatal(err)
		}
		vals, err := rb.ParseFloats(v)
		if err != nil || len(vals) != 6 {
			t.Fatalf("calc_fk_tcp(%v) = %q: %v", j, v, err)
		}
		var p [6]float64
		copy(p[:], vals)
		boxPose := RainbowPoseToSpatial(p)
		modelPose, err := a.model.Transform(degsToInputs(j))
		if err != nil {
			t.Fatal(err)
		}
		dPos := boxPose.Point().Distance(modelPose.Point())
		dAng := spatialmath.OrientationBetween(boxPose.Orientation(), modelPose.Orientation()).AxisAngles().Theta * 180 / math.Pi
		worstPos = math.Max(worstPos, dPos)
		worstAng = math.Max(worstAng, dAng)
		if dPos > 1 || dAng > 0.5 {
			t.Errorf("joints %v: position off by %.2f mm, orientation by %.2f deg (box %v, model %v)",
				j, dPos, dAng, boxPose.Point(), modelPose.Point())
		}
	}
	t.Logf("worst position error %.3f mm, worst orientation error %.3f deg", worstPos, worstAng)
}

// TestHardwareSimulationMoves runs the two reference poses in the controller's
// simulation mode: only the commanded reference moves, never the arm.
func TestHardwareSimulationMoves(t *testing.T) {
	a := hardwareArm(t)
	ctx := context.Background()
	for _, target := range [][]referenceframe.Input{degsToInputs([6]float64{0, 0, 0, 0, 0, 0}), degsToInputs([6]float64{0, -30, 90, 0, 60, 0})} {
		start := time.Now()
		if err := a.MoveToJointPositions(ctx, target, nil); err != nil {
			t.Fatal(err)
		}
		t.Logf("move to %v took %v", target, time.Since(start))
	}
	// Stop mid-move.
	errCh := make(chan error, 1)
	go func() { errCh <- a.MoveToJointPositions(ctx, degsToInputs([6]float64{0, 0, 0, 0, 0, 0}), nil) }()
	time.Sleep(300 * time.Millisecond)
	if err := a.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}
	t.Logf("move after stop returned: %v", <-errCh)
}
