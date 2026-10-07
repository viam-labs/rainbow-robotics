package arm

import (
	"context"
	"errors"
	"math"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"github.com/viam-labs/rainbow-robotics/rb"
	"github.com/viam-labs/rainbow-robotics/rb/fakebox"
)

func newTestArm(t *testing.T, boxOpts fakebox.Options, mutate func(*Config)) (*fakebox.Box, *rb10) {
	t.Helper()
	box, err := fakebox.Start(boxOpts)
	if err != nil {
		t.Fatal(err)
	}
	cfg := Config{
		Host:                box.Host(),
		CommandPort:         box.CommandPort(),
		DataPort:            box.DataPort(),
		OperationMode:       ModeSimulation,
		SpeedDegsPerSec:     1000, // the fake box moves at the commanded speed; keep tests fast
		MaxSpeedDegsPerSec:  2000,
		StatePollHz:         200,
		ActivateTimeoutSecs: 5,
		CommandTimeoutSecs:  1,
	}
	if mutate != nil {
		mutate(&cfg)
	}
	if _, _, err := cfg.Validate("test"); err != nil {
		box.Close()
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	a, err := NewRB10(ctx, nil, rdkarm.Named("rb10"), cfg, false, logging.NewTestLogger(t))
	if err != nil {
		box.Close()
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = a.Close(context.Background()); box.Close() })
	return box, a.(*rb10)
}

func deg(vals ...float64) []referenceframe.Input {
	out := make([]referenceframe.Input, len(vals))
	for i, v := range vals {
		out[i] = v * math.Pi / 180
	}
	return out
}

func TestValidate(t *testing.T) {
	var c Config
	if _, _, err := c.Validate("p"); err == nil {
		t.Error("expected host required error")
	}
	c = Config{Host: "10.0.2.7"}
	_, opt, err := c.Validate("p")
	if err != nil {
		t.Fatal(err)
	}
	if len(opt) != 1 || !strings.Contains(opt[0], "builtin") {
		t.Errorf("optional deps = %v", opt)
	}
	bad := []Config{
		{Host: "h", SpeedDegsPerSec: 500},
		{Host: "h", OperationMode: "sim"},
		{Host: "h", StatePollHz: 1},
		{Host: "h", SpeedBar: ptr(2.0)},
		{Host: "h", CommandPort: 70000},
		{Host: "h", PayloadCoGMM: []float64{1, 2}},
		{Host: "h", PayloadKg: ptr(1.0), PayloadCoGMM: []float64{1, 2}},
		{Host: "h", ServoHoldSecs: ptr(1.0)},
	}
	for i, b := range bad {
		if _, _, err := b.Validate("p"); err == nil {
			t.Errorf("case %d: expected validation error for %+v", i, b)
		}
	}
	d := c.withDefaults()
	if d.SpeedDegsPerSec != 60 || d.OperationMode != ModeReal || d.MoveHz != 100 || !*d.ActivateOnStart {
		t.Errorf("defaults wrong: %+v", d)
	}
}

func TestConstructorActivatesAndSetsMode(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: false}, nil)
	st := box.State()
	if !st.Activated() || !st.Simulation() {
		t.Errorf("box not activated/in simulation after startup: %+v", st)
	}
	cmds := box.Commands()
	joined := strings.Join(cmds, "\n")
	if !strings.Contains(joined, "mc jall init") || !strings.Contains(joined, "pgmode simulation") {
		t.Errorf("startup commands = %v", cmds)
	}
	desc, err := a.DoCommand(context.Background(), map[string]any{"get_state_description": nil})
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(desc["description"].(string), "simulation / activated / idle") {
		t.Errorf("description = %v", desc)
	}
}

func TestConstructorRefusesRealModeWhenNotActivated(t *testing.T) {
	box, err := fakebox.Start(fakebox.Options{Activated: false})
	if err != nil {
		t.Fatal(err)
	}
	defer box.Close()
	cfg := Config{Host: box.Host(), CommandPort: box.CommandPort(), DataPort: box.DataPort(),
		OperationMode: ModeReal, ActivateOnStart: ptr(false), CommandTimeoutSecs: 1}
	_, err = NewRB10(context.Background(), nil, rdkarm.Named("rb10"), cfg, false, logging.NewTestLogger(t))
	if err == nil || !strings.Contains(err.Error(), "not activated") {
		t.Fatalf("expected not-activated error, got %v", err)
	}
}

func TestConstructorSendsSettings(t *testing.T) {
	box, _ := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) {
		c.SpeedBar = ptr(0.3)
		c.PayloadKg = ptr(2.5)
		c.PayloadCoGMM = []float64{0, 0, 50}
		c.CollisionDetection = ptr(true)
		c.CollisionSensitivity = ptr(0.8)
	})
	joined := strings.Join(box.Commands(), "\n")
	for _, want := range []string{"set_speed_bar(0.3)", "set_payload_info(2.5,0,0,50)", "set_collision_onoff(1)", "set_collision_th(0.8)"} {
		if !strings.Contains(joined, want) {
			t.Errorf("missing %q in %q", want, joined)
		}
	}
}

func TestJointPositionsAndEndPosition(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	ctx := context.Background()
	box.SetJoints([6]float64{10, -20, 30, -40, 50, -60})
	deadline := time.Now().Add(time.Second)
	for time.Now().Before(deadline) {
		jp, err := a.JointPositions(ctx, nil)
		if err == nil && math.Abs(jp[0]-10*math.Pi/180) < 1e-6 {
			break
		}
		time.Sleep(5 * time.Millisecond)
	}
	jp, err := a.JointPositions(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	want := deg(10, -20, 30, -40, 50, -60)
	for i := range want {
		if math.Abs(jp[i]-want[i]) > 1e-5 {
			t.Errorf("joint %d = %v want %v", i, jp[i], want[i])
		}
	}
	pose, err := a.EndPosition(ctx, nil)
	if err != nil {
		t.Fatal(err)
	}
	model, _ := a.Kinematics(ctx)
	expect, _ := model.Transform(jp)
	if pose.Point().Distance(expect.Point()) > 1e-6 {
		t.Errorf("EndPosition %v != model transform %v", pose.Point(), expect.Point())
	}
}

func TestZeroPoseIsStraightUp(t *testing.T) {
	model, err := buildSVAModel("rb10", 60, 200)
	if err != nil {
		t.Fatal(err)
	}
	pose, err := model.Transform(deg(0, 0, 0, 0, 0, 0))
	if err != nil {
		t.Fatal(err)
	}
	// d1 + a1 + a2 along z; y = -187.5 + 148.4 - 117.15 - 115.3; x = 0.
	p := pose.Point()
	if math.Abs(p.X) > 1e-6 || math.Abs(p.Y-(-187.5+148.4-117.15-115.3)) > 1e-6 || math.Abs(p.Z-(197+612.7+570.15+117.15)) > 1e-6 {
		t.Errorf("zero pose TCP = %v", p)
	}
	// Tool frame convention: Z points out of the flange (along -Y at zero pose),
	// X stays along the base X. Grippers attach along Z and "point down" means
	// the flange faces down.
	rm := pose.Orientation().RotationMatrix()
	if z := rm.Mul(r3.Vector{Z: 1}); z.Sub(r3.Vector{Y: -1}).Norm() > 1e-9 {
		t.Errorf("tool Z at zero pose = %v, want (0,-1,0)", z)
	}
	if x := rm.Mul(r3.Vector{X: 1}); x.Sub(r3.Vector{X: 1}).Norm() > 1e-9 {
		t.Errorf("tool X at zero pose = %v, want (1,0,0)", x)
	}
	limits := model.DoF()
	if len(limits) != 6 {
		t.Fatalf("DoF = %d", len(limits))
	}
	if math.Abs(limits[2].Max-165*math.Pi/180) > 1e-9 || math.Abs(limits[0].Max-2*math.Pi) > 1e-9 {
		t.Errorf("limits = %+v", limits)
	}
	for i, l := range limits {
		if l.MaxVelocity == nil || l.MaxAcceleration == nil {
			t.Fatalf("joint %d missing velocity/acceleration limit", i)
		}
		if math.Abs(*l.MaxVelocity-60*math.Pi/180) > 1e-9 {
			t.Errorf("joint %d max_velocity = %v", i, *l.MaxVelocity)
		}
	}
	// The re-marshalled kinematics file carries the limits for GetKinematics clients.
	raw := model.ModelConfig().OriginalFile.Bytes
	if !strings.Contains(string(raw), `"max_velocity":60`) {
		t.Errorf("max_velocity not in kinematics bytes: %s", raw)
	}
}

func TestGeometriesNoSelfCollisionAtReferencePoses(t *testing.T) {
	model, err := buildSVAModel("rb10", 60, 200)
	if err != nil {
		t.Fatal(err)
	}
	for _, pose := range [][]referenceframe.Input{deg(0, 0, 0, 0, 0, 0), deg(0, -30, 90, 0, 60, 0), deg(45, -60, 120, -60, 90, 0)} {
		gif, err := model.Geometries(pose)
		if err != nil {
			t.Fatal(err)
		}
		geoms := gif.Geometries()
		if len(geoms) != 7 {
			t.Fatalf("expected 7 geometries, got %d", len(geoms))
		}
		// Non-adjacent links must not intersect.
		for i := 0; i < len(geoms); i++ {
			for j := i + 2; j < len(geoms); j++ {
				hit, _, err := geoms[i].CollidesWith(geoms[j], 0)
				if err != nil {
					t.Fatal(err)
				}
				if hit {
					t.Errorf("pose %v: %s collides with %s", pose, geoms[i].Label(), geoms[j].Label())
				}
			}
		}
	}
}

func TestMoveToJointPositions(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	ctx := context.Background()
	target := deg(10, 20, 30, 0, 0, 0)
	if err := a.MoveToJointPositions(ctx, target, nil); err != nil {
		t.Fatal(err)
	}
	if j := box.Joints(); j != [6]float64{10, 20, 30, 0, 0, 0} {
		t.Errorf("box joints = %v", j)
	}
	joined := strings.Join(box.Commands(), "\n")
	if !strings.Contains(joined, "move_j(jnt[10, 20, 30, 0, 0, 0],1000,200)") {
		t.Errorf("move_j not sent as expected: %s", joined)
	}
	// Moving to where we already are succeeds without the controller ever reporting motion.
	if err := a.MoveToJointPositions(ctx, target, nil); err != nil {
		t.Fatalf("no-op move: %v", err)
	}
	// extra speed override, clamped to the max.
	if err := a.MoveToJointPositions(ctx, deg(0, 0, 0, 0, 0, 0), map[string]any{"speed_degs_per_sec": 5000.0}); err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "move_j(jnt[0, 0, 0, 0, 0, 0],2000,200)") {
		t.Errorf("speed not clamped: %v", box.Commands())
	}
	// Out-of-range joint is refused before anything is sent.
	box.ClearCommands()
	if err := a.MoveToJointPositions(ctx, deg(0, 0, 170, 0, 0, 0), nil); err == nil {
		t.Error("expected joint limit error")
	}
	if len(box.Commands()) != 0 {
		t.Errorf("commands sent despite limit error: %v", box.Commands())
	}
}

func TestMoveThroughJointPositionsBatch(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	ctx := context.Background()
	wps := [][]referenceframe.Input{deg(0, 0, 0, 0, 0, 0), deg(5, 0, 0, 0, 0, 0), deg(5, 0, 0, 0, 0, 0), deg(5, 5, 0, 0, 0, 0), deg(5, 5, 5, 0, 0, 0)}
	opts := &rdkarm.MoveOptions{MaxVelRads: 10 * math.Pi / 180}
	if err := a.MoveThroughJointPositions(ctx, wps, opts, nil); err != nil {
		t.Fatal(err)
	}
	cmds := box.Commands()
	var adds int
	for _, c := range cmds {
		if strings.HasPrefix(c, "move_jb2_add(") {
			adds++
			if !strings.Contains(c, ",10,200,0,0.5)") {
				t.Errorf("unexpected jb2 add args: %s", c)
			}
		}
	}
	if adds != 4 { // duplicate consecutive waypoint deduped
		t.Errorf("move_jb2_add count = %d: %v", adds, cmds)
	}
	if j := box.Joints(); j != [6]float64{5, 5, 5, 0, 0, 0} {
		t.Errorf("box joints = %v", j)
	}
	// A first waypoint far from the current position is refused.
	err := a.MoveThroughJointPositions(ctx, [][]referenceframe.Input{deg(90, 0, 0, 0, 0, 0), deg(91, 0, 0, 0, 0, 0)}, nil, nil)
	if err == nil || !strings.Contains(err.Error(), "refusing move") {
		t.Errorf("expected reject-threshold error, got %v", err)
	}
}

func TestMoveThroughJointPositionsChunked(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.JointBatchChunkSize = 2 })
	ctx := context.Background()
	wps := [][]referenceframe.Input{deg(1, 0, 0, 0, 0, 0), deg(2, 0, 0, 0, 0, 0), deg(3, 0, 0, 0, 0, 0)}
	if err := a.MoveThroughJointPositions(ctx, wps, nil, nil); err != nil {
		t.Fatal(err)
	}
	var runs, movej int
	for _, c := range box.Commands() {
		if c == "move_jb2_run()" {
			runs++
		}
		if strings.HasPrefix(c, "move_j(") {
			movej++
		}
	}
	if runs != 1 || movej != 1 {
		t.Errorf("runs=%d move_j=%d cmds=%v", runs, movej, box.Commands())
	}
}

func TestStopMidMove(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.SpeedDegsPerSec = 20 })
	ctx := context.Background()
	errCh := make(chan error, 1)
	go func() { errCh <- a.MoveToJointPositions(ctx, deg(90, 0, 0, 0, 0, 0), nil) }()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) && !box.Moving() {
		time.Sleep(5 * time.Millisecond)
	}
	if moving, _ := a.IsMoving(ctx); !moving {
		t.Fatal("expected IsMoving true")
	}
	if err := a.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}
	err := <-errCh
	if err == nil {
		t.Fatal("expected the move to fail after Stop")
	}
	if !errors.Is(err, context.Canceled) || !strings.Contains(err.Error(), "interrupted by Stop") {
		t.Errorf("move error = %v; want context.Canceled wrapped as 'interrupted by Stop'", err)
	}
	if j := box.Joints(); j[0] >= 90 {
		t.Errorf("arm reached target despite stop: %v", j)
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "task stop") {
		t.Errorf("task stop not sent: %v", box.Commands())
	}
	if moving, _ := a.IsMoving(ctx); moving {
		t.Error("still moving after stop")
	}
	// Stop when idle is a no-op success.
	if err := a.Stop(ctx, nil); err != nil {
		t.Errorf("stop while idle: %v", err)
	}
}

func TestContextCancelSendsTaskStop(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.SpeedDegsPerSec = 20 })
	ctx, cancel := context.WithCancel(context.Background())
	errCh := make(chan error, 1)
	go func() { errCh <- a.MoveToJointPositions(ctx, deg(90, 0, 0, 0, 0, 0), nil) }()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) && !box.Moving() {
		time.Sleep(5 * time.Millisecond)
	}
	cancel()
	if err := <-errCh; !errors.Is(err, context.Canceled) {
		t.Fatalf("expected context.Canceled, got %v", err)
	}
	deadline = time.Now().Add(time.Second)
	for time.Now().Before(deadline) && box.Moving() {
		time.Sleep(5 * time.Millisecond)
	}
	if box.Moving() {
		t.Error("box still moving after cancel")
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "task stop") {
		t.Errorf("task stop not sent: %v", box.Commands())
	}
}

func TestFaultMidMove(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.SpeedDegsPerSec = 20 })
	ctx := context.Background()
	errCh := make(chan error, 1)
	go func() { errCh <- a.MoveToJointPositions(ctx, deg(90, 0, 0, 0, 0, 0), nil) }()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) && !box.Moving() {
		time.Sleep(5 * time.Millisecond)
	}
	box.SetCollision(true)
	err := <-errCh
	if err == nil || !errors.Is(err, errStopped) || !strings.Contains(err.Error(), "collision") {
		t.Fatalf("expected collision stop error, got %v", err)
	}
	// Moves are refused while the fault stands, and clear_error reports it.
	if err := a.MoveToJointPositions(ctx, deg(0, 0, 0, 0, 0, 0), nil); err == nil || !strings.Contains(err.Error(), "collision") {
		t.Errorf("expected refusal, got %v", err)
	}
	res, err := a.DoCommand(ctx, map[string]any{"clear_error": nil})
	if err != nil {
		t.Fatal(err)
	}
	if res["cleared"] != false {
		t.Errorf("clear_error should report the standing fault: %v", res)
	}
	box.SetCollision(false)
	res, err = a.DoCommand(ctx, map[string]any{"clear_error": nil})
	if err != nil {
		t.Fatal(err)
	}
	if res["cleared"] != true {
		t.Errorf("clear_error = %v", res)
	}
	if err := a.MoveToJointPositions(ctx, deg(0, 0, 0, 0, 0, 0), nil); err != nil {
		t.Errorf("move after clear: %v", err)
	}
}

func TestEStopRefusesMoves(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	box.SetEStop(true)
	time.Sleep(30 * time.Millisecond)
	err := a.MoveToJointPositions(context.Background(), deg(1, 0, 0, 0, 0, 0), nil)
	if err == nil || !strings.Contains(err.Error(), "e-stop") {
		t.Fatalf("expected e-stop refusal, got %v", err)
	}
}

func TestReconnectRerunsStartup(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.SpeedBar = ptr(0.4) })
	ctx := context.Background()
	box.ClearCommands()
	box.DropConnections()
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		if strings.Contains(strings.Join(box.Commands(), "\n"), "set_speed_bar(0.4)") {
			break
		}
		time.Sleep(10 * time.Millisecond)
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "set_speed_bar(0.4)") {
		t.Fatalf("startup not re-run after reconnect: %v", box.Commands())
	}
	deadline = time.Now().Add(2 * time.Second)
	var err error
	for time.Now().Before(deadline) {
		if err = a.MoveToJointPositions(ctx, deg(1, 0, 0, 0, 0, 0), nil); err == nil {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}
	if err != nil {
		t.Fatalf("move after reconnect: %v", err)
	}
}

func TestStreamedTiming(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.MoveHz = 100 })
	ctx := context.Background()
	batches := make(chan []rdkarm.TrajectoryPoint)
	responses := make(chan rdkarm.Response, 16)
	const n = 50
	go func() {
		defer close(batches)
		for b := 0; b < 2; b++ {
			var batch []rdkarm.TrajectoryPoint
			for i := 0; i < n/2; i++ {
				k := b*n/2 + i
				batch = append(batch, rdkarm.TrajectoryPoint{
					Time:      time.Duration(k) * 10 * time.Millisecond,
					Positions: deg(float64(k)*0.1, 0, 0, 0, 0, 0),
				})
			}
			batches <- batch
		}
	}()
	start := time.Now()
	err := a.MoveThroughJointPositionsStreamed(ctx, batches, responses, nil)
	elapsed := time.Since(start)
	if err != nil {
		t.Fatal(err)
	}
	if got := box.ServoSetpoints(); got != n {
		t.Errorf("servo setpoints = %d, want %d", got, n)
	}
	if len(responses) != 2 {
		t.Errorf("responses = %d, want 2", len(responses))
	}
	// 49 intervals of 10ms plus the stop/barrier; generous upper bound.
	if elapsed < 480*time.Millisecond || elapsed > 2*time.Second {
		t.Errorf("elapsed = %v", elapsed)
	}
	if j := box.Joints(); math.Abs(j[0]-4.9) > 1e-6 {
		t.Errorf("final joints = %v", j)
	}
	joined := strings.Join(box.Commands(), "\n")
	if !strings.Contains(joined, "move_servo_j(jnt[0.1, 0, 0, 0, 0, 0],0.01,0.03,0.5,0.5)") {
		t.Errorf("servo command args wrong:\n%s", joined)
	}
	if !strings.Contains(joined, "move_speed_j(jnt[0, 0, 0, 0, 0, 0]") {
		t.Error("stream stop not sent")
	}
	if box.Moving() {
		t.Error("box still moving after stream")
	}
}

func TestStreamedRejectsFarFirstPoint(t *testing.T) {
	_, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	batches := make(chan []rdkarm.TrajectoryPoint, 1)
	batches <- []rdkarm.TrajectoryPoint{{Time: 0, Positions: deg(45, 0, 0, 0, 0, 0)}}
	close(batches)
	err := a.MoveThroughJointPositionsStreamed(context.Background(), batches, make(chan rdkarm.Response, 1), nil)
	if err == nil || !strings.Contains(err.Error(), "refusing stream") {
		t.Fatalf("expected refusal, got %v", err)
	}
}

func TestSetManualMode(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true, Real: true}, func(c *Config) { c.OperationMode = ModeReal })
	ctx := context.Background()
	if err := a.SetManualMode(ctx, true, 200*time.Millisecond, nil); err != nil {
		t.Fatal(err)
	}
	if on, _ := a.ManualMode(ctx, nil); !on {
		t.Error("expected manual mode on")
	}
	if err := a.MoveToJointPositions(ctx, deg(1, 0, 0, 0, 0, 0), nil); err == nil || !strings.Contains(err.Error(), "freedrive") {
		t.Errorf("expected freedrive refusal, got %v", err)
	}
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if on, _ := a.ManualMode(ctx, nil); !on {
			break
		}
		time.Sleep(10 * time.Millisecond)
	}
	if on, _ := a.ManualMode(ctx, nil); on {
		t.Error("manual mode did not time out")
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "freedrive_teach_off()") {
		t.Error("freedrive_teach_off not sent")
	}
	props, _ := a.Properties(ctx, nil)
	if !props.SupportManualMode || !props.SupportCartesianCommands {
		t.Errorf("properties = %+v", props)
	}
}

func TestSetManualModeRefusedInSimulation(t *testing.T) {
	_, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	err := a.SetManualMode(context.Background(), true, 0, nil)
	if err == nil || !strings.Contains(err.Error(), "204") {
		t.Fatalf("expected refusal citing error 204, got %v", err)
	}
}

func TestDoCommand(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true, FK: func(j [6]float64) [6]float64 { return [6]float64{1, 2, 3, 0, 0, 0} }}, nil)
	ctx := context.Background()

	res, err := a.DoCommand(ctx, map[string]any{"set_speed": 3000.0})
	if err != nil {
		t.Fatal(err)
	}
	if res["speed_degs_per_sec"] != 2000.0 {
		t.Errorf("set_speed not clamped: %v", res)
	}
	res, _ = a.DoCommand(ctx, map[string]any{"get_speed": nil})
	if res["speed_degs_per_sec"] != 2000.0 {
		t.Errorf("get_speed = %v", res)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"bogus": 1}); err == nil || !strings.Contains(err.Error(), "bogus") {
		t.Errorf("expected unknown key error, got %v", err)
	}
	res, err = a.DoCommand(ctx, map[string]any{"get_state": nil})
	if err != nil {
		t.Fatal(err)
	}
	if res["activated"] != true || res["operation_mode"] != ModeSimulation || res["robot_state"] != 1.0 {
		t.Errorf("get_state = %v", res)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"set_speed_bar": 0.5}); err != nil {
		t.Fatal(err)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"set_collision": map[string]any{"enabled": true, "sensitivity": 0.6, "mode": 1.0, "after": 0.0}}); err != nil {
		t.Fatal(err)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"set_payload": map[string]any{"kg": 1.5, "cog_mm": []any{1.0, 2.0, 3.0}}}); err != nil {
		t.Fatal(err)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"set_box_dout": map[string]any{"port": 2.0, "value": 1.0}}); err != nil {
		t.Fatal(err)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"set_tool_out": map[string]any{"voltage": 24.0, "out0": 1.0, "out1": 0.0}}); err != nil {
		t.Fatal(err)
	}
	res, err = a.DoCommand(ctx, map[string]any{"calc_fk": []any{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
	if err != nil {
		t.Fatal(err)
	}
	if tcp := res["controller_tcp"].([]float64); tcp[2] != 3 {
		t.Errorf("calc_fk = %v", res)
	}
	if _, err := a.DoCommand(ctx, map[string]any{"eval": "task pause"}); err == nil {
		t.Error("eval should be disabled by default")
	}
	res, err = a.DoCommand(ctx, map[string]any{"set_operation_mode": ModeReal})
	if err != nil {
		t.Fatal(err)
	}
	if res["operation_mode"] != ModeReal {
		t.Errorf("set_operation_mode = %v", res)
	}
	joined := strings.Join(box.Commands(), "\n")
	for _, want := range []string{
		"set_speed_bar(0.5)", "set_collision_onoff(1)", "set_collision_th(0.6)", "set_collision_mode(1)", "set_collision_after(0)",
		"set_payload_info(1.5,1,2,3)", "set_box_dout(2,1)", "tool_out 24,1,0", "calc_fk_tcp(0,0,0,0,0,0)", "pgmode real",
	} {
		if !strings.Contains(joined, want) {
			t.Errorf("missing %q", want)
		}
	}
	res, err = a.DoCommand(ctx, map[string]any{"get_io": nil})
	if err != nil {
		t.Fatal(err)
	}
	if res["box_digital_out"].([]any)[2] != 1.0 || res["tool_voltage_out"] != 24.0 {
		t.Errorf("get_io = %v", res)
	}
	st, err := a.Status(ctx)
	if err != nil || st["ready"] != true {
		t.Errorf("Status = %v %v", st, err)
	}
}

func TestEvalWhenAllowed(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.AllowRawCommands = true })
	res, err := a.DoCommand(context.Background(), map[string]any{"eval": "set_speed_bar(0.9)"})
	if err != nil {
		t.Fatal(err)
	}
	if res["acked"] != true || box.State().DefaultSpeed != 0.9 {
		t.Errorf("eval result %v, speed %v", res, box.State().DefaultSpeed)
	}
}

func TestCloseIdempotentAndPowerOff(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.PowerOffOnClose = true })
	ctx := context.Background()
	if err := a.Close(ctx); err != nil {
		t.Fatal(err)
	}
	if err := a.Close(ctx); err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "AvadaKedavra()") {
		t.Error("power off not sent on close")
	}
	if _, err := a.JointPositions(ctx, nil); err == nil {
		t.Error("expected error after close")
	}
}

func TestCloseDoesNotPowerOffByDefault(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	_ = a.Close(context.Background())
	if strings.Contains(strings.Join(box.Commands(), "\n"), "AvadaKedavra()") {
		t.Error("power off sent without power_off_on_close")
	}
}

func TestStaleSnapshotRefused(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) { c.StatePollHz = 100 })
	// Kill the data connection and block reconnects long enough for the snapshot to go stale.
	box.Close()
	time.Sleep(snapshotMaxAge + 200*time.Millisecond)
	if _, err := a.JointPositions(context.Background(), nil); err == nil || !strings.Contains(err.Error(), "not currently known") {
		t.Errorf("expected stale error, got %v", err)
	}
}

func TestSimulatedArm(t *testing.T) {
	ctx := context.Background()
	s, err := NewSimulated(rdkarm.Named("sim"), SimConfig{SpeedDegsPerSec: 90, SimulateTime: true}, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close(ctx)
	if _, _, err := (&SimConfig{}).Validate("p"); err != nil {
		t.Fatal(err)
	}
	start := time.Now()
	if err := s.MoveToJointPositions(ctx, deg(9, 0, 0, 0, 0, 0), nil); err != nil {
		t.Fatal(err)
	}
	if d := time.Since(start); d < 80*time.Millisecond || d > time.Second {
		t.Errorf("simulated move took %v, expected ~100ms", d)
	}
	jp, _ := s.JointPositions(ctx, nil)
	if math.Abs(jp[0]-9*math.Pi/180) > 1e-9 {
		t.Errorf("joints = %v", jp)
	}
	if err := s.MoveToJointPositions(ctx, deg(0, 0, 170, 0, 0, 0), nil); err == nil {
		t.Error("expected limit error")
	}
	pose, err := s.EndPosition(ctx, nil)
	if err != nil || pose.Point().Z < 1000 {
		t.Errorf("end position %v %v", pose, err)
	}
	if geoms, err := s.Geometries(ctx, nil); err != nil || len(geoms) != 7 {
		t.Errorf("geometries %d %v", len(geoms), err)
	}
	res, err := s.DoCommand(ctx, map[string]any{"set_joints": []any{0.0, -30.0, 90.0, 0.0, 60.0, 0.0}})
	if err != nil {
		t.Fatal(err)
	}
	if res["joints_deg"] == nil {
		t.Errorf("set_joints = %v", res)
	}
	// Stop mid-move.
	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveToJointPositions(ctx, deg(0, -30, 90, 0, 60, 90), nil) }()
	time.Sleep(50 * time.Millisecond)
	if err := s.Stop(ctx, nil); err != nil {
		t.Fatal(err)
	}
	if err := <-errCh; err == nil {
		t.Error("expected cancelled move")
	}
	// Instant twin.
	inst, err := NewSimulated(rdkarm.Named("sim2"), SimConfig{}, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	defer inst.Close(ctx)
	start = time.Now()
	if err := inst.MoveThroughJointPositions(ctx, [][]referenceframe.Input{deg(10, 0, 0, 0, 0, 0), deg(20, 0, 0, 0, 0, 0)}, nil, nil); err != nil {
		t.Fatal(err)
	}
	if time.Since(start) > 100*time.Millisecond {
		t.Error("instant twin should not take time")
	}
	jp, _ = inst.JointPositions(ctx, nil)
	if math.Abs(jp[0]-20*math.Pi/180) > 1e-9 {
		t.Errorf("joints = %v", jp)
	}
}

func TestRegistration(t *testing.T) {
	for _, m := range []resource.Model{Model, SimulatedModel} {
		if _, ok := resource.LookupRegistration(rdkarm.API, m); !ok {
			t.Errorf("model %s not registered", m)
		}
	}
	if Model.String() != "viam:rainbow-robotics:rb10-1300" {
		t.Errorf("model name = %s", Model)
	}
}

func TestRainbowPoseRoundTrip(t *testing.T) {
	in := [6]float64{100, -200, 300, 10, -20, 30}
	out := SpatialToRainbowPose(RainbowPoseToSpatial(in))
	for i := range in {
		if math.Abs(in[i]-out[i]) > 1e-6 {
			t.Errorf("index %d: %v -> %v", i, in[i], out[i])
		}
	}
}

func TestSnapshotHelpers(t *testing.T) {
	var st rb.SystemState
	st.InitStateInfo = 6
	st.InformationChunk1 = rb.InfoBitEStop1 | rb.InfoBitEStop2 | rb.InfoBitArmPower // e-stop released
	a := &rb10{cfg: Config{OperationMode: ModeSimulation}}
	if r := a.faultReason(&st); r != "" {
		t.Errorf("unexpected fault %q", r)
	}
	st.JntInfo[4] = rb.JointBitTMP | rb.JointBitFLT
	if r := a.faultReason(&st); !strings.Contains(r, "joint 4") || !strings.Contains(r, "TMP") {
		t.Errorf("fault = %q", r)
	}
}

func TestPlanToPoseWithSVAModel(t *testing.T) {
	model, err := buildSVAModel("rb10", 60, 200)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	start := deg(0, -30, 90, 0, 60, 0)
	startPose, err := model.Transform(start)
	if err != nil {
		t.Fatal(err)
	}
	goal := spatialmath.NewPose(startPose.Point().Add(r3.Vector{X: 100, Y: 50, Z: -100}), startPose.Orientation())
	plan, err := armplanning.PlanFrameMotion(ctx, logging.NewTestLogger(t), goal, model, start, nil, nil)
	if err != nil {
		t.Fatalf("planning failed: %v", err)
	}
	if len(plan) < 2 {
		t.Fatalf("plan too short: %d", len(plan))
	}
	end, err := model.Transform(plan[len(plan)-1])
	if err != nil {
		t.Fatal(err)
	}
	if d := end.Point().Distance(goal.Point()); d > 5 {
		t.Errorf("plan ends %.1f mm from goal", d)
	}
	// Execute the plan on the simulated twin.
	s, err := NewSimulated(rdkarm.Named("sim"), SimConfig{}, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close(ctx)
	if _, err := s.DoCommand(ctx, map[string]any{"set_joints": []any{0.0, -30.0, 90.0, 0.0, 60.0, 0.0}}); err != nil {
		t.Fatal(err)
	}
	if err := s.MoveThroughJointPositions(ctx, plan, nil, nil); err != nil {
		t.Fatal(err)
	}
	// Without a motion service MoveToPosition is refused rather than planned
	// in-process, which keeps nlopt out of the module binary.
	if err := s.MoveToPosition(ctx, startPose, nil); !errors.Is(err, errNoMotionService) {
		t.Fatalf("MoveToPosition without motion service = %v, want errNoMotionService", err)
	}
}

func TestURDFMatchesSVA(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "..")
	urdf, err := buildURDFModel("rb10", nil)
	if err != nil {
		t.Fatal(err)
	}
	sva, err := buildSVAModel("rb10", 60, 200)
	if err != nil {
		t.Fatal(err)
	}
	if len(urdf.DoF()) != 6 {
		t.Fatalf("URDF DoF = %d", len(urdf.DoF()))
	}
	for _, pose := range [][]referenceframe.Input{deg(0, 0, 0, 0, 0, 0), deg(0, -30, 90, 0, 60, 0), deg(37, -80, 110, 25, -40, 70)} {
		a, err := urdf.Transform(pose)
		if err != nil {
			t.Fatal(err)
		}
		b, err := sva.Transform(pose)
		if err != nil {
			t.Fatal(err)
		}
		if d := a.Point().Distance(b.Point()); d > 1e-6 {
			t.Errorf("pose %v: URDF and SVA TCP differ by %v mm (%v vs %v)", pose, d, a.Point(), b.Point())
		}
		if th := spatialmath.OrientationBetween(a.Orientation(), b.Orientation()).AxisAngles().Theta; th > 1e-6 {
			t.Errorf("pose %v: URDF and SVA orientation differ by %v rad", pose, th)
		}
	}
	geoms, err := urdf.Geometries(deg(0, 0, 0, 0, 0, 0))
	if err != nil {
		t.Fatal(err)
	}
	if len(geoms.Geometries()) != 7 {
		t.Errorf("URDF geometries = %d, want 7 meshes", len(geoms.Geometries()))
	}
}

func TestLoad3DModels(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "..")
	models := load3DModels(logging.NewTestLogger(t))
	if len(models) != 7 {
		t.Fatalf("got %d models, want 7", len(models))
	}
	for _, link := range LinkNames {
		m, ok := models[link]
		if !ok || m.ContentType != "model/gltf-binary" || len(m.Mesh) < 1000 || string(m.Mesh[:4]) != "glTF" {
			t.Errorf("model %s missing or malformed", link)
		}
	}
	t.Setenv("VIAM_MODULE_ROOT", t.TempDir())
	if n := len(load3DModels(logging.NewTestLogger(t))); n != 0 {
		t.Errorf("expected no models from an empty root, got %d", n)
	}
}

func TestAutoReactivateAfterEStopRelease(t *testing.T) {
	oldDebounce, oldRetry := reactivateDebounce, reactivateRetry
	reactivateDebounce, reactivateRetry = 50*time.Millisecond, 200*time.Millisecond
	defer func() { reactivateDebounce, reactivateRetry = oldDebounce, oldRetry }()

	box, a := newTestArm(t, fakebox.Options{Activated: true}, func(c *Config) {
		c.AutoReactivate = ptr(true)
		c.SpeedBar = ptr(0.3)
	})
	ctx := context.Background()

	box.SetEStop(true)
	time.Sleep(50 * time.Millisecond)
	if err := a.MoveToJointPositions(ctx, deg(1, 0, 0, 0, 0, 0), nil); err == nil || !strings.Contains(err.Error(), "e-stop") {
		t.Fatalf("expected e-stop refusal while pressed, got %v", err)
	}
	// Held e-stop must not trigger activation.
	time.Sleep(150 * time.Millisecond)
	if strings.Contains(strings.Join(box.Commands(), "\n"), "mc jall init") {
		t.Fatal("activation attempted while the e-stop was held")
	}

	box.ClearCommands()
	box.SetEStop(false)
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) && box.State().InitStateInfo != rb.InitStateActivated {
		time.Sleep(10 * time.Millisecond)
	}
	if box.State().InitStateInfo != rb.InitStateActivated {
		t.Fatalf("arm not re-activated after e-stop release; commands: %v", box.Commands())
	}
	// The rest of the startup sequence follows activation; give it a moment.
	deadline = time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) && !strings.Contains(strings.Join(box.Commands(), "\n"), "set_speed_bar(0.3)") {
		time.Sleep(10 * time.Millisecond)
	}
	cmds := strings.Join(box.Commands(), "\n")
	if !strings.Contains(cmds, "mc jall init") || !strings.Contains(cmds, "set_speed_bar(0.3)") {
		t.Errorf("re-activation did not re-run the startup sequence: %v", box.Commands())
	}
	// Moves work again once startup has completed.
	deadline = time.Now().Add(2 * time.Second)
	var err error
	for time.Now().Before(deadline) {
		if err = a.MoveToJointPositions(ctx, deg(1, 0, 0, 0, 0, 0), nil); err == nil {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}
	if err != nil {
		t.Fatalf("move after auto re-activation: %v", err)
	}
}

func TestNoAutoReactivateByDefault(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	box.SetEStop(true)
	time.Sleep(30 * time.Millisecond)
	box.ClearCommands()
	box.SetEStop(false)
	time.Sleep(300 * time.Millisecond)
	if strings.Contains(strings.Join(box.Commands(), "\n"), "mc jall init") {
		t.Fatal("re-activated without auto_reactivate")
	}
	if err := a.MoveToJointPositions(context.Background(), deg(1, 0, 0, 0, 0, 0), nil); err == nil || !strings.Contains(err.Error(), "not activated") {
		t.Fatalf("expected 'not activated' refusal, got %v", err)
	}
}

func TestMoveLDoCommand(t *testing.T) {
	box, a := newTestArm(t, fakebox.Options{Activated: true}, nil)
	ctx := context.Background()
	box.Mutate(func(st *rb.SystemState) { st.TCPRef = [6]float32{0, -271.55, 1497, 0, 0, 0} })
	res, err := a.DoCommand(ctx, map[string]any{"move_l": map[string]any{
		"pose": []any{100.0, -271.55, 1400.0, 0.0, 0.0, 90.0}, "speed": 1000.0, "acceleration": 500.0,
	}})
	if err != nil {
		t.Fatalf("move_l: %v", err)
	}
	if res["speed_mm_s"] != maxLinearSpeedMMs {
		t.Errorf("speed not capped: %v", res["speed_mm_s"])
	}
	cmds := strings.Join(box.Commands(), "\n")
	if !strings.Contains(cmds, "move_l(pnt[100, -271.55, 1400, 0, 0, 90],250,500)") {
		t.Errorf("move_l script not sent: %v", box.Commands())
	}
	if tcp := box.State().TCPRef; tcp[0] != 100 || tcp[2] != 1400 || tcp[5] != 90 {
		t.Errorf("fake box tcp_ref = %v", tcp)
	}
	// move_jl uses joint speeds and the session defaults.
	if _, err := a.DoCommand(ctx, map[string]any{"move_jl": map[string]any{"pose": []any{0.0, -271.55, 1497.0, 0.0, 0.0, 0.0}}}); err != nil {
		t.Fatalf("move_jl: %v", err)
	}
	if !strings.Contains(strings.Join(box.Commands(), "\n"), "move_jl(pnt[0, -271.55, 1497, 0, 0, 0],1000,") {
		t.Errorf("move_jl script not sent: %v", box.Commands())
	}
	// Refused while e-stopped.
	box.SetEStop(true)
	time.Sleep(30 * time.Millisecond)
	if _, err := a.DoCommand(ctx, map[string]any{"move_l": map[string]any{"pose": []any{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}}); err == nil {
		t.Fatal("move_l accepted while e-stopped")
	}
}
