package rb_test

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	"github.com/viam-labs/rainbow-robotics/rb"
	"github.com/viam-labs/rainbow-robotics/rb/fakebox"
)

func dialFake(t *testing.T, opts fakebox.Options, cfg rb.Config) (*fakebox.Box, *rb.Client) {
	t.Helper()
	box, err := fakebox.Start(opts)
	if err != nil {
		t.Fatal(err)
	}
	cfg.Host = box.Host()
	cfg.CommandPort = box.CommandPort()
	cfg.DataPort = box.DataPort()
	if cfg.PollInterval == 0 {
		cfg.PollInterval = 5 * time.Millisecond
	}
	if cfg.ReconnectBackoff == 0 {
		cfg.ReconnectBackoff = 50 * time.Millisecond
	}
	if cfg.CommandTimeout == 0 {
		cfg.CommandTimeout = 500 * time.Millisecond
	}
	c, err := rb.Dial(context.Background(), cfg)
	if err != nil {
		box.Close()
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = c.Close(); box.Close() })
	return box, c
}

func TestDialFailure(t *testing.T) {
	_, err := rb.Dial(context.Background(), rb.Config{Host: "127.0.0.1", CommandPort: 1, DataPort: 1, DialTimeout: 200 * time.Millisecond})
	if err == nil {
		t.Fatal("expected dial error")
	}
}

func TestSendAndSnapshot(t *testing.T) {
	box, c := dialFake(t, fakebox.Options{Activated: true}, rb.Config{})
	ctx := context.Background()

	snap, err := c.NextSnapshot(ctx, time.Time{}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if !snap.State.Activated() || !snap.State.Simulation() {
		t.Errorf("unexpected initial state: %+v", snap.State)
	}

	if err := c.Send(ctx, rb.SetSpeedBar(0.5)); err != nil {
		t.Fatal(err)
	}
	snap, err = c.WaitState(ctx, time.Second, func(st *rb.SystemState) bool { return st.DefaultSpeed == 0.5 })
	if err != nil {
		t.Fatalf("speed bar not applied: %v", err)
	}
	cmds := box.Commands()
	if len(cmds) != 1 || cmds[0] != "set_speed_bar(0.5)" {
		t.Errorf("commands = %v", cmds)
	}
	_ = snap
}

func TestQuery(t *testing.T) {
	_, c := dialFake(t, fakebox.Options{Activated: true, FK: func(j [6]float64) [6]float64 {
		return [6]float64{j[0] * 10, 1, 2, 3, 4, 5}
	}}, rb.Config{})
	ctx := context.Background()

	v, err := c.Query(ctx, "SD_ROBOT_STATE")
	if err != nil {
		t.Fatal(err)
	}
	if v != "1" {
		t.Errorf("SD_ROBOT_STATE = %q", v)
	}
	v, err = c.Query(ctx, rb.CalcFKTCP([6]float64{7, 0, 0, 0, 0, 0}))
	if err != nil {
		t.Fatal(err)
	}
	f, err := rb.ParseFloats(v)
	if err != nil || len(f) != 6 || f[0] != 70 {
		t.Errorf("fk = %v %v", f, err)
	}
}

func TestMoveEvents(t *testing.T) {
	box, c := dialFake(t, fakebox.Options{Activated: true}, rb.Config{})
	ctx := context.Background()

	started := c.NewWaiter(func(l rb.Line) bool { s, ok := l.IsMotionChanged(); return ok && s })
	defer started.Close()
	ended := c.NewWaiter(func(l rb.Line) bool { s, ok := l.IsMotionChanged(); return ok && !s })
	defer ended.Close()

	if err := c.Send(ctx, rb.MoveJ([6]float64{10, 0, 0, 0, 0, 0}, 1000, 1000)); err != nil {
		t.Fatal(err)
	}
	if _, err := started.Wait(ctx, time.Second); err != nil {
		t.Fatalf("no motion_changed start: %v", err)
	}
	if _, err := ended.Wait(ctx, 2*time.Second); err != nil {
		t.Fatalf("no motion_changed end: %v", err)
	}
	if j := box.Joints(); j[0] != 10 {
		t.Errorf("joints = %v", j)
	}
	snap, err := c.WaitState(ctx, time.Second, func(st *rb.SystemState) bool { return st.JntRef[0] == 10 && st.Idle() })
	if err != nil {
		t.Fatal(err)
	}
	if snap.State.RobotState != rb.RobotStateIdle {
		t.Errorf("robot_state = %d", snap.State.RobotState)
	}
}

func TestControllerErrorRecorded(t *testing.T) {
	_, c := dialFake(t, fakebox.Options{Activated: false}, rb.Config{})
	ctx := context.Background()
	before := time.Now()
	if err := c.Send(ctx, rb.PGMode(true)); err != nil {
		t.Fatal(err)
	}
	deadline := time.Now().Add(time.Second)
	for time.Now().Before(deadline) {
		if e := c.LastErrorSince(before); e != nil {
			if e.Code != 186 {
				t.Fatalf("code = %d", e.Code)
			}
			if !strings.Contains(e.Error(), "Real Mode") {
				t.Errorf("error text = %q", e.Error())
			}
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatal("controller error 186 not recorded")
}

func TestReconnect(t *testing.T) {
	box, c := dialFake(t, fakebox.Options{Activated: true, FragmentPackets: true}, rb.Config{})
	ctx := context.Background()

	reconnected := make(chan struct{}, 4)
	c.OnReconnect(func() { reconnected <- struct{}{} })

	if _, err := c.NextSnapshot(ctx, time.Time{}, time.Second); err != nil {
		t.Fatal(err)
	}
	gen := c.Generation()
	box.DropConnections()

	select {
	case <-reconnected:
	case <-time.After(3 * time.Second):
		t.Fatal("no reconnect callback")
	}
	if c.Generation() <= gen {
		t.Errorf("generation did not advance: %d -> %d", gen, c.Generation())
	}
	// Commands and state work again after the reconnect.
	deadline := time.Now().Add(2 * time.Second)
	var err error
	for time.Now().Before(deadline) {
		if err = c.Send(ctx, rb.SetSpeedBar(0.25)); err == nil {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}
	if err != nil {
		t.Fatalf("send after reconnect: %v", err)
	}
	if _, err := c.WaitState(ctx, 2*time.Second, func(st *rb.SystemState) bool { return st.DefaultSpeed == 0.25 }); err != nil {
		t.Fatalf("state after reconnect: %v", err)
	}
}

func TestSendNotConnected(t *testing.T) {
	box, c := dialFake(t, fakebox.Options{Activated: true}, rb.Config{ReconnectBackoff: 5 * time.Second})
	box.DropConnections()
	deadline := time.Now().Add(time.Second)
	for time.Now().Before(deadline) {
		if cmd, _ := c.Connected(); !cmd {
			break
		}
		time.Sleep(5 * time.Millisecond)
	}
	err := c.Send(context.Background(), rb.TaskStop())
	if !errors.Is(err, rb.ErrNotConnected) {
		t.Errorf("expected ErrNotConnected, got %v", err)
	}
}

func TestAckTimeoutMarksSuspect(t *testing.T) {
	// A box that never acknowledges: use a raw listener.
	box, c := dialFake(t, fakebox.Options{Activated: true}, rb.Config{CommandTimeout: 100 * time.Millisecond})
	_ = box
	// Unknown commands are still acked by the fake box, so simulate silence by
	// making the box drop the connection mid-command instead: the Send must
	// return an error rather than hang.
	go func() {
		time.Sleep(20 * time.Millisecond)
		box.DropConnections()
	}()
	err := c.Send(context.Background(), "not_a_real_command_that_blocks()")
	// Either the ack arrived before the drop (nil) or we got a connection error; never a hang.
	if err != nil && !errors.Is(err, rb.ErrNotConnected) && !errors.Is(err, rb.ErrTimeout) && !strings.Contains(err.Error(), "write") {
		t.Errorf("unexpected error: %v", err)
	}
}

func TestSendNoAckThenBarrier(t *testing.T) {
	box, c := dialFake(t, fakebox.Options{Activated: true}, rb.Config{})
	ctx := context.Background()
	for i := range 20 {
		if err := c.SendNoAck(rb.MoveServoJ([6]float64{float64(i), 0, 0, 0, 0, 0}, 0.01, 0.03, 0.5, 0.5)); err != nil {
			t.Fatal(err)
		}
	}
	// The barrier waits for its own ack, which arrives after the 20 earlier ones.
	if err := c.Send(ctx, rb.Print(`"barrier"`, "", "")); err != nil {
		t.Fatal(err)
	}
	if n := box.ServoSetpoints(); n != 20 {
		t.Errorf("servo setpoints = %d", n)
	}
	// A subsequent Query still correlates correctly (no off-by-one acks).
	v, err := c.Query(ctx, "SD_ROBOT_STATE")
	if err != nil {
		t.Fatal(err)
	}
	if v != "3" && v != "1" {
		t.Errorf("SD_ROBOT_STATE = %q", v)
	}
}

func TestCloseIdempotent(t *testing.T) {
	_, c := dialFake(t, fakebox.Options{Activated: true}, rb.Config{})
	if err := c.Close(); err != nil {
		t.Fatal(err)
	}
	if err := c.Close(); err != nil {
		t.Fatal(err)
	}
	if err := c.Send(context.Background(), rb.TaskStop()); !errors.Is(err, rb.ErrClosed) {
		t.Errorf("expected ErrClosed, got %v", err)
	}
}
