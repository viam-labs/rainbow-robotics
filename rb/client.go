// Package rb is a pure-Go client for the Rainbow Robotics RB-series control
// box. It speaks the newline-terminated ASCII script protocol on the command
// port (5000) and polls the binary systemSTAT packet on the data port (5001).
// It has no Viam dependencies so it can be tested in isolation and reused.
package rb

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"net"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

// Logger is the minimal logging surface the client needs; rdk's logging.Logger satisfies it.
type Logger interface {
	Debugf(format string, args ...any)
	Infof(format string, args ...any)
	Warnf(format string, args ...any)
	Errorf(format string, args ...any)
}

type nopLogger struct{}

func (nopLogger) Debugf(string, ...any) {}
func (nopLogger) Infof(string, ...any)  {}
func (nopLogger) Warnf(string, ...any)  {}
func (nopLogger) Errorf(string, ...any) {}

// Config configures a Client.
type Config struct {
	Host        string
	CommandPort int // default 5000
	DataPort    int // default 5001

	DialTimeout      time.Duration // default 5s
	CommandTimeout   time.Duration // ACK wait, default 2s
	PollInterval     time.Duration // reqdata period, default 10ms
	ReconnectBackoff time.Duration // default 1s
	Logger           Logger
}

func (c *Config) withDefaults() Config {
	out := *c
	if out.CommandPort == 0 {
		out.CommandPort = 5000
	}
	if out.DataPort == 0 {
		out.DataPort = 5001
	}
	if out.DialTimeout == 0 {
		out.DialTimeout = 5 * time.Second
	}
	if out.CommandTimeout == 0 {
		out.CommandTimeout = 2 * time.Second
	}
	if out.PollInterval == 0 {
		out.PollInterval = 10 * time.Millisecond
	}
	if out.ReconnectBackoff == 0 {
		out.ReconnectBackoff = time.Second
	}
	if out.Logger == nil {
		out.Logger = nopLogger{}
	}
	return out
}

// Snapshot is the most recent state packet and when it was received.
type Snapshot struct {
	State SystemState
	At    time.Time
}

// Age returns how old the snapshot is.
func (s Snapshot) Age() time.Duration { return time.Since(s.At) }

// Client owns both sockets to one control box. All methods are safe for
// concurrent use. Commands are serialised: one script line is in flight at a
// time, and every line is matched to its acknowledgement by sequence number
// (the box acknowledges every script line exactly once, in order).
type Client struct {
	cfg Config
	log Logger

	ctx    context.Context
	cancel context.CancelFunc
	wg     sync.WaitGroup
	closed atomic.Bool

	cmd  *cmdConn
	data *dataConn

	reconnectMu sync.Mutex
	onReconnect func()
}

// Dial connects to both ports. It fails if either connection cannot be
// established within DialTimeout; after a successful Dial each connection is
// re-established automatically with ReconnectBackoff on any error.
func Dial(ctx context.Context, cfg Config) (*Client, error) {
	cfg = cfg.withDefaults()
	if cfg.Host == "" {
		return nil, errors.New("rb: host is required")
	}
	c := &Client{cfg: cfg, log: cfg.Logger}
	c.ctx, c.cancel = context.WithCancel(context.Background())
	c.cmd = newCmdConn(c)
	c.data = newDataConn(c)

	cmdSock, err := c.dial(ctx, cfg.CommandPort)
	if err != nil {
		c.cancel()
		return nil, fmt.Errorf("rb: command port: %w", err)
	}
	dataSock, err := c.dial(ctx, cfg.DataPort)
	if err != nil {
		_ = cmdSock.Close()
		c.cancel()
		return nil, fmt.Errorf("rb: data port: %w", err)
	}

	// Attach synchronously so Send and Snapshot work as soon as Dial returns.
	c.cmd.attach(cmdSock)
	c.data.attach(dataSock)
	c.wg.Add(2)
	go func() { defer c.wg.Done(); c.cmd.run(cmdSock) }()
	go func() { defer c.wg.Done(); c.data.run(dataSock) }()
	return c, nil
}

func (c *Client) dial(ctx context.Context, port int) (net.Conn, error) {
	d := net.Dialer{Timeout: c.cfg.DialTimeout}
	conn, err := d.DialContext(ctx, "tcp", net.JoinHostPort(c.cfg.Host, strconv.Itoa(port)))
	if err != nil {
		return nil, err
	}
	if tc, ok := conn.(*net.TCPConn); ok {
		_ = tc.SetNoDelay(true)
		_ = tc.SetKeepAlive(true)
		_ = tc.SetKeepAlivePeriod(5 * time.Second)
	}
	return conn, nil
}

// Close shuts both connections down and waits (bounded) for the goroutines.
// It is idempotent.
func (c *Client) Close() error {
	if !c.closed.CompareAndSwap(false, true) {
		return nil
	}
	c.cancel()
	c.cmd.closeCurrent(0)
	c.data.closeCurrent()
	done := make(chan struct{})
	go func() { c.wg.Wait(); close(done) }()
	select {
	case <-done:
	case <-time.After(5 * time.Second):
		c.log.Warnf("rb: timed out waiting for connection goroutines to exit")
	}
	return nil
}

// OnReconnect registers a function that is called, on its own goroutine, each
// time the command connection is re-established after having been lost.
func (c *Client) OnReconnect(f func()) {
	c.reconnectMu.Lock()
	defer c.reconnectMu.Unlock()
	c.onReconnect = f
}

// Connected reports the state of the command and data connections.
func (c *Client) Connected() (command, data bool) {
	return c.cmd.isConnected(), c.data.isConnected()
}

// Generation increments each time the command connection is (re)established,
// so a caller can tell whether a connection was lost during an operation.
func (c *Client) Generation() uint64 { return c.cmd.generation.Load() }

// Send writes one script line and waits for its acknowledgement.
func (c *Client) Send(ctx context.Context, script string) error {
	return c.cmd.send(ctx, script, true)
}

// SendNoAck writes one script line without waiting for its acknowledgement.
// It is used for real-time setpoint streaming; a later Send acts as a barrier.
func (c *Client) SendNoAck(script string) error {
	return c.cmd.send(context.Background(), script, false)
}

// NewWaiter registers a waiter for lines matching pred. Register before sending
// the command whose reply you expect, and always Close the waiter.
func (c *Client) NewWaiter(pred func(Line) bool) *Waiter {
	return c.cmd.newWaiter(pred)
}

// Query sends print(expr,"<tag>[", "]") and returns the printed value.
func (c *Client) Query(ctx context.Context, expr string) (string, error) {
	tag := NewTag()
	w := c.NewWaiter(func(l Line) bool {
		return l.IsPrint() && strings.HasPrefix(l.Msg, tag+"[") && strings.HasSuffix(l.Msg, "]")
	})
	defer w.Close()
	if err := c.Send(ctx, PrintTagged(expr, tag)); err != nil {
		return "", err
	}
	l, err := w.Wait(ctx, c.cfg.CommandTimeout)
	if err != nil {
		return "", fmt.Errorf("rb: no reply to print(%s): %w", expr, err)
	}
	return l.Msg[len(tag)+1 : len(l.Msg)-1], nil
}

// LastError returns the most recent error[code][N] line seen on the command
// port, or nil.
func (c *Client) LastError() *ControllerError { return c.cmd.lastError() }

// LastErrorSince returns the most recent controller error received at or after t, or nil.
func (c *Client) LastErrorSince(t time.Time) *ControllerError {
	e := c.cmd.lastError()
	if e == nil || e.At.Before(t) {
		return nil
	}
	return e
}

// Snapshot returns the latest state packet. ok is false before the first packet.
func (c *Client) Snapshot() (snap Snapshot, ok bool) { return c.data.snapshot() }

// NextSnapshot blocks until a state packet newer than the one identified by
// after arrives (pass the zero time for "any"), the timeout elapses, or ctx ends.
func (c *Client) NextSnapshot(ctx context.Context, after time.Time, timeout time.Duration) (Snapshot, error) {
	return c.data.next(ctx, after, timeout)
}

// WaitState polls fresh snapshots until pred is true.
func (c *Client) WaitState(ctx context.Context, timeout time.Duration, pred func(*SystemState) bool) (Snapshot, error) {
	deadline := time.Now().Add(timeout)
	var last time.Time
	for {
		remaining := time.Until(deadline)
		if remaining <= 0 {
			return Snapshot{}, ErrTimeout
		}
		snap, err := c.data.next(ctx, last, remaining)
		if err != nil {
			return snap, err
		}
		last = snap.At
		if pred(&snap.State) {
			return snap, nil
		}
	}
}

// ---------------------------------------------------------------------------
// Command connection

// Waiter receives lines matching a predicate.
type Waiter struct {
	id   uint64
	pred func(Line) bool
	ch   chan Line
	cc   *cmdConn
}

// Wait returns the next matching line.
func (w *Waiter) Wait(ctx context.Context, timeout time.Duration) (Line, error) {
	t := time.NewTimer(timeout)
	defer t.Stop()
	select {
	case l := <-w.ch:
		return l, nil
	case <-t.C:
		return Line{}, ErrTimeout
	case <-ctx.Done():
		return Line{}, ctx.Err()
	case <-w.cc.c.ctx.Done():
		return Line{}, ErrClosed
	}
}

// Close unregisters the waiter.
func (w *Waiter) Close() {
	w.cc.waitersMu.Lock()
	delete(w.cc.waiters, w.id)
	w.cc.waitersMu.Unlock()
}

type cmdConn struct {
	c *Client

	mu         sync.Mutex
	conn       net.Conn
	connected  bool
	generation atomic.Uint64
	suspect    bool

	writeMu sync.Mutex // one writer; keeps lines atomic and orders sequence numbers
	sent    uint64     // commands written on the current connection (guarded by writeMu)

	ackMu     sync.Mutex
	acked     uint64        // ACKs received on the current connection
	ackNotify chan struct{} // closed and replaced on every ACK

	waitersMu  sync.Mutex
	waiters    map[uint64]*Waiter
	nextWaiter uint64

	errMu   sync.Mutex
	lastErr *ControllerError
}

func newCmdConn(c *Client) *cmdConn {
	return &cmdConn{c: c, waiters: map[uint64]*Waiter{}, ackNotify: make(chan struct{})}
}

func (cc *cmdConn) isConnected() bool {
	cc.mu.Lock()
	defer cc.mu.Unlock()
	return cc.connected
}

func (cc *cmdConn) lastError() *ControllerError {
	cc.errMu.Lock()
	defer cc.errMu.Unlock()
	if cc.lastErr == nil {
		return nil
	}
	e := *cc.lastErr
	return &e
}

func (cc *cmdConn) newWaiter(pred func(Line) bool) *Waiter {
	cc.waitersMu.Lock()
	defer cc.waitersMu.Unlock()
	cc.nextWaiter++
	w := &Waiter{id: cc.nextWaiter, pred: pred, ch: make(chan Line, 32), cc: cc}
	cc.waiters[w.id] = w
	return w
}

// run supervises the command connection for the life of the client. initial
// must already be attached.
func (cc *cmdConn) run(initial net.Conn) {
	c := cc.c
	conn := initial
	attempts := 0
	everConnected := false
	attached := true
	for {
		if conn == nil {
			var err error
			conn, err = c.dial(c.ctx, c.cfg.CommandPort)
			if err != nil {
				attempts++
				if attempts%30 == 1 {
					c.log.Warnf("rb: command port reconnect attempt %d failed: %v", attempts, err)
				}
				if !sleepCtx(c.ctx, c.cfg.ReconnectBackoff) {
					return
				}
				continue
			}
			attempts = 0
			attached = false
		}
		if !attached {
			cc.attach(conn)
		}
		if everConnected {
			c.log.Infof("rb: command connection to %s re-established", c.cfg.Host)
			c.reconnectMu.Lock()
			f := c.onReconnect
			c.reconnectMu.Unlock()
			if f != nil {
				go f()
			}
		}
		everConnected = true

		err := cc.readLoop(conn)
		cc.detach()
		_ = conn.Close()
		conn = nil
		if c.ctx.Err() != nil {
			return
		}
		c.log.Warnf("rb: command connection lost: %v; reconnecting", err)
		if !sleepCtx(c.ctx, c.cfg.ReconnectBackoff) {
			return
		}
	}
}

func (cc *cmdConn) attach(conn net.Conn) {
	cc.writeMu.Lock()
	cc.ackMu.Lock()
	cc.sent = 0
	cc.acked = 0
	close(cc.ackNotify)
	cc.ackNotify = make(chan struct{})
	cc.ackMu.Unlock()
	cc.writeMu.Unlock()

	cc.mu.Lock()
	cc.conn = conn
	cc.connected = true
	cc.suspect = false
	cc.generation.Add(1)
	cc.mu.Unlock()
}

func (cc *cmdConn) detach() {
	cc.mu.Lock()
	cc.conn = nil
	cc.connected = false
	cc.mu.Unlock()
	// Wake any Send blocked on an ACK so it can observe the disconnect.
	cc.ackMu.Lock()
	close(cc.ackNotify)
	cc.ackNotify = make(chan struct{})
	cc.ackMu.Unlock()
}

// closeCurrent closes the live connection, forcing a reconnect. gen limits the
// close to a specific connection generation (0 = any).
func (cc *cmdConn) closeCurrent(gen uint64) {
	cc.mu.Lock()
	defer cc.mu.Unlock()
	if cc.conn != nil && (gen == 0 || cc.generation.Load() == gen) {
		_ = cc.conn.Close()
	}
}

func (cc *cmdConn) readLoop(conn net.Conn) error {
	c := cc.c
	sc := bufio.NewScanner(conn)
	sc.Buffer(make([]byte, 0, 64*1024), 1024*1024)
	for sc.Scan() {
		raw := strings.TrimSpace(sc.Text())
		if raw == "" {
			continue
		}
		l := ParseLine(raw)
		switch {
		case l.IsACK():
			cc.ackMu.Lock()
			cc.acked++
			close(cc.ackNotify)
			cc.ackNotify = make(chan struct{})
			cc.ackMu.Unlock()
			if l.Type == LineNotAllowed {
				c.log.Warnf("rb: control box replied %q", raw)
			}
		case l.Type == LineError:
			if code, ok := l.ErrorCode(); ok {
				cc.errMu.Lock()
				cc.lastErr = &ControllerError{Code: code, At: time.Now(), Raw: raw}
				cc.errMu.Unlock()
				c.log.Warnf("rb: control box error %s", ErrorText(code))
			} else {
				c.log.Warnf("rb: control box error line %q", raw)
			}
		default:
			c.log.Debugf("rb: <- %s", raw)
		}
		cc.deliver(l)
	}
	if err := sc.Err(); err != nil {
		return err
	}
	return errors.New("connection closed by control box")
}

func (cc *cmdConn) deliver(l Line) {
	cc.waitersMu.Lock()
	defer cc.waitersMu.Unlock()
	for _, w := range cc.waiters {
		if w.pred(l) {
			select {
			case w.ch <- l:
			default:
				cc.c.log.Warnf("rb: dropping line for slow waiter: %s", l.Raw)
			}
		}
	}
}

func (cc *cmdConn) send(ctx context.Context, script string, waitAck bool) error {
	c := cc.c
	if c.closed.Load() {
		return ErrClosed
	}
	cc.writeMu.Lock()
	cc.mu.Lock()
	conn := cc.conn
	gen := cc.generation.Load()
	cc.mu.Unlock()
	if conn == nil {
		cc.writeMu.Unlock()
		return ErrNotConnected
	}
	cc.sent++
	seq := cc.sent
	c.log.Debugf("rb: -> %s", script)
	_ = conn.SetWriteDeadline(time.Now().Add(c.cfg.CommandTimeout))
	_, err := conn.Write([]byte(script + "\n"))
	cc.writeMu.Unlock()
	if err != nil {
		cc.closeCurrent(gen)
		return fmt.Errorf("rb: write %q: %w", script, err)
	}
	if !waitAck {
		return nil
	}
	return cc.waitAck(ctx, seq, gen, script)
}

func (cc *cmdConn) waitAck(ctx context.Context, seq, gen uint64, script string) error {
	timer := time.NewTimer(cc.c.cfg.CommandTimeout)
	defer timer.Stop()
	for {
		cc.ackMu.Lock()
		acked := cc.acked
		notify := cc.ackNotify
		cc.ackMu.Unlock()
		if cc.generation.Load() != gen || !cc.isConnected() {
			return fmt.Errorf("rb: connection lost while waiting for ack of %q: %w", script, ErrNotConnected)
		}
		if acked >= seq {
			cc.mu.Lock()
			cc.suspect = false
			cc.mu.Unlock()
			return nil
		}
		select {
		case <-notify:
		case <-timer.C:
			cc.mu.Lock()
			wasSuspect := cc.suspect
			cc.suspect = true
			cc.mu.Unlock()
			if wasSuspect {
				cc.c.log.Warnf("rb: second consecutive ack timeout; reconnecting command port")
				cc.closeCurrent(gen)
			}
			return fmt.Errorf("rb: no ack for %q within %v: %w", script, cc.c.cfg.CommandTimeout, ErrTimeout)
		case <-ctx.Done():
			return ctx.Err()
		case <-cc.c.ctx.Done():
			return ErrClosed
		}
	}
}

// ---------------------------------------------------------------------------
// Data connection

type dataConn struct {
	c *Client

	mu        sync.Mutex
	conn      net.Conn
	connected bool

	snapMu sync.RWMutex
	snap   Snapshot
	have   bool
	notify chan struct{}
}

func newDataConn(c *Client) *dataConn {
	return &dataConn{c: c, notify: make(chan struct{})}
}

func (dc *dataConn) isConnected() bool {
	dc.mu.Lock()
	defer dc.mu.Unlock()
	return dc.connected
}

func (dc *dataConn) snapshot() (Snapshot, bool) {
	dc.snapMu.RLock()
	defer dc.snapMu.RUnlock()
	return dc.snap, dc.have
}

func (dc *dataConn) store(st SystemState) {
	dc.snapMu.Lock()
	dc.snap = Snapshot{State: st, At: time.Now()}
	dc.have = true
	close(dc.notify)
	dc.notify = make(chan struct{})
	dc.snapMu.Unlock()
}

func (dc *dataConn) next(ctx context.Context, after time.Time, timeout time.Duration) (Snapshot, error) {
	timer := time.NewTimer(timeout)
	defer timer.Stop()
	for {
		dc.snapMu.RLock()
		snap, have, notify := dc.snap, dc.have, dc.notify
		dc.snapMu.RUnlock()
		if have && snap.At.After(after) {
			return snap, nil
		}
		select {
		case <-notify:
		case <-timer.C:
			return Snapshot{}, ErrTimeout
		case <-ctx.Done():
			return Snapshot{}, ctx.Err()
		case <-dc.c.ctx.Done():
			return Snapshot{}, ErrClosed
		}
	}
}

func (dc *dataConn) closeCurrent() {
	dc.mu.Lock()
	defer dc.mu.Unlock()
	if dc.conn != nil {
		_ = dc.conn.Close()
	}
}

func (dc *dataConn) attach(conn net.Conn) {
	dc.mu.Lock()
	dc.conn = conn
	dc.connected = true
	dc.mu.Unlock()
}

// run supervises the data connection for the life of the client. initial must
// already be attached.
func (dc *dataConn) run(initial net.Conn) {
	c := dc.c
	conn := initial
	attempts := 0
	attached := true
	for {
		if conn == nil {
			var err error
			conn, err = c.dial(c.ctx, c.cfg.DataPort)
			if err != nil {
				attempts++
				if attempts%30 == 1 {
					c.log.Warnf("rb: data port reconnect attempt %d failed: %v", attempts, err)
				}
				if !sleepCtx(c.ctx, c.cfg.ReconnectBackoff) {
					return
				}
				continue
			}
			attempts = 0
			attached = false
		}
		if !attached {
			dc.attach(conn)
		}

		err := dc.pollLoop(conn)

		dc.mu.Lock()
		dc.conn = nil
		dc.connected = false
		dc.mu.Unlock()
		_ = conn.Close()
		conn = nil
		if c.ctx.Err() != nil {
			return
		}
		c.log.Warnf("rb: data connection lost: %v; reconnecting", err)
		if !sleepCtx(c.ctx, c.cfg.ReconnectBackoff) {
			return
		}
	}
}

func (dc *dataConn) pollLoop(conn net.Conn) error {
	c := dc.c
	br := bufio.NewReaderSize(conn, 4096)
	readTimeout := max(c.cfg.PollInterval*20, 500*time.Millisecond)
	ticker := time.NewTicker(c.cfg.PollInterval)
	defer ticker.Stop()
	for {
		_ = conn.SetWriteDeadline(time.Now().Add(readTimeout))
		if _, err := conn.Write([]byte("reqdata\n")); err != nil {
			return fmt.Errorf("write reqdata: %w", err)
		}
		_ = conn.SetReadDeadline(time.Now().Add(readTimeout))
		pkt, err := ReadPacket(br)
		if err != nil {
			return fmt.Errorf("read state packet: %w", err)
		}
		st, err := DecodeSystemState(pkt)
		if err != nil {
			c.log.Warnf("rb: discarding state packet: %v", err)
		} else {
			dc.store(st)
		}
		select {
		case <-ticker.C:
		case <-c.ctx.Done():
			return c.ctx.Err()
		}
	}
}

func sleepCtx(ctx context.Context, d time.Duration) bool {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-t.C:
		return true
	case <-ctx.Done():
		return false
	}
}
