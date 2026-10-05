# rainbow-robotics module: conventions and safety rules

Read this before touching anything under `arm/` or `rb/`. The full design brief
is `RB10-1300 Viam Arm Module Scope.md`.

## Layout

- `rb/` is the protocol client. It has **zero Viam imports** so it can be tested
  alone and reused for a gripper model. Command formatters live in
  `rb/command.go` and are tested byte for byte; the 580-byte state layout in
  `rb/state.go` mirrors rbpodo's `data_type.hpp` field for field.
- `rb/fakebox/` is the in-process control box used by every test. Extend it
  when you add a command; never add a test that needs hardware outside
  `arm/hardware_test.go`.
- `arm/` is the Viam resource. `rb10.go` owns config, startup, safety checks and
  the simple API; `move.go` the blocking moves; `streamed.go` the real-time
  executor; `docommand.go` the DoCommand table; `simulated.go` the twin.
- Package name is `arm`; import the RDK one as `rdkarm`.

## Concurrency rules

- The data poller is the only writer of the snapshot. Read it with
  `client.Snapshot()` / `NextSnapshot()`; never cache state elsewhere.
- One `operation.SingleOperationManager` guards all moves; `Stop` cancels it.
- Commands are matched to acknowledgements by sequence number: every script line
  gets exactly one ACK (or "not allowed") in order. `SendNoAck` is only for
  `move_servo_j` streaming; always follow a stream with a `Send` barrier.
- A move's completion is decided from the snapshot (`robot_state`, joint
  distance to target, fault bits), never from a single event line, because
  events are broadcast to every client including the pendant.
- `Close` must stay idempotent and must never power the arm down unless
  `power_off_on_close` is set.

## Safety rules

- Never default a new attribute to something that moves the arm faster.
- `ensureControllable()` runs before every move and is the whole safety story.
  Any new motion path must call it and must honour `ctx.Done()` by sending
  `task stop`.
- Every new command path must be exercised in the controller's simulation mode
  (`operation_mode: simulation`) before real mode.
- Tests that talk to a real box go in `arm/hardware_test.go`, are gated on the
  `RB_HOST` environment variable, and only ever use simulation mode.
- Faults (e-stop, collision, soft e-stop, EMS, joint error bits) are never
  auto-cleared; `clear_error` is an explicit DoCommand.
- Log every script line sent and every non-ACK line received at debug, refused
  moves and state transitions at info, controller errors at warn.

## Units

Degrees, millimetres and seconds on the wire; radians and millimetres inside
(`referenceframe.Input`). Convert only in `arm/kinematics.go` helpers. Wire
degrees are rounded to a micro-degree so scripts never carry float noise.

## Build and test

    make lint     # gofmt, go vet, golangci-lint (etc/.golangci.yaml)
    make test     # go test -race ./...
    make module   # bin/module.tar.gz for viam module build
    RB_HOST=10.0.2.7 make test-hardware

## Things marked "verify on hardware" in the scope

Batch size limit of `move_jb2`, meaning of the blend argument, the recovery
script after a collision (`clear_error`), whether `AvadaKedavra()` is safe from
a driver, per-joint speed ceilings, the Euler convention of `tcp_ref`, the
Get3DModels mesh frame convention in the app, and the flange variant
(`rb10_1300e` vs `rb10_1300e_u`, 3 mm at wrist1). Update README when resolved.
