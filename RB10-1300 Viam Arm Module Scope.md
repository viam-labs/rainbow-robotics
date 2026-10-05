# RB10-1300 Viam Arm Module Scope

Sep 29, 2026 · @Brad Grigsby

## Summary

Build a pure-Go Viam arm module for the Rainbow Robotics RB10-1300 that talks directly to the control box over its two TCP sockets, with no vendor SDK and no cgo. The controller speaks newline-terminated ASCII script commands on port 5000 and returns a fixed 580-byte binary state packet on port 5001, which is the same shape of protocol Viam's Go Dobot and UFactory modules already implement natively. Go is therefore the right choice, not a compromise.

The module registers one model, `viam:rainbow-robotics:rb10-1300`, implements the full RDK `arm.Arm` interface (including `MoveThroughJointPositionsStreamed`, `Get3DModels`, `Properties`, `SetManualMode`), ships an SVA kinematics file derived from Rainbow's published DH parameters and URDF, and hands Cartesian moves to Viam's motion planner. A `rb10-1300-simulated` twin and the controller's own `pgmode simulation` cover development without hardware.

This document is the hand-off brief for Claude Code. It records what the reference modules do, what the RB10 interface looks like on the wire, the decisions already made, and the order to build in. Everything marked **verify on hardware** was read from source and docs, not tested on the arm.

## How existing Viam arm modules are built

Every Viam-maintained arm driver with a documented TCP protocol is written in Go with no vendor SDK; C++ was used only where a native vendor library or custom controller firmware was unavoidable. The Dobot CR10A module is the closest analog to what the RB10 needs: ASCII commands on one socket, a fixed binary feedback struct on another.

| Module | Language | Transport to controller | MoveToPosition | MoveThroughJointPositions | Stop / IsMoving | Kinematics shipped |
| --- | --- | --- | --- | --- | --- | --- |
| [viam-ufactory-xarm](https://github.com/viam-modules/viam-ufactory-xarm) | Go | Hand-rolled Modbus-style TCP :502, servo setpoints streamed at `move_hz` (100 Hz) | Motion service dependency (`motion` attr, `NewPoseInFrame("<name>_origin")`) | Trapezoidal interpolation or trajex TOTG, one setpoint per tick | Motion state 3 then restart / `opMgr.OpRunning()` | Embedded SVA JSON + opt-in URDF and STL meshes, GLBs for `Get3DModels` |
| [viam-dobot](https://github.com/viam-modules/viam-dobot) | Go | ASCII dashboard :29999 (`MovJ(joint={...})`) + 1440-byte binary feedback :30004 at 125 Hz | In-process `armplanning.MoveArm` | One controller `MovJ` per waypoint | Dashboard `Stop` + `opMgr.CancelRunning` / feedback `robot_mode` | Embedded SVA + opt-in URDF; `cr10a-simulated` twin |
| [universal-robots](https://github.com/viam-modules/universal-robots) | C++20 | URCL library (RTDE, dashboard, URScript) at `robot_control_freq_hz` | Controller-hosted Cartesian move | trajex TOTG, PV/PVA points streamed; also streamed API | Cancel future / move-slot flag | SVA synthesized at runtime from controller-calibrated DH |
| [yaskawa-robots](https://github.com/viam-modules/yaskawa-robots) | C++20 | Custom MotoPlus firmware on the YRC, raw boost::asio TCP | Unimplemented; Cartesian goals go through the motion service | trajex TOTG | Socket command / state | SVA JSON per model, limits patched in |
| [viam-franka-arm](https://github.com/viam-modules/viam-franka-arm) | Go + cgo | libfranka 1 kHz loop via C shim | Motion service | 1 kHz control callback | Best-effort / atomic flag | URDF only |
| [so-101](https://github.com/viam-devrel/so-101) | Go | Feetech serial bus | Motion service with approach-cone goal (5 DoF) | Waypoint lookahead over the servo bus | Bus stop / atomic flag | Embedded SVA + URDF, embedded GLBs |
| [rebot-b601](https://github.com/viam-devrel/rebot-b601) | Python | CAN via `motorbridge` | Motion service | Own trajectory code on a thread | Thread event / velocity poll | URDF decorated at request time |

No public Viam module for Fairino, Kinova, Techman, ABB, or Rainbow Robotics exists under viam-modules, viam-labs, viam-devrel, or viam-soleng, and the registry search for "rainbow" is empty. Fairino was checked under about thirty name variants; if one exists it is private.

The patterns that repeat across the Go modules, and that this module copies:

- **Registration**: `resource.NewModel(ns, family, model)`, one `resource.RegisterComponent(arm.API, model, resource.Registration[arm.Arm, *Config]{Constructor})` per model, `module.ModularMain(resource.APIModel{...})`. Modular resources are always rebuilt on reconfigure, so all connection setup lives in the constructor and `Close` must be idempotent.
- **Config**: `host` required, `speed_degs_per_sec` and `acceleration_degs_per_sec_per_sec` optional with defaults, `motion` as an optional dependency, `use_urdf` plus `mesh_decimation_ratios` for mesh collision geometry.
- **Kinematics**: SVA JSON embedded with `//go:embed`, loaded with `referenceframe.UnmarshalModelJSON`, velocity and acceleration limits written in with `referenceframe.SetJointLimits` so `GetKinematics` clients and the planner see them.
- **Motion**: `arm.CheckDesiredJointPositions` before every joint move; `operation.SingleOperationManager` around moves, `CancelRunning` in `Stop`, `OpRunning()` as the `IsMoving` fallback; a background goroutine polling controller state with reconnect and backoff (`goutils.ManagedGo`, `SelectContextOrWait`).
- **DoCommand**: key-presence dispatch (`{"set_speed": 30}`), with `set_speed`, `set_acceleration`, `clear_error`, `get_state` as the common vocabulary.
- **Packaging**: `Makefile` target producing `bin/module.tar.gz` (binary + `meta.json` + data files), `meta.json` with a `build` block for `viam module build`, `deploy.yml` on release via `viamrobotics/build-action@v1`, arch `linux/amd64`, `linux/arm64`, `darwin/arm64` with `darwin_deps: [nlopt-static, go]` because RDK motion planning links nlopt through cgo.
- **Simulated twin** in the same module (`cr10a-simulated`, `so101:simulated`) so the motion stack can be exercised without hardware.

The RDK `arm.Arm` interface a module must satisfy today (RDK v1.9.0 line; `referenceframe.Input` is a `float64` in radians or mm):

```go
type Arm interface {
	resource.Resource            // Name, DoCommand, Status, Close
	resource.Shaped              // Geometries
	resource.Actuator            // IsMoving, Stop
	framesystem.InputEnabled     // Kinematics, CurrentInputs, GoToInputs
	EndPosition(ctx, extra) (spatialmath.Pose, error)
	MoveToPosition(ctx, pose, extra) error
	MoveToJointPositions(ctx, positions []referenceframe.Input, extra) error
	MoveThroughJointPositions(ctx, positions [][]referenceframe.Input, options *MoveOptions, extra) error
	MoveThroughJointPositionsStreamed(ctx, batches <-chan []TrajectoryPoint, responses chan<- Response, extra) error
	JointPositions(ctx, extra) ([]referenceframe.Input, error)
	Get3DModels(ctx, extra) (map[string]*commonpb.Mesh, error)
	Properties(ctx, extra) (Properties, error)   // SupportManualMode, SupportCartesianCommands
	SetManualMode(ctx, manualMode bool, enabledFor time.Duration, extra) error
	ManualMode(ctx, extra) (bool, error)
}
```

`MoveOptions` carries `MaxVelRads`, `MaxAccRads`, per-joint `MaxVelRadsJoints` and `MaxAccRadsJoints`, and `MaxTCPSpeedMPerSec`. `TrajectoryPoint` carries `Time` (zero for the first point, strictly increasing), `Positions`, and optional `Constraints{Velocities, Accelerations}`. The server prepends `operation.CancelOtherWithLabel` to every move, so a new move cancels the context of the one in flight.

## The RB10-1300 and the Rainbow Robotics control interface

The control box is a TCP server at a default IP of 10.0.2.7 with two always-open ports: 5000 takes newline-terminated ASCII script commands and answers with ASCII lines; 5001 answers the ASCII request `reqdata` with one 580-byte little-endian binary `systemSTAT` packet. The official SDK, [rbpodo](https://github.com/RainbowRobotics/rbpodo) (Apache-2.0, C++17 with pybind11 Python bindings, v0.16.14, last commit July 2026), is a thin wrapper over exactly this and documents the wire format in its headers. There is no checksum, no handshake, no authentication in the SDK, and no binary command format.

### Arm facts that shape the design

| Item | Value | Source |
| --- | --- | --- |
| Payload / reach / DoF | 10 kg / 1300 mm / 6 | [System specifications](https://rainbowrobotics.github.io/rb_cobot_docs/manual/appendix/system_specifications) |
| Joint range | ±360° all joints, elbow (J3) ±165° | same |
| Max joint speed / TCP speed | 180°/s / 1 m/s | same (one distributor lists J1–J2 at 120°/s; **verify on hardware**) |
| Repeatability / weight | ±0.05 mm / 37.1 kg | same |
| Tool flange I/O | E series: DI 6, DO 2, RS-485, 12/24 V at 2 A | same |
| Control box I/O | DI 16, DO 16, AI 4, AO 4 | [Control box](https://rainbowrobotics.github.io/rb_cobot_docs/manual/product_introduction/control_box) |
| Other protocols | Modbus TCP :502 (read state, no setpoint streaming), EtherNet/IP, PROFINET | [Modbus map](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/modbus_address_map) |
| Control cycle | 10 ms default; 1.8 ms with "High-speed External Script Mode" on the pendant | [Socket/serial setup](https://rainbowrobotics.github.io/rb_cobot_docs/manual/setup/socket_serial) |
| Units on the wire | degrees, mm, deg/s, mm/s; Cartesian orientation as Euler Z-Y'-X'' (rx, ry, rz) | [UI Script reference](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/ui_script) |

### Command port 5000

Client sends one script string per line, e.g. `move_j(jnt[0, 45, 90, 0, 90, 0], 60, 80)`. The box replies with the literal line `The command was executed` as the acknowledgement, and separately broadcasts event lines to every connected client: `info[motion_changed][<n≠0>]` when a move starts, `info[motion_changed][0]` when it ends, `error[code][<int>]` on faults, `info[<tag>][done]` after activation. A first command that is not a recognised script keyword returns `The command is not allowed`. Queries are done by printing a system variable with a random tag: send `print(SD_ROBOT_STATE,"#ab12c_[", "]")`, receive `info[print_...][#ab12c_[3]]`.

The commands the module needs, with units:

| Purpose | Script sent | Notes |
| --- | --- | --- |
| Activate (power + servo on) | `mc jall init` | wait for `info[*][done]`; `init_state_info` in state reaches 6 |
| Real vs simulation | `pgmode real` / `pgmode simulation` | boot default is simulation; `real` needs an activated arm |
| Global speed scale | `set_speed_bar(0.5)` | 0–1 |
| Joint move | `move_j(jnt[j0..j5], v, a)` | deg, deg/s, deg/s²; non-blocking |
| Blended joint batch | `move_jb2_clear()`, `move_jb2_add(jnt[...], v, a, 0, blend)`, `move_jb2_run()` | controller-side blending through waypoints |
| Streaming setpoint | `move_servo_j(jnt[...], t1, t2, gain, alpha)` | t1 ≥ 0.002 s to reach, t2 hold 0.02–0.2 s; resend before t1+t2 expires |
| Streaming velocity | `move_speed_j(jnt[dj0..dj5], t1, t2, gain, alpha)` | deg/s; all zeros ends a servo stream |
| Halt motion | `task stop` | what Rainbow's own ROS 2 driver uses; `task pause` / `task resume_a` also exist |
| Freedrive | `freedrive_teach_on()` / `freedrive_teach_off()` | real mode only |
| Collision | \`set\_collision\_onoff(0 | 1)` ,  `set\_collision\_th(0..1)` ,  `set\_collision\_mode(0 |
| TCP and payload | `set_tcp_info(x,y,z,rx,ry,rz)`, `set_payload_info(w,cx,cy,cz)` | resets when a pendant program ends |
| Tool and box I/O | `tool_out V,s0,s1`, \`set\_box\_dout(port, -1 | 0 |
| FK on the box | `print(calc_fk_tcp(j0..j5), "#tag_[", "]")` | optional cross-check of the SVA model |
| Power off | `AvadaKedavra()` (rbpodo `shutdown`) or `arm_powerdown()` | **verify on hardware** which is safe to call from a driver |

### Data port 5001

Send `reqdata`, read one packet: header `0x24`, size low, size high, `0x03`, then the struct. Rainbow's ROS 2 driver polls at 500 Hz. The fields the module uses: `jnt_ref[6]` (commanded joint angle, deg; the field the ROS 2 driver publishes as position, since `jnt_ang` freezes in simulation mode), `jnt_ang[6]` (encoder, deg), `tcp_ref[6]` (mm, deg), `robot_state` (1 idle, 3 moving, 5 idle under conveyor or force control, 60+ under MovePB/ITPL/Pro, 560+ under MoveXB), `task_state` (1 idle, 3 running, 2 paused), `real_vs_simulation_mode` (1 sim), `init_state_info` (6 = activated), `init_error`, `is_freedrive_mode`, `collision_detect_onoff`, `op_stat_collision_occur`, `op_stat_soft_estop_occur`, `op_stat_ems_flag`, `information_chunk_1` (bit 3 and 5 e-stop, bit 6 arm power), `jnt_info[6]` (per-joint fault bits), `jnt_temperature[6]`, `jnt_cur[6]`, digital and analog I/O arrays, and `eft_*` force-torque. The full 580-byte layout with offsets is in rbpodo's `data_type.hpp` and the official [data structure page](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/data_structure); it has only ever grown by appending (464 bytes in the 2020 ROS 1 driver, 580 now), so the decoder must frame on the size header and tolerate longer packets.

### Kinematics from Rainbow's own files

The official [DH parameters](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/dh_parameters) for RB10-1300 are d1 197.0, d2 187.5, d3 148.4, d4 117.15, d5 117.15, d6 115.3, a1 612.7, a2 570.15 mm, and they match the URDF in [rbpodo\_ros2](https://github.com/RainbowRobotics/rbpodo_ros2) (`rbpodo_description/robots/rb10_1300e/`), which also ships DAE visual and STL collision meshes per link under Apache-2.0. Zero pose is the arm pointing straight up. Joint origins from that URDF, in metres:

| Joint | Parent to child | Origin xyz | Axis |
| --- | --- | --- | --- |
| base | link0 → link1 | 0, 0, 0 | 0 0 1 |
| shoulder | link1 → link2 | 0, 0, 0.197 | 0 1 0 |
| elbow | link2 → link3 | 0, −0.1875, 0.6127 | 0 1 0 |
| wrist1 | link3 → link4 | 0, 0.1484, 0.57015 | 0 1 0 |
| wrist2 | link4 → link5 | 0, −0.11715, 0 | 0 0 1 |
| wrist3 | link5 → link6 | 0, 0, 0.11715 | 0 1 0 |
| tcp (fixed) | link6 → tcp | 0, −0.1153, 0 | — |

The `rb10_1300e_u` variant differs only at wrist1 (y = 0.1514). Which variant Brad received is an open question for the hardware bring-up.

### Caveats read from the sources

- rbpodo is documented as "under development and subject to change" and "not thread-safe"; the ROS 2 driver says "DO NOT USE in production". The wire protocol underneath them has been stable since 2020.
- Events and errors are broadcast to all connected clients, and the pendant is one of them; there is no motion ownership arbitration.
- rbpodo does a single 1 KB `recv` with no framing on either port; the Go client must reassemble lines and length-framed packets properly.
- The docs mention a "prescribed key and grammar" gate on ports 5000/5001 that no client sends; the only observable gate is the first-command filter. Test on the real box early.
- No Go client for the RB series exists anywhere public; the 2021 Python `rb-api` and 2020 C++ `rb5-ros-driver` are the only community raw-socket clients.

## Language decision: Go

Go, pure, with no vendor SDK and no cgo in the protocol layer. The decision holds because the RB protocol is line-oriented ASCII plus one fixed-layout little-endian struct, which is roughly 400 lines of `net`, `bufio`, `fmt`, and `encoding/binary`; the Dobot and UFactory modules prove this path inside Viam, and Rainbow's own docs say the client language is irrelevant as long as the string grammar matches.

| Option | What it would cost | Why it loses |
| --- | --- | --- |
| **Go, native protocol (chosen)** | Write the command formatter, line parser, ACK/event correlator, and 580-byte state decoder ourselves (\~1 day) | Full RDK reuse: `referenceframe` for SVA parsing and limits, `arm.CheckDesiredJointPositions`, `operation.SingleOperationManager`, `armplanning.MoveArm`, `motion.FromProvider`; single static binary; API surface always current |
| Python over `rbpodo` | Fast first move (`pip install rbpodo`), but wheels are x86\_64 only (ARM must build from source), rbpodo is not thread-safe so every call needs `asyncio.to_thread`, no `referenceframe` so joint-limit checks and FK come from hand-written code, venv bootstrap on the target at first run | Every Python arm module in the survey re-implements what RDK gives Go for free, and the rebot-b601 module needed a servicer shim until SDK 0.83 |
| C++ over `rbpodo` + viam-cpp-sdk | Conan or Docker builds, SDK pinned in four places, clang-tidy gates, builder images per arch, no RDK helpers | Justified for UR (URCL) and Yaskawa (MotoPlus firmware) where a native library was unavoidable; nothing in the RB protocol needs it |

What Go commits us to:

- Importing RDK motion planning pulls in nlopt through cgo, so `meta.json` needs `darwin_deps: [nlopt-static, go]` and Windows builds use `-tags no_cgo`. This is the same as every other Go arm module.
- Trajectory time-parameterization (trajex TOTG) is available in Go only with cgo and prebuilt static archives; the plan below avoids needing it in v1 by letting the controller blend joint batches, and by executing already-timed points in the streamed API.
- `rbpodo` remains the reference implementation for reading the wire format and for cross-checking behaviour from a laptop, but it is never a runtime dependency.

## Module architecture and repository layout

Two packages: `rb`, a protocol client with no Viam imports that owns both sockets, and `arm`, the Viam resource that owns the move slot and the snapshot. Keeping the protocol free of RDK types makes it testable in isolation and reusable for a gripper model later.

&#91;embedded content: module architecture · 2 goroutines, 2 sockets\]

The arm resource sends script lines through the command client and reads everything it needs from the snapshot, which only the data poller writes; the command reader goroutine turns ACK and event lines into channel messages that the in-flight move waits on.

```text
rainbow-robotics/
├── cmd/module/main.go            # module.ModularMain with both models
├── arm/
│   ├── rb10.go                   # Config, Validate, constructor, Close, simple API methods
│   ├── move.go                   # move_j, move_jb2 batch, streamed executor, completion wait
│   ├── docommand.go              # key-presence dispatch
│   ├── kinematics.go             # go:embed SVA, SetJointLimits, URDF path, GLB loader
│   ├── simulated.go              # rb10-1300-simulated (no sockets)
│   ├── rb10_1300_kinematics.json
│   ├── rb10_1300e.urdf, meshes/, 3d_models/
│   └── *_test.go                 # driven against rb/fakebox
├── rb/                           # protocol client, zero Viam imports
│   ├── client.go                 # Dial(host, cmdPort, dataPort), reconnect, Close
│   ├── command.go                # script formatter, Send, SendNoAck, waiters
│   ├── parser.go                 # ACK / info / warn / error / unknown line classifier
│   ├── state.go                  # SystemState struct, 580-byte LE decoder, framing
│   ├── errors.go                 # error code table from rbpodo res/error_code.csv
│   └── fakebox/                  # in-process fake control box for tests
├── tools/                        # DAE to GLB conversion, golden packet capture
├── Makefile  meta.json  go.mod  CLAUDE.md  README.md  CHANGELOG.md  THIRD_PARTY_NOTICES
└── .github/workflows/{test,deploy}.yml
```

Concurrency rules, stated once so every file follows them:

- The data poller goroutine is the only writer of the snapshot; readers take an `RLock` and copy. Snapshot carries the receive time so callers can reject stale data.
- The command connection has one writer (serialized by a mutex) and one reader goroutine. The reader classifies lines and delivers them to whichever waiter registered for them (ACK, `motion_changed`, a print tag); unclaimed lines are logged at debug level.
- One `operation.SingleOperationManager` guards all moves; `Stop` cancels it. A move's completion wait polls the snapshot rather than trusting a single event line, because events are broadcast and can be consumed by the pendant.
- Both connections reconnect with 1 s backoff on any read or write error; after a reconnect the module re-checks activation, mode, and payload before allowing moves.
- `Close` cancels a module-level context that every goroutine selects on, then waits with a 5 s bound.

## Configuration schema

Only `host` is required; everything else defaults to values that move the arm slowly in whatever mode the pendant already has it in. Attribute names follow the UFactory module so users see the same vocabulary across Viam arms.

| Attribute | Type | Default | Validation and meaning |
| --- | --- | --- | --- |
| `host` | string | required | Control box IP (Rainbow's default is 10.0.2.7). |
| `command_port` | int | 5000 | 1–65535. |
| `data_port` | int | 5001 | 1–65535. |
| `speed_degs_per_sec` | number | 60 | 0 < v ≤ 180. Joint speed for `move_j` and batch moves; also published as `max_velocity` in the kinematics. |
| `acceleration_degs_per_sec_per_sec` | number | 200 | > 0. Published as `max_acceleration`. Rainbow publishes no ceiling; **verify on hardware**. |
| `max_speed_degs_per_sec`, `max_acceleration_degs_per_sec_per_sec` | number | 180 / unset | Ceilings that `set_speed`, `set_acceleration` and `MoveOptions` are clamped to. |
| `operation_mode` | string | `real` | `real` or `simulation`. Sent as `pgmode` after activation. `simulation` moves only the controller's reference and is the safe bring-up default. |
| `activate_on_start` | bool | true | Send `mc jall init` if `init_state_info` ≠ 6 and wait for it to reach 6. |
| `speed_bar` | number | unset | 0–1. When set, sent as `set_speed_bar` at start; when unset the pendant's value is left alone. |
| `state_poll_hz` | int | 100 | 10–500. Rate of `reqdata` polling on port 5001. |
| `move_hz` | int | 100 | 10–500. Setpoint rate for `move_servo_j` streaming. Above 100 needs the pendant's high-speed external script mode. |
| `servo_gain`, `servo_alpha`, `servo_hold_secs` | number | 0.5 / 0.5 / 0.03 | Passed through to `move_servo_j` as `gain`, `alpha`, `t2`; these are the values Rainbow's ROS 2 driver uses. |
| `joint_batch_blend` | number | 0.5 | Blend value for `move_jb2_add`. Semantics of the blend argument to be confirmed against the UI Script reference. |
| `reject_move_request_threshold_deg` | number | 10 | A streamed or batched move whose first waypoint is further than this (L∞) from the measured position is rejected. |
| `collision_detection` | bool | unset | When set, sent as `set_collision_onoff`. |
| `collision_sensitivity` | number | unset | 0–1, sent as `set_collision_th`. |
| `payload_kg`, `payload_cog_mm` | number, \[x,y,z\] | unset | Sent as `set_payload_info`. Rainbow resets this when a pendant program ends, so the module re-sends it after every reconnect. |
| `motion` | string | `builtin` | Name of the motion service used by `MoveToPosition`; declared as an optional dependency. If it does not resolve, the module falls back to in-process `armplanning.MoveArm`. |
| `use_urdf` | bool | false | Serve the URDF with STL collision meshes from `$VIAM_MODULE_ROOT/arm/rb10_1300e.urdf` instead of the SVA JSON. |
| `mesh_decimation_ratios` | \[number\] | unset | Per-link decimation applied when loading the URDF meshes. |
| `command_timeout_secs` | number | 2 | How long to wait for `The command was executed` before failing a command. |
| `power_off_on_close` | bool | false | Send `AvadaKedavra()` in `Close`. Off by default so a viam-server restart does not drop the arm. |

A minimal config:

```json
{
  "name": "rb10",
  "api": "rdk:component:arm",
  "model": "viam-labs:rainbow-robotics:rb10-1300",
  "attributes": {
    "host": "10.0.2.7",
    "speed_degs_per_sec": 30,
    "operation_mode": "simulation"
  }
}
```

`Validate(path string) ([]string, []string, error)` returns `host` missing as `resource.NewConfigValidationFieldRequiredError`, range errors as `resource.NewConfigValidationError`, and the `motion` name in the optional-dependency slice. The simulated model `viam-labs:rainbow-robotics:rb10-1300-simulated` accepts only `speed_degs_per_sec`, `acceleration_degs_per_sec_per_sec`, `simulate_time`, `use_urdf`, and `mesh_decimation_ratios`, mirroring the Dobot module's simulated twin.

## Arm API implementation

Every method reads from one mutex-protected state snapshot fed by the port-5001 poller, and every move goes through one `operation.SingleOperationManager` slot; joint values cross the boundary as radians inside and degrees on the wire.

| Viam method | Implementation |
| --- | --- |
| `JointPositions`, `CurrentInputs` | `jnt_ref[6]` from the latest snapshot, degrees to radians. Error if no snapshot yet or the snapshot is older than 500 ms ("joint positions not currently known"). |
| `EndPosition` | `model.Transform(inputs)` on the SVA model, so the pose the planner sees and the pose reported agree by construction. The controller's `tcp_ref` is exposed only through DoCommand `get_state` for cross-checking. |
| `MoveToJointPositions` | `arm.CheckDesiredJointPositions`; `ensureControllable()` (activated, not e-stopped, not in freedrive, mode matches config); `opMgr.New(ctx)`; send `move_j(jnt[...], v, a)` with `v`, `a` from config (or `extra["speed_degs_per_sec"]`, `extra["acceleration_degs_per_sec_per_sec"]`, clamped); wait for ACK; wait up to 1 s for `robot_state` to become 3; then poll until `robot_state` returns to 1; finally check every joint is within `0.5°` of target, else return "stopped before reaching target". `ctx.Done()` at any point sends `task stop`. |
| `MoveThroughJointPositions` | Check every waypoint; dedupe consecutive duplicates (L∞ < 1e-3 rad); reject if the first waypoint is further than `reject_move_request_threshold_deg` from the measured position; clamp `MoveOptions.MaxVelRads`/`MaxAccRads` (and per-joint arrays, taking the minimum) against config ceilings; send `move_jb2_clear()`, one `move_jb2_add(jnt[...], v, a, 0, blend)` per waypoint, `move_jb2_run()`; then the same start/finish wait as above with the last waypoint as target. If the batch is rejected for size (limit unknown, **verify on hardware**), split into chunks and run them back to back. |
| `MoveThroughJointPositionsStreamed` | Real-time executor. Disable ACK waiting on the command connection (a per-connection flag; ACK lines are drained by the reader). Record `t0` at the first point; for each point in each batch, sleep until `t0 + point.Time`, send `move_servo_j(jnt[...], t1, t2, gain, alpha)` with `t1 = max(0.002, next.Time - point.Time)`. Write one `arm.Response{}` per batch after its last point is sent. When `batches` closes, send `move_speed_j(jnt[0,0,0,0,0,0], ...)`, re-enable ACK waiting with a `print("barrier","","")` round trip, wait for `robot_state` 1, return. A first point further than the reject threshold from the current position is refused before anything is sent. Velocities and accelerations in `Constraints` are ignored in v1 (the controller tracks position setpoints). |
| `MoveToPosition` | `motion.Move(ctx, MoveReq{ComponentName, Destination: NewPoseInFrame("<name>_origin", pose)})` when the `motion` dependency resolved, else `armplanning.MoveArm(ctx, logger, a, pose)`; both end up in `MoveThroughJointPositions`. `extra["linear"]: true` may later route to the controller's `move_l`; not in v1. |
| `GoToInputs` | `MoveThroughJointPositions(ctx, inputs, nil, nil)`. |
| `Stop` | `opMgr.CancelRunning(ctx)`; if a stream is active, send `move_speed_j` zeros; send `task stop`; wait up to 2 s for `robot_state` 1. Returns nil if the arm was already idle. |
| `IsMoving` | \`robot\_state == 3 |
| `Kinematics` | Embedded SVA JSON parsed once in the constructor; `referenceframe.SetJointLimits` writes `max_velocity` = `speed_degs_per_sec` and `max_acceleration` from config into every joint so `GetKinematics` bytes and `TrajectoryLimits` carry them. With `use_urdf`, `referenceframe.ParseModelXMLFile($VIAM_MODULE_ROOT/arm/rb10_1300e.urdf, name, ratios)`. |
| `Geometries` | `model.Geometries(inputs).Geometries()`. |
| `Get3DModels` | One GLB per link from `$VIAM_MODULE_ROOT/arm/3d_models/rb10-1300/link{0..6}.glb`, content type `model/gltf-binary`, keyed by link name as in the SVA file. |
| `Properties` | `{SupportManualMode: true, SupportCartesianCommands: true}`. |
| `SetManualMode` | `freedrive_teach_on()` / `freedrive_teach_off()`; refuses in simulation mode (controller error 204). A non-zero `enabledFor` starts a timer goroutine that turns freedrive off. Moves are refused while `is_freedrive_mode` is 1. |
| `ManualMode` | `is_freedrive_mode == 1` from the snapshot. |
| `Status` | Same map as DoCommand `get_state`. |
| `DoCommand` | See the extras section. Unknown key returns an error naming the key. |
| `Close` | Cancel the module context, stop any stream and running operation, `task stop` if moving, optionally `AvadaKedavra()`, close both sockets, wait for goroutines with a 5 s bound. Idempotent. |

Two details worth settling in code rather than debating: joint direction and offset conventions must be confirmed once against the URDF (the SVA file is built from it, so a sign error shows up as a mismatch between `model.Transform` and the box's `calc_fk_tcp`), and the completion check after a move must distinguish "reached target" from "controller went idle after a collision stop" by comparing joints to the target, exactly as the RDK simulated arm distinguishes done from stopped.

## Kinematics model, frame system and motion planning

The module ships `arm/rb10_1300_kinematics.json`, an SVA file written by hand from the rbpodo\_ros2 URDF, and validates it against the controller's own forward kinematics on first hardware contact. SVA is what every Viam-maintained arm serves and what `referenceframe.SetJointLimits` can patch; URDF stays as an opt-in for mesh collision geometry.

The SVA file, in outline (translations in mm, one link per URDF joint origin, a fixed `tcp` link at the end):

```json
{
  "name": "rb10-1300",
  "kinematic_param_type": "SVA",
  "links": [
    {"id": "base_link",  "parent": "world",   "translation": {"x": 0, "y": 0, "z": 0}},
    {"id": "link1",      "parent": "base",    "translation": {"x": 0, "y": 0,       "z": 197.0},  "geometry": {"...": "capsule"}},
    {"id": "link2",      "parent": "shoulder","translation": {"x": 0, "y": -187.5,  "z": 612.7}},
    {"id": "link3",      "parent": "elbow",   "translation": {"x": 0, "y": 148.4,   "z": 570.15}},
    {"id": "link4",      "parent": "wrist1",  "translation": {"x": 0, "y": -117.15, "z": 0}},
    {"id": "link5",      "parent": "wrist2",  "translation": {"x": 0, "y": 0,       "z": 117.15}},
    {"id": "tcp",        "parent": "wrist3",  "translation": {"x": 0, "y": -115.3,  "z": 0}}
  ],
  "joints": [
    {"id": "base",     "type": "revolute", "parent": "base_link", "axis": {"x": 0, "y": 0, "z": 1}, "min": -360, "max": 360},
    {"id": "shoulder", "type": "revolute", "parent": "link1",     "axis": {"x": 0, "y": 1, "z": 0}, "min": -360, "max": 360},
    {"id": "elbow",    "type": "revolute", "parent": "link2",     "axis": {"x": 0, "y": 1, "z": 0}, "min": -165, "max": 165},
    {"id": "wrist1",   "type": "revolute", "parent": "link3",     "axis": {"x": 0, "y": 1, "z": 0}, "min": -360, "max": 360},
    {"id": "wrist2",   "type": "revolute", "parent": "link4",     "axis": {"x": 0, "y": 0, "z": 1}, "min": -360, "max": 360},
    {"id": "wrist3",   "type": "revolute", "parent": "link5",     "axis": {"x": 0, "y": 1, "z": 0}, "min": -360, "max": 360}
  ]
}
```

Rules for building and checking it:

- Link geometry: one capsule per link sized from the STL collision mesh bounding box in `rbpodo_ros2/rbpodo_description/meshes/rb10_1300e/collision/`, with the capsule axis along the link's long dimension. Follow the shape of `rdk/components/arm/kinematics/ur5e.json` for the geometry block syntax.
- Joint limits are the mechanical ranges (±360°, elbow ±165°). Speed and acceleration limits are not in the file; the constructor writes them from config with `referenceframe.SetJointLimits`, which also re-marshals the bytes returned by `GetKinematics`.
- Joint sign conventions: the URDF axes are all positive-y or positive-z, and the controller reports angles in the same convention its URDF uses, so no sign flips are expected. The proof is the FK cross-check below, run before anything else on hardware.
- The model name is `rb10-1300`; RDK names the arm's frame after the component, and the motion service targets `<name>_origin`.
- Zero pose is the arm straight up. The planner needs no home pose, but the bring-up checklist uses `[0, 0, 0, 0, 0, 0]` and `[0, -30, 90, 0, 60, 0]` (elbow bent, clear of the base) as its two reference poses.

FK cross-check test (run with the arm in simulation mode, TCP offset on the controller set to zero): for 50 random joint vectors inside the limits, `move_j` to each, read `tcp_ref`, convert Rainbow's Euler Z-Y'-X'' (rx, ry, rz in degrees) to a `spatialmath.Pose`, and compare with `model.Transform(inputs)`. Pass is position within 1 mm and orientation within 0.5°. The same loop can use `print(calc_fk_tcp(...))` without moving. This test is the single most valuable thing to run first, because every later feature depends on the model being right.

URDF option: `arm/rb10_1300e.urdf` plus `arm/meshes/rb10_1300e/collision/link{0..6}.stl` are copied from rbpodo\_ros2 (Apache-2.0, attribution kept in a `THIRD_PARTY_NOTICES` file), with `package://` mesh paths rewritten to relative paths so `referenceframe.ParseModelXMLFile` can load them. The 3D viewer GLBs under `arm/3d_models/rb10-1300/` are converted once from the DAE visual meshes (`assimp` or Blender in a script under `tools/`), one file per link, and are not collision geometry.

Motion planning integration needs nothing special once the SVA file is correct: `armplanning.MoveArm` and the builtin motion service plan against `Kinematics()` and execute through `MoveThroughJointPositions`; the streaming motion service uses `TrajectoryLimits` from the published joint limits and drives `MoveThroughJointPositionsStreamed` with pre-timed PVA points. That is why `max_velocity` and `max_acceleration` must be present on every joint: RDK treats them as all-or-nothing.

## Safety, state management, error handling and reconnection

The module refuses to command motion unless it has just observed, in its own snapshot, that the arm is activated, in the configured mode, not e-stopped, not collision-stopped, and not in freedrive. That check runs before every move and is the whole safety story; the states below are how it is tracked.

&#91;embedded content: module state machine · 6 states\]

Manual mode is a flag on `ready`: moves are refused while `is_freedrive_mode` is 1, and `SetManualMode(false)` or the `enabledFor` timer clears it.

Rules:

- **`ensureControllable()`** reads one snapshot and requires: age under 500 ms; `init_state_info == 6`; `real_vs_simulation_mode` matching `operation_mode`; `information_chunk_1` bits 3 and 5 (e-stop) clear; `op_stat_collision_occur`, `op_stat_soft_estop_occur`, `op_stat_ems_flag` zero; `is_freedrive_mode == 0`; no `jnt_info` fault bits (JAM, FLT, BIG, TMP). A failed check returns an error naming the first failing condition.
- **Faults observed mid-move**: the completion wait watches the same bits every poll. When one trips, the move returns `stopped before reaching target: <reason>` and the state goes to `faulted`; the module never auto-clears a fault.
- **Stop is two-layered**: `opMgr.CancelRunning` unblocks the Go side immediately and `task stop` halts the controller; the caller returns only after `robot_state` is 1 or 2 s have passed.
- **Context cancellation** is honoured at every wait: a cancelled `MoveToJointPositions` sends `task stop` on its way out, so a client timeout never leaves the arm in motion. The RDK server already cancels the previous move when a new one arrives.
- **Errors from the box** (`error[code][N]`) are mapped through the table copied from rbpodo's `res/error_code.csv` (about 600 entries, Korean and English) so log lines say "186: real mode requires an activated arm" rather than a number.
- **Command timeouts**: an ACK not seen within `command_timeout_secs` fails the command and marks the command connection suspect; the next failure reconnects it.
- **Reconnection**: each socket has its own goroutine that re-dials with 1 s backoff on any error, logging every 30th attempt. After reconnecting the module re-runs the startup sequence (check activation, send `pgmode`, resend `speed_bar`, payload and collision settings) before reporting `ready`. A move in flight during a disconnect fails with "connection to arm lost".
- **Broadcast events from other clients** (the pendant, a second script client) are logged at info level and never acted on directly; only the snapshot drives state transitions.
- **Streaming safety**: if the executor falls more than one `t1 + t2` behind schedule (GC pause, network stall) it sends `move_speed_j` zeros and returns an error rather than letting the controller finish the last setpoint on its own; the controller's own timeout on `move_servo_j` is the backstop.
- **Close never powers the arm down** unless `power_off_on_close` is set, so a viam-server restart does not drop the brakes on a loaded arm.
- **Logging**: every script line sent and every non-ACK line received at debug level; state transitions and refused moves at info; faults at warn.

## DoCommand extras and stretch goals

DoCommand uses key-presence dispatch like the UFactory and UR modules (`{"set_speed": 30}`), and v1 ships only the commands that operate the arm safely from the Viam app; I/O, gripper and raw-script access follow once the core is solid.

| Key | Argument | Effect | Version |
| --- | --- | --- | --- |
| `set_speed` | deg/s | Session joint speed, clamped to `max_speed_degs_per_sec`; returns the effective value. Reset on reconfigure. | v1 |
| `set_acceleration` | deg/s² | Same for acceleration. | v1 |
| `get_speed`, `get_acceleration` | — | Current session values. | v1 |
| `get_state` | — | Decoded snapshot: joints (deg), `tcp_ref`, `robot_state`, `task_state`, mode, `init_state_info`, e-stop and power bits, collision flags, `jnt_info` bits, temperatures, currents, `eft_*`, snapshot age in ms. | v1 |
| `get_state_description` | — | One human string, e.g. `real / activated / idle / collision-stopped`. | v1 |
| `activate` | — | `mc jall init`, wait for `init_state_info` = 6. | v1 |
| `set_operation_mode` | `real` or `simulation` | `pgmode ...`; refused while moving. | v1 |
| `set_speed_bar` | 0–1 | `set_speed_bar(...)`. | v1 |
| `set_collision` | `{enabled, sensitivity, mode, after}` | The four `set_collision_*` calls. | v1 |
| `clear_error` | — | Recovery after a collision or soft e-stop. The exact recovery script is not documented in rbpodo; candidate is `task stop` followed by re-checking `op_stat_*` bits. **verify on hardware** | v1 |
| `set_payload` | `{kg, cog_mm}` | `set_payload_info`. | v1 |
| `power_off` | — | `AvadaKedavra()` after confirming idle. | v1 |
| `get_io` | — | Box and tool-flange digital and analog inputs and outputs from the snapshot. | v1.1 |
| `set_box_dout`, `set_box_aout`, `set_tool_out` | port, value | Pass-through to the matching scripts. | v1.1 |
| `get_ft` | — | `eft_*` force-torque if the arm has the sensor. | v1.1 |
| `calc_fk` | joints (deg) | `print(calc_fk_tcp(...))` from the box, for cross-checking the model. | v1.1 |
| `eval` | script string | Raw script pass-through, gated behind config `allow_raw_commands: true`. | v1.1 |

Stretch goals, each its own milestone after v1:

- **Gripper models** in the same module family (`viam-labs:rainbow-robotics:gripper`) over Rainbow's `gripper_macro` scripts or the tool-flange RS-485, following the UFactory pattern of a gripper resource that shares the arm's connection.
- **Controller-native linear moves**: `MoveToPosition` with `extra["linear"]: true` sending `move_l`, useful when the user wants a straight-line TCP path without the motion service.
- **In-module TOTG** through the trajex Go bindings (cgo, linux and darwin only), so `MoveThroughJointPositions` streams time-optimal `move_servo_j` setpoints instead of relying on `move_jb2` blending. Only worth it if hardware testing shows the controller's blending is too conservative or the batch size limit is small.
- **A discovery service** that finds control boxes on the 10.0.2.x subnet, matching the SO-101 module's `discovery` model.
- **Modbus TCP read-only fallback** on port 502 for state when port 5001 is unavailable; not needed unless a site firewall blocks 5001.

## Build, test, CI, meta.json and registry publishing

Copy the UFactory module's build shape: a `Makefile` whose `module` target produces `bin/module.tar.gz`, a `meta.json` with a `build` block so `viam module build` and the cloud build work, and two GitHub workflows, one that lints and tests every push inside the `rdk-devenv` container and one that publishes on release.

Toolchain and pins:

- Go 1.25 (what RDK v1.9.0 requires); `go.viam.com/rdk` pinned to the same tag `viam-ufactory-xarm` uses at the time of writing (v1.9.0), `go.viam.com/utils` and `go.viam.com/api` at whatever that RDK tag needs.
- Lint with `golangci-lint` using `etc/.golangci.yaml` copied from the UFactory module; `gofmt` and `go vet` gates in CI.
- Repo `github.com/viam-labs/rainbow-robotics`, module id `viam-labs:rainbow-robotics`, entrypoint `bin/rainbow-robotics`.

`meta.json`:

```json
{
  "module_id": "viam-labs:rainbow-robotics",
  "visibility": "public",
  "url": "https://github.com/viam-labs/rainbow-robotics",
  "description": "Go module for Rainbow Robotics RB-series collaborative arms",
  "models": [
    {"api": "rdk:component:arm", "model": "viam-labs:rainbow-robotics:rb10-1300", "markdown_link": "README.md#configure-the-arm"},
    {"api": "rdk:component:arm", "model": "viam-labs:rainbow-robotics:rb10-1300-simulated", "markdown_link": "README.md#simulated-arm"}
  ],
  "entrypoint": "bin/rainbow-robotics",
  "build": {
    "build": "make module",
    "path": "bin/module.tar.gz",
    "arch": ["linux/amd64", "linux/arm64", "darwin/arm64"],
    "darwin_deps": ["nlopt-static", "go"]
  }
}
```

The tarball holds `meta.json`, the binary, `arm/rb10_1300e.urdf`, `arm/meshes/`, `arm/3d_models/`, and `THIRD_PARTY_NOTICES`; the SVA JSON is embedded in the binary and needs no file. `make module` runs `GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) go build -o bin/rainbow-robotics ./cmd/module` and tars those paths, the same recipe as `viam-dobot`.

Workflows:

- `.github/workflows/test.yml`: on push and pull request, run `make lint test build` inside `ghcr.io/viamrobotics/rdk-devenv:amd64`.
- `.github/workflows/deploy.yml`: on a published GitHub release, `viamrobotics/build-action@v1` with `version: ${{ github.ref_name }}` builds every arch in `meta.json` and uploads to the registry. Tags are `vX.Y.Z`; release notes come from `CHANGELOG.md`.

Windows is out of scope for v1 (cgo and nlopt); it can be added later with `-tags no_cgo` the way the SO-101 module does it. A `CLAUDE.md` at the repo root records the conventions above plus the hardware-safety rules from the testing section, since the Dobot module showed that having Claude Code read those before touching `arm/` avoids most review churn.

## Testing plan

Three layers: unit tests on the pure protocol code with golden bytes, an in-process fake control box that speaks both ports so the whole arm can be driven in CI, and a short hardware checklist that goes simulation first, slow second, full speed last.

**Unit tests** (`rb/` package, no network):

- Command formatter: every script string in the command table, checked byte for byte, including `strconv.FormatFloat(v, 'f', -1, 64)` so no value is ever written in exponent notation.
- Line parser: `The command was executed`, `info[motion_changed][0]`, `error[code][186]`, nested brackets in `info[print_x][#ab12c_[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]]`, `The command is not allowed`, and a line split across two reads.
- State decoder: a 580-byte golden packet captured from rbpodo (or hand-built from the offset table) decodes to known field values; a 464-byte packet decodes the leading fields and zeros the rest; a longer packet is accepted; a packet with the wrong type byte is skipped.
- Unit conversions: degrees and radians round trips, Rainbow's Euler Z-Y'-X'' to `spatialmath.Orientation` and back, against values printed by rbpodo.

**Fake control box** (`rb/fakebox`, used by `arm` tests): a `net.Listen` server on two ephemeral ports that accepts script lines, replies with the ACK, updates an internal joint vector on `move_j`, `move_jb2_run` and `move_servo_j` at a configurable rate, emits `info[motion_changed]` events, serves `reqdata` with a real 580-byte packet, and can be told to raise an e-stop bit or a collision flag mid-move. This is the xArm test-server pattern extended to two ports. With it the `arm` package tests cover constructor and `Close`, every API method, `Stop` mid-move, context cancellation, reconnect after the fake drops the socket, and `MoveThroughJointPositionsStreamed` timing (points sent within 2 ms of their scheduled time at 100 Hz).

**Planning tests** against the SVA file, no hardware: parse the model, run `motionplan` on a few poses the way `xarm_test.go` does, and assert `Geometries` returns six capsules that do not self-intersect at the two reference poses.

**Hardware bring-up checklist**, in order, each step gating the next:

- [ ] Laptop on 10.0.2.x, `nc 10.0.2.7 5000` accepts `get_control_box_info()` and returns `info[controlbox][...]`; record the system version.
- [ ] Run rbpodo's `basic.py` once in simulation mode to confirm the box behaves as the docs say, and capture a `reqdata` packet as the golden bytes.
- [ ] Module in `operation_mode: simulation`: `JointPositions` matches the pendant, `MoveToJointPositions` to the two reference poses, `Stop` mid-move, `IsMoving` toggles.
- [ ] FK cross-check: 50 random poses, model versus `calc_fk_tcp`, within 1 mm and 0.5°.
- [ ] `MoveThroughJointPositions` with a 50-waypoint planner output in simulation; note whether `move_jb2` accepts it whole.
- [ ] Streamed moves in simulation at 50 Hz, then 100 Hz; watch `robot_state` never drops to 1 mid-stream.
- [ ] Switch to `real` with `speed_bar` 0.2, e-stop within reach, nothing inside the 1.3 m reach; repeat the reference poses and one `MoveToPosition` through the motion service.
- [ ] Collision test: obstruct gently with collision detection on; confirm the module reports "stopped before reaching target" and `clear_error` recovers.
- [ ] Raise `speed_bar` to 1.0 and `speed_degs_per_sec` to 90; repeat the planner test; measure setpoint-to-motion latency for the streamed path.
- [ ] Power-cycle the control box with viam-server running; confirm the module reconnects and re-activates without manual intervention.

Safety rules to write into `CLAUDE.md`: never default a new attribute to something that moves the arm faster; every new command path must be exercised in simulation mode first; a test that talks to a real box must be behind a `RB_HOST` environment variable and skipped otherwise.

## Milestones and order of work for Claude Code

Build bottom-up: the protocol client and its fake box first, because everything above them can then be tested in CI without the arm, and because the first hardware session should be spent confirming the wire format and the kinematics rather than debugging Viam plumbing.

&#91;embedded content: delivery roadmap · 5 phases, 4 gates\]

Phase 3 is the accent because it is where the module becomes useful: once the motion service can plan and execute a move on the real arm, everything after it is hardening.

Suggested first tasks, in order:

1. Scaffold the repo with `viam module generate` (Go, arm, `viam-labs:rainbow-robotics`), then replace the generated `Makefile`, `meta.json` and workflows with the shapes in the build section.
2. Write `rb/state.go` with the `SystemState` struct and decoder, and a test that decodes a hand-built 580-byte packet; get `binary.Size(SystemState{}) == 580` as the first assertion.
3. Write `rb/parser.go` and `rb/command.go` with the formatter table and the ACK/event waiter; table-driven tests from the command table in this doc.
4. Write `rb/fakebox` and `rb/client.go` together, so the client's reconnect and framing are tested against a server that drops connections and fragments packets.
5. Write `arm/rb10.go` with config, constructor, `Close`, `JointPositions`, `EndPosition`, `Kinematics`, `Geometries`, `Stop`, `IsMoving`, and the SVA file; wire `cmd/module/main.go`; run it against the fake box under a local viam-server.
6. Add `MoveToJointPositions` and the completion wait, then the simulated twin, then take it to the arm in simulation mode and run the FK cross-check.
7. `MoveThroughJointPositions`, `MoveToPosition`, `Get3DModels`, `Properties`, `SetManualMode`, DoCommand v1; test through the motion service.
8. `MoveThroughJointPositionsStreamed`, fault paths, reconnection, the bring-up checklist end to end.
9. README with the attribute table and DoCommand table, `CLAUDE.md`, `CHANGELOG.md`, tag `v0.1.0`.

Each numbered step is a pull request; steps 2 through 4 can land without touching anything Viam-specific.

## Open questions, risks and assumptions

Nothing here blocks starting the protocol layer and the fake box; every item resolves in the first hour on hardware or with one answer from Brad.

| Question or risk | Why it matters | How it gets resolved |
| --- | --- | --- |
| Which flange variant arrived: `rb10_1300e` or `rb10_1300e_u` | 3.0 mm difference at wrist1 in the kinematics | Read the model string from the pendant; FK cross-check catches a mismatch |
| Control box software version | `systemSTAT` layout has grown over time; the 580-byte layout is documented for 4.3.1 and current boxes run 8.x | `get_control_box_info()` on first contact; decoder frames on the size header regardless |
| Does `move_jb2` accept 50 to 200 waypoints | Determines whether the batch path needs chunking or the servo-stream fallback for planner output | Bring-up checklist step 5 |
| Meaning of the blend argument to `move_jb2_add` | Affects how closely the arm follows planner paths | UI Script reference on the pendant, then a test with a tight corner |
| The documented "prescribed key and grammar" gate on ports 5000/5001 | rbpodo sends no key and works; if a key is required on newer software the module needs a config attribute for it | First `nc` session in the checklist |
| Correct recovery script after a collision stop | `clear_error` semantics | Test with collision detection on; Rainbow error table (`res/error_code.csv` in rbpodo) as reference |
| `AvadaKedavra()` versus `arm_powerdown()` versus `shutdown` | Which one is safe to call from `Close` and `power_off` | Ask Rainbow support or test with the pendant watching |
| Per-joint speed ceilings | Official spec says 180°/s for all joints; one distributor lists 120°/s for J1 and J2 | Keep 180 as the default ceiling; make it per-joint configurable if the box rejects higher values |
| Streaming above 100 Hz | Needs the pendant's high-speed external script mode | Document it; `move_hz` default stays at 100 |
| Events broadcast to every client, no ownership | The pendant or a second client can inject motion or errors while the module is driving | Refuse moves unless the module observed the last `robot_state` transition it caused; log every unexpected event line |
| Namespace: `viam-labs:rainbow-robotics` versus `viam:rainbow-robotics` | Changing it after publishing breaks configs | Decide before the first registry upload |
| Mesh licensing | URDF and meshes come from rbpodo\_ros2 under Apache-2.0 | Keep `THIRD_PARTY_NOTICES` with the upstream license and commit hash |

Assumptions this scope makes, stated so Claude Code does not have to guess: the state packet is little-endian and packed (every existing client treats it that way); `jnt_ref` is the joint position to report (Rainbow's ROS 2 driver does the same); the module owns the arm exclusively while viam-server runs; and Cartesian moves go through Viam's planner rather than the controller's own IK, so obstacle and frame-system awareness comes for free.

## Sources

Read directly (cloned at the commit current on 2026-09-29): [RainbowRobotics/rbpodo](https://github.com/RainbowRobotics/rbpodo) (`include/rbpodo/{cobot,cobot_data,common,data_type}.hpp`, `src/*.cpp`, `examples/`), [RainbowRobotics/rbpodo\_ros2](https://github.com/RainbowRobotics/rbpodo_ros2) (`rbpodo_description/robots/rb10_1300e/`, `rbpodo_hardware/src/robot.cpp`), [Rainbow RB cobot docs](https://rainbowrobotics.github.io/rb_cobot_docs/) (UI Script, data structure, DH parameters, system specifications, socket setup), [viam-modules/viam-ufactory-xarm](https://github.com/viam-modules/viam-ufactory-xarm), [viam-modules/viam-dobot](https://github.com/viam-modules/viam-dobot), [viam-modules/universal-robots](https://github.com/viam-modules/universal-robots), [viam-modules/yaskawa-robots](https://github.com/viam-modules/yaskawa-robots), [viam-modules/viam-franka-arm](https://github.com/viam-modules/viam-franka-arm), [viam-devrel/so-101](https://github.com/viam-devrel/so-101), [viam-devrel/rebot-b601](https://github.com/viam-devrel/rebot-b601), [viamrobotics/rdk](https://github.com/viamrobotics/rdk) (`components/arm/{arm.go,pb_helpers.go,server.go,fake,sim}`, `referenceframe/limits.go`, `motionplan/armplanning`), [viam-modules/trajex](https://github.com/viam-modules/trajex). Viam docs: [Add an arm](https://docs.viam.com/hardware/common-components/add-an-arm/), [Kinematics reference](https://docs.viam.com/motion-planning/reference/kinematics/), [Module developer reference](https://docs.viam.com/build-modules/module-reference/). Community clients: [leonardoedgar/rb5-ros-driver](https://github.com/leonardoedgar/rb5-ros-driver), [messy-snail/rb-api](https://github.com/messy-snail/rb-api).
