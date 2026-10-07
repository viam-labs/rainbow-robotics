# Rainbow Robotics arm module for Viam

A pure-Go [Viam](https://www.viam.com) module for the Rainbow Robotics
RB10-1300 collaborative arm. It talks directly to the control box over its two
TCP sockets (newline-terminated script commands on port 5000, the binary
`systemSTAT` packet on port 5001), with no vendor SDK and no cgo in the
protocol layer.

Models:

| Model | Description |
| --- | --- |
| `viam:rainbow-robotics:rb10-1300` | The arm, over the control box's native protocol |
| `viam:rainbow-robotics:rb10-1300-simulated` | In-process twin with the same kinematics, for planning without hardware |

Items marked **unverified on hardware** were derived from Rainbow's
documentation and SDK source and have not yet been confirmed against a physical
arm. See [Running on real hardware](#running-on-real-hardware) for the
validation sequence.

## Configure the arm

Only `host` is required. Every other default moves the arm slowly in whatever
mode the controller is already in. The `frame` block is what lets the motion
service (and so `MoveToPosition`) address the arm.

```json
{
  "name": "rb10",
  "api": "rdk:component:arm",
  "model": "viam:rainbow-robotics:rb10-1300",
  "attributes": {
    "host": "10.0.2.7",
    "speed_degs_per_sec": 30,
    "operation_mode": "simulation"
  },
  "frame": { "parent": "world" }
}
```

Start with `operation_mode: simulation`: the controller then moves only its
commanded reference, and the module exercises every code path without moving
the arm. Switch to `real` once the validation sequence below passes.

### Attributes

| Attribute | Type | Default | Meaning |
| --- | --- | --- | --- |
| `host` | string | required | Control box IP (Rainbow's default is `10.0.2.7`). |
| `command_port` | int | 5000 | Script command port. |
| `data_port` | int | 5001 | State packet port. |
| `speed_degs_per_sec` | number | 60 | Joint speed for `move_j` and batch moves; also published as `max_velocity` in the kinematics. 0 < v ≤ `max_speed_degs_per_sec`. |
| `acceleration_degs_per_sec_per_sec` | number | 200 | Joint acceleration; published as `max_acceleration`. **unverified on hardware** |
| `max_speed_degs_per_sec` | number | 180 | Ceiling that `set_speed`, `MoveOptions` and per-move overrides are clamped to. |
| `max_acceleration_degs_per_sec_per_sec` | number | unset | Ceiling for acceleration; unset means no clamp. |
| `operation_mode` | `real` or `simulation` | `real` | Sent as `pgmode` after activation. `simulation` is the safe bring-up default. |
| `activate_on_start` | bool | true | Send `mc jall init` if the arm is not activated and wait for activation. |
| `activate_timeout_secs` | number | 60 | How long to wait for activation. |
| `auto_reactivate` | bool | false | Pressing the e-stop cuts arm power and drops the controller's activation; releasing it does not restore them. When true, the module re-runs the startup sequence (activation, mode, speed bar, payload, collision settings) once the e-stop has been released and the arm has stayed deactivated for a second. It never acts while the e-stop is held and never clears faults. |
| `speed_bar` | number | unset | 0–1. When set, sent as `set_speed_bar` at start and after every reconnect; when unset the pendant's value is left alone. |
| `state_poll_hz` | int | 100 | Rate of `reqdata` polling (10–500). |
| `move_hz` | int | 100 | Default setpoint spacing for streamed moves (10–500). Above 100 needs the pendant's high-speed external script mode. |
| `servo_gain`, `servo_alpha`, `servo_hold_secs` | number | 0.5 / 0.5 / 0.03 | Passed to `move_servo_j` as `gain`, `alpha`, `t2` (the values Rainbow's ROS 2 driver uses). |
| `joint_batch_blend` | number | 0.5 | Blend argument of `move_jb2_add`. **unverified on hardware** |
| `joint_batch_chunk_size` | int | 0 (no chunking) | Split long waypoint lists into batches of this size if the controller rejects large `move_jb2` batches. The arm stops between chunks. |
| `reject_move_request_threshold_deg` | number | 10 | A batch or streamed move whose first waypoint is further than this (L∞) from the current position is refused. |
| `collision_detection` | bool | unset | When set, sent as `set_collision_onoff`. |
| `collision_sensitivity` | number | unset | 0–1, sent as `set_collision_th`. |
| `payload_kg`, `payload_cog_mm` | number, [x,y,z] | unset | Sent as `set_payload_info`; re-sent after every reconnect because the controller resets it when a pendant program ends. |
| `motion` | string | `builtin` | Motion service used by `MoveToPosition`. The arm needs a `frame` block in the machine config (at minimum a `parent`) for the service to address it; without one, `MoveToPosition` is refused and every joint-space method still works. |
| `use_urdf` | bool | false | Serve the URDF with STL collision meshes instead of the SVA file. The URDF's own ±180° joint limits apply and velocity limits cannot be published (RSDK-14232). |
| `mesh_decimation_ratios` | [number] | unset | Per-mesh decimation when loading the URDF. |
| `command_timeout_secs` | number | 2 | How long to wait for `The command was executed`. |
| `power_off_on_close` | bool | false | Send `AvadaKedavra()` in `Close`. Off so a viam-server restart never drops the arm. **unverified on hardware** |
| `allow_raw_commands` | bool | false | Enables the `eval` DoCommand. |

### How moves are executed

| Viam method | What the module sends |
| --- | --- |
| `MoveToJointPositions` | `move_j(jnt[...], v, a)`, then waits for `robot_state` to go 3 → 1 and the joints to be within 0.5° of target. Per-move overrides: `extra["speed_degs_per_sec"]`, `extra["acceleration_degs_per_sec_per_sec"]`. |
| `MoveThroughJointPositions` | `move_jb2_clear()`, one `move_jb2_add` per waypoint, `move_jb2_run()`; the controller blends through the waypoints. `MoveOptions` velocity and acceleration limits are clamped and applied to the whole batch. |
| `MoveThroughJointPositionsStreamed` | One `move_servo_j` per trajectory point, each sent at the previous point's time with `t1` equal to the point spacing. Ends with `move_speed_j` zeros and a barrier. Velocity and acceleration constraints are ignored (the controller tracks positions). |
| `MoveToPosition` | Motion service `Move`, which plans against this arm's kinematics and geometries and executes through `MoveThroughJointPositions`. Planning is never done inside the module, so the binary is pure Go with no `libnlopt` dependency. |
| `Stop` | Cancels the running operation, `move_speed_j` zeros if streaming, `task stop`, waits up to 2 s for idle. |
| `SetManualMode` | `freedrive_teach_on()` / `freedrive_teach_off()`; real mode only. A non-zero duration turns freedrive off automatically. |

`MoveToJointPositions` is a plain `move_j` with no planning involved; only
`MoveToPosition` goes through the motion service. For Cartesian moves without
Viam planning use the `move_l` and `move_jl` DoCommands below, which hand the
pose to the controller's own IK.

Before every move the module checks its latest state packet (under 500 ms old)
for: activated, configured operation mode, no e-stop, no collision stop, no soft
e-stop, no kinematic emergency stop, no joint error bits, not in freedrive. The
two e-stop inputs are normally-closed safety channels that read 1 while
released; either reading 0 is treated as engaged (`get_state` shows both as
`estop_channel_1_closed` / `estop_channel_2_closed`). A
failed check returns an error naming the first failing condition. A fault seen
mid-move returns `stopped before reaching target: <reason>`; faults are never
auto-cleared.

Both sockets reconnect with 1 s backoff. After a command-port reconnect the
module re-runs the startup sequence (activation check, `pgmode`, speed bar,
payload, collision settings). A move in flight during a disconnect fails with
"connection to arm lost".

### DoCommand

Commands are dispatched on key presence, e.g. `{"set_speed": 30}`.

| Key | Argument | Effect |
| --- | --- | --- |
| `set_speed` | deg/s | Session joint speed, clamped to `max_speed_degs_per_sec`; returns the effective value. |
| `set_acceleration` | deg/s² | Same for acceleration. |
| `get_speed`, `get_acceleration` | — | Current session values. |
| `get_state` | — | Decoded snapshot: joints, `tcp_ref`, `robot_state`, mode, activation, e-stop and power bits, collision flags, joint faults, temperatures, currents, force-torque, snapshot age. |
| `get_state_description` | — | One string, e.g. `real / activated / idle`. |
| `activate` | — | `mc jall init`, wait for activation, re-apply mode and settings. |
| `set_operation_mode` | `"real"` or `"simulation"` | `pgmode ...`; refused while moving. |
| `set_speed_bar` | 0–1 | `set_speed_bar(...)`. |
| `set_collision` | `{"enabled", "sensitivity", "mode", "after"}` | The four `set_collision_*` scripts. |
| `clear_error` | — | Sends `task stop` and reports which fault conditions remain. **unverified on hardware**: the documented recovery script is unknown. |
| `set_payload` | `{"kg", "cog_mm": [x,y,z]}` | `set_payload_info`. |
| `power_off` | — | `AvadaKedavra()` when idle. **unverified on hardware** |
| `get_io` | — | Box and tool-flange digital and analog I/O from the snapshot. |
| `set_box_dout` | `{"port", "value"}` | `set_box_dout(port, -1/0/1)`. |
| `set_box_aout` | `{"port", "value"}` | `set_box_aout(port, volts)`. |
| `set_tool_out` | `{"voltage", "out0", "out1"}` | `tool_out V,s0,s1`. |
| `get_ft` | — | `eft_*` force-torque values. |
| `calc_fk` | `[j0..j5]` degrees | Controller-side FK via `print(calc_fk_tcp(...))`, returned next to the module's own model pose for cross-checking. |
| `move_l` | `{"pose": [x,y,z,rx,ry,rz], "speed", "acceleration"}` | Straight-line TCP move planned by the controller's own IK, no Viam motion planning. `pose` is in the controller's `tcp_ref` convention (mm, degrees), exactly as `get_state` reports it. Speed and acceleration are mm/s and mm/s², default 50 and 50, speed capped at 250 mm/s. Blocks until the TCP reference is within 1 mm and 0.5°. |
| `move_jl` | same as `move_l` | Joint-interpolated move to a TCP pose with controller IK. Speed and acceleration are joint deg/s and deg/s², defaulting to the session values and clamped like every joint move. |
| `eval` | script string | Raw single-line script pass-through; requires `allow_raw_commands: true`. |

`GetStatus` returns the same map as `get_state`.

### Kinematics and 3D models

The module embeds `arm/rb10_1300_kinematics.json`, an SVA file transcribed
from Rainbow's `rbpodo_ros2` URDF for `rb10_1300e` (joint origins d1 197.0,
a1 612.7, a2 570.15, d4 117.15, d5 117.15, d6 115.3 mm). Zero pose is the arm
straight up. The end-effector frame is a zero-length `tcp` link after `link6`
(matching the URDF's `tcp_joint`) with Z pointing out of the flange and X along
the base X at zero pose, so "point the tool down" means the flange faces down
and grippers attach along Z. The rotation lives on `tcp` rather than on
`link6` so the links that carry geometry and meshes keep identity orientation.
Whether the controller's `tcp_ref` uses the same X direction is **unverified on
hardware**; the FK cross-check reports it. Joint limits are ±360° (elbow
±165°); the configured speed and acceleration are written into every joint so
the motion service's trajectory limits see them. Collision geometry is one box for the base and one capsule per
link, sized from the collision meshes.

`Get3DModels` serves one GLB per link (`link0`…`link6`) from
`arm/3d_models/rb10-1300/`, converted from the DAE visual meshes with
`tools/dae2glb` and then re-expressed by `tools/glbrebase` (`make tools` runs
both). The Viam visualizer anchors a link's GLB at the frame `<arm>:<link id>`
offset by that link's **collision geometry centre and orientation**, so each
mesh has its origin at the capsule (or box) centre with the capsule axis along
Z, in metres. `TestGLBsSitInsideCollisionGeometry` checks every GLB fits its
geometry so a mesh left in URDF frame fails CI. Changing a link's geometry
pose in the kinematics file means re-running `make tools` (needs the
rbpodo_ros2 checkout). The GLBs match the default SVA kinematics; with
`use_urdf` the link geometries differ and the overlay is unverified.

The kinematics describe the `rb10_1300e` flange variant. On an `rb10_1300e_u`
(wrist1 y offset 151.4 mm instead of 148.4 mm) the FK cross-check shows a 3 mm
error; the fix is that one translation in the kinematics file.

## Simulated arm

```json
{
  "name": "rb10-sim",
  "api": "rdk:component:arm",
  "model": "viam:rainbow-robotics:rb10-1300-simulated",
  "attributes": { "speed_degs_per_sec": 60, "simulate_time": true },
  "frame": { "parent": "world" }
}
```

| Attribute | Default | Meaning |
| --- | --- | --- |
| `speed_degs_per_sec` | 60 | Interpolation speed. |
| `acceleration_degs_per_sec_per_sec` | 200 | Published limit only. |
| `simulate_time` | false | When true, moves take real time at the configured speed; when false, joints jump to the target. |
| `use_urdf`, `mesh_decimation_ratios`, `motion` | | As for the real arm. |

DoCommand supports `set_speed`, `set_acceleration`, `get_speed`,
`get_acceleration`, `get_state` and `set_joints` (teleport, degrees).

## Development

```sh
make test          # unit tests against the in-process fake control box
make lint
make module        # bin/module.tar.gz
RB_HOST=10.0.2.7 make test-hardware   # FK cross-check and simulation-mode moves on a real box
```

Repository layout: `rb/` is the protocol client (no Viam imports),
`rb/fakebox/` the fake control box used by tests, `arm/` the Viam resource,
`tools/` mesh conversion utilities. Conventions and safety rules for
contributors are in `CLAUDE.md`.

### Running without hardware

`tools/fakebox` runs the test double as a standalone control box on ports 5000
and 5001, so the real `rb10-1300` model can be driven from a local viam-server:

```sh
go run ./tools/fakebox -activated
```

Then configure the arm with `"host": "127.0.0.1"` and
`"operation_mode": "simulation"`. The fake box interpolates joints at the
configured speed and answers every script the module sends; commands it does
not implement are refused with `not allowed`, which is the cue to extend
`rb/fakebox/`.

## Running on real hardware

### Prerequisites

- An RB10-1300 with its control box powered on and the teach pendant connected.
  The module activates the arm itself (`mc jall init`) unless
  `activate_on_start` is false.
- The machine running viam-server on the control box's network. Rainbow's
  default control box address is `10.0.2.7`, so the host needs an address on
  `10.0.2.x`. Ports 5000 (script) and 5001 (state) must be reachable.
- Nothing inside the 1.3 m reach, and the e-stop within reach of whoever is
  testing.

Confirm connectivity before configuring anything:

```sh
printf 'get_control_box_info()\r\n' | nc -w 2 10.0.2.7 5000
```

The box answers `info[controlbox][...]` followed by `The command was executed`.

### Install the module

From the Viam registry, add the `viam:rainbow-robotics` module to the machine
and configure a component with model `viam:rainbow-robotics:rb10-1300`. To run
a local build instead, `make build` and add a local module whose executable
path is `bin/rainbow-robotics`.

### Validation sequence

Each step gates the next. The first six steps never move the arm.

1. **Connectivity.** The `nc` check above succeeds. Note the controller version
   it reports.
2. **Gated hardware tests.** These run only when `RB_HOST` is set and only ever
   use simulation mode:

   ```sh
   RB_HOST=10.0.2.7 make test-hardware
   ```

   `TestHardwareControlBoxInfo` checks the handshake. `TestHardwareFKCrossCheck`
   compares the module's model with the controller's `calc_fk_tcp` at 50 random
   joint configurations, within 1 mm and 0.5°, with the controller's TCP offset
   at zero. It settles the kinematics file, the joint sign convention, the Euler
   convention of `tcp_ref` and the flange variant in one run.
   `TestHardwareSimulationMoves` exercises blocking moves, `Stop` and
   `IsMoving`.
3. **Module in simulation mode.** Configure the arm with
   `operation_mode: simulation`. `JointPositions` should match the pendant.
   Move to `[0,0,0,0,0,0]` and `[0,-30,90,0,60,0]`, stop mid-move, and check
   `IsMoving`.
4. **Batch moves.** Run `MoveToPosition` through the motion service so the
   planner produces a long waypoint list. If the controller rejects the batch,
   set `joint_batch_chunk_size`.
5. **Streamed moves.** Stream at 50 Hz, then 100 Hz; `robot_state` must stay
   at 3 for the whole stream. Rates above 100 Hz need the pendant's high-speed
   external script mode.
6. **Settings.** Confirm `speed_bar`, payload and collision settings from the
   config appear on the pendant after a module restart.
7. **Real mode, slow.** Set `operation_mode: real` and `speed_bar: 0.2`. Repeat
   the reference poses and one motion-service move.
8. **Faults.** With collision detection on, obstruct the arm gently. The move
   should fail with `stopped before reaching target: collision stop is active`.
   Then test `clear_error`. The documented recovery script is unverified, so
   this step may need a code change.
9. **Speed.** Raise `speed_bar` to 1.0 and `speed_degs_per_sec` to 90; repeat
   the planner test and measure setpoint-to-motion latency on the streamed path.
10. **Resilience.** Power-cycle the control box with viam-server running. The
    module should reconnect and re-activate without intervention.

Faults are never cleared automatically. `power_off_on_close` defaults to false
so a viam-server restart never powers the arm down.

## Sources

Wire format and script grammar from
[RainbowRobotics/rbpodo](https://github.com/RainbowRobotics/rbpodo) and the
[RB cobot docs](https://rainbowrobotics.github.io/rb_cobot_docs/); kinematics
and meshes from
[RainbowRobotics/rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2)
(Apache-2.0, see `THIRD_PARTY_NOTICES`).
