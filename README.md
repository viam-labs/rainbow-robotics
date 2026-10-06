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

Everything marked **verify on hardware** below was read from Rainbow's
documentation and SDK source, not tested on an arm yet. See the bring-up
checklist at the end.

## Configure the arm

Only `host` is required. Every other default moves the arm slowly in whatever
mode the pendant already has it in.

```json
{
  "name": "rb10",
  "api": "rdk:component:arm",
  "model": "viam:rainbow-robotics:rb10-1300",
  "attributes": {
    "host": "10.0.2.7",
    "speed_degs_per_sec": 30,
    "operation_mode": "simulation"
  }
}
```

Start with `operation_mode: simulation`: the controller then moves only its
commanded reference, and the module exercises every code path without moving
the arm. Switch to `real` once the checklist passes.

### Attributes

| Attribute | Type | Default | Meaning |
| --- | --- | --- | --- |
| `host` | string | required | Control box IP (Rainbow's default is `10.0.2.7`). |
| `command_port` | int | 5000 | Script command port. |
| `data_port` | int | 5001 | State packet port. |
| `speed_degs_per_sec` | number | 60 | Joint speed for `move_j` and batch moves; also published as `max_velocity` in the kinematics. 0 < v ≤ `max_speed_degs_per_sec`. |
| `acceleration_degs_per_sec_per_sec` | number | 200 | Joint acceleration; published as `max_acceleration`. **verify on hardware** |
| `max_speed_degs_per_sec` | number | 180 | Ceiling that `set_speed`, `MoveOptions` and per-move overrides are clamped to. |
| `max_acceleration_degs_per_sec_per_sec` | number | unset | Ceiling for acceleration; unset means no clamp. |
| `operation_mode` | `real` or `simulation` | `real` | Sent as `pgmode` after activation. `simulation` is the safe bring-up default. |
| `activate_on_start` | bool | true | Send `mc jall init` if the arm is not activated and wait for activation. |
| `activate_timeout_secs` | number | 60 | How long to wait for activation. |
| `speed_bar` | number | unset | 0–1. When set, sent as `set_speed_bar` at start and after every reconnect; when unset the pendant's value is left alone. |
| `state_poll_hz` | int | 100 | Rate of `reqdata` polling (10–500). |
| `move_hz` | int | 100 | Default setpoint spacing for streamed moves (10–500). Above 100 needs the pendant's high-speed external script mode. |
| `servo_gain`, `servo_alpha`, `servo_hold_secs` | number | 0.5 / 0.5 / 0.03 | Passed to `move_servo_j` as `gain`, `alpha`, `t2` (the values Rainbow's ROS 2 driver uses). |
| `joint_batch_blend` | number | 0.5 | Blend argument of `move_jb2_add`. **verify on hardware** |
| `joint_batch_chunk_size` | int | 0 (no chunking) | Split long waypoint lists into batches of this size if the controller rejects large `move_jb2` batches. The arm stops between chunks. |
| `reject_move_request_threshold_deg` | number | 10 | A batch or streamed move whose first waypoint is further than this (L∞) from the current position is refused. |
| `collision_detection` | bool | unset | When set, sent as `set_collision_onoff`. |
| `collision_sensitivity` | number | unset | 0–1, sent as `set_collision_th`. |
| `payload_kg`, `payload_cog_mm` | number, [x,y,z] | unset | Sent as `set_payload_info`; re-sent after every reconnect because the controller resets it when a pendant program ends. |
| `motion` | string | `builtin` | Motion service used by `MoveToPosition` (optional dependency). Falls back to in-process planning if it does not resolve. |
| `use_urdf` | bool | false | Serve the URDF with STL collision meshes instead of the SVA file. The URDF's own ±180° joint limits apply and velocity limits cannot be published (RSDK-14232). |
| `mesh_decimation_ratios` | [number] | unset | Per-mesh decimation when loading the URDF. |
| `command_timeout_secs` | number | 2 | How long to wait for `The command was executed`. |
| `power_off_on_close` | bool | false | Send `AvadaKedavra()` in `Close`. Off so a viam-server restart never drops the arm. **verify on hardware** |
| `allow_raw_commands` | bool | false | Enables the `eval` DoCommand. |

### How moves are executed

| Viam method | What the module sends |
| --- | --- |
| `MoveToJointPositions` | `move_j(jnt[...], v, a)`, then waits for `robot_state` to go 3 → 1 and the joints to be within 0.5° of target. Per-move overrides: `extra["speed_degs_per_sec"]`, `extra["acceleration_degs_per_sec_per_sec"]`. |
| `MoveThroughJointPositions` | `move_jb2_clear()`, one `move_jb2_add` per waypoint, `move_jb2_run()`; the controller blends through the waypoints. `MoveOptions` velocity and acceleration limits are clamped and applied to the whole batch. |
| `MoveThroughJointPositionsStreamed` | One `move_servo_j` per trajectory point, each sent at the previous point's time with `t1` equal to the point spacing. Ends with `move_speed_j` zeros and a barrier. Velocity and acceleration constraints are ignored (the controller tracks positions). |
| `MoveToPosition` | Motion service `Move` when available, else in-process `armplanning.MoveArm`; both end up in `MoveThroughJointPositions`. |
| `Stop` | Cancels the running operation, `move_speed_j` zeros if streaming, `task stop`, waits up to 2 s for idle. |
| `SetManualMode` | `freedrive_teach_on()` / `freedrive_teach_off()`; real mode only. A non-zero duration turns freedrive off automatically. |

Before every move the module checks its latest state packet (under 500 ms old)
for: activated, configured operation mode, no e-stop, no collision stop, no soft
e-stop, no kinematic emergency stop, no joint error bits, not in freedrive. A
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
| `clear_error` | — | Sends `task stop` and reports which fault conditions remain. **verify on hardware**: the documented recovery script is unknown. |
| `set_payload` | `{"kg", "cog_mm": [x,y,z]}` | `set_payload_info`. |
| `power_off` | — | `AvadaKedavra()` when idle. **verify on hardware** |
| `get_io` | — | Box and tool-flange digital and analog I/O from the snapshot. |
| `set_box_dout` | `{"port", "value"}` | `set_box_dout(port, -1/0/1)`. |
| `set_box_aout` | `{"port", "value"}` | `set_box_aout(port, volts)`. |
| `set_tool_out` | `{"voltage", "out0", "out1"}` | `tool_out V,s0,s1`. |
| `get_ft` | — | `eft_*` force-torque values. |
| `calc_fk` | `[j0..j5]` degrees | Controller-side FK via `print(calc_fk_tcp(...))`, returned next to the module's own model pose for cross-checking. |
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
`link6` so the links that carry geometry and meshes keep identity orientation. **verify on hardware** whether the controller's `tcp_ref`
uses the same X direction; the FK cross-check reports it. Joint limits are
±360° (elbow ±165°); the configured speed and
acceleration are written into every joint so the motion service's trajectory
limits see them. Collision geometry is one box for the base and one capsule per
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

If Brad's arm turns out to be the `rb10_1300e_u` variant (wrist1 y offset
151.4 mm instead of 148.4 mm) the FK cross-check will show a 3 mm error; change
that one translation in the kinematics file.

## Simulated arm

```json
{
  "name": "rb10-sim",
  "api": "rdk:component:arm",
  "model": "viam:rainbow-robotics:rb10-1300-simulated",
  "attributes": { "speed_degs_per_sec": 60, "simulate_time": true }
}
```

| Attribute | Default | Meaning |
| --- | --- | --- |
| `speed_degs_per_sec` | 60 | Interpolation speed. |
| `acceleration_degs_per_sec_per_sec` | 200 | Published limit only. |
| `simulate_time` | false | When true, moves take real time at the configured speed; when false, joints jump to the target. |
| `use_urdf`, `mesh_decimation_ratios` | | As for the real arm. |

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
`tools/` mesh conversion utilities. See `CLAUDE.md` for the conventions and
safety rules.

## Hardware bring-up checklist

Each step gates the next.

1. Laptop on `10.0.2.x`; `nc 10.0.2.7 5000` accepts `get_control_box_info()`
   and answers `info[controlbox][...]`. Record the system version.
2. `RB_HOST=10.0.2.7 go test -run TestHardwareControlBoxInfo -v ./arm/`.
3. Module in `operation_mode: simulation`: `JointPositions` matches the pendant,
   `MoveToJointPositions` to `[0,0,0,0,0,0]` and `[0,-30,90,0,60,0]`, `Stop`
   mid-move, `IsMoving` toggles (`TestHardwareSimulationMoves`).
4. FK cross-check: `TestHardwareFKCrossCheck` (50 random poses, model versus
   `calc_fk_tcp`, within 1 mm and 0.5°) with the controller TCP offset at zero.
   This confirms the kinematics file, the joint sign convention and the Euler
   convention of `tcp_ref` in one go.
5. `MoveThroughJointPositions` with a 50-waypoint planner output in simulation;
   note whether `move_jb2` accepts it whole. If not, set `joint_batch_chunk_size`.
6. Streamed moves in simulation at 50 Hz, then 100 Hz; `robot_state` must not
   drop to 1 mid-stream.
7. Switch to `real` with `speed_bar: 0.2`, e-stop within reach, nothing inside
   the 1.3 m reach; repeat the reference poses and one `MoveToPosition` through
   the motion service.
8. Collision test: obstruct gently with collision detection on; confirm
   `stopped before reaching target: collision stop is active` and that
   `clear_error` recovers (or find the right recovery script and update it).
9. Raise `speed_bar` to 1.0 and `speed_degs_per_sec` to 90; repeat the planner
   test; measure setpoint-to-motion latency on the streamed path.
10. Power-cycle the control box with viam-server running; confirm the module
    reconnects and re-activates without manual intervention.

## Sources

Wire format and script grammar from
[RainbowRobotics/rbpodo](https://github.com/RainbowRobotics/rbpodo) and the
[RB cobot docs](https://rainbowrobotics.github.io/rb_cobot_docs/); kinematics
and meshes from
[RainbowRobotics/rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2)
(Apache-2.0, see `THIRD_PARTY_NOTICES`).
