# Changelog

## Unreleased

Initial implementation of the `viam:rainbow-robotics` module.

- `rb`: pure-Go client for the RB-series control box. ASCII script commands on
  port 5000 with sequence-numbered acknowledgement tracking, 580-byte
  `systemSTAT` decoding on port 5001, tagged `print()` queries, controller
  error table, per-socket reconnect with 1 s backoff.
- `rb/fakebox`: in-process fake control box (both ports, motion simulation,
  events, faults, fragmented packets, dropped connections) for CI.
- `arm`: `viam:rainbow-robotics:rb10-1300` implementing the full RDK
  `arm.Arm` API: `move_j`, controller-blended `move_jb2` batches, streamed
  `move_servo_j` execution, `task stop`, freedrive manual mode, motion-service
  or in-process Cartesian planning, DoCommand vocabulary, `Get3DModels` GLBs,
  SVA kinematics with configured velocity and acceleration limits, opt-in
  URDF with STL collision meshes.
- `arm`: `viam:rainbow-robotics:rb10-1300-simulated` twin (no sockets).
- Hardware tests gated by `RB_HOST` for the FK cross-check and simulation-mode moves.
