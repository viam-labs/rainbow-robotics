package arm

import (
	"errors"
	"fmt"
	"time"

	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
)

// Operation modes accepted by the operation_mode attribute.
const (
	ModeReal       = "real"
	ModeSimulation = "simulation"
)

// Defaults for optional attributes. None of them moves the arm faster than a
// slow bring-up needs; see CLAUDE.md before changing any of them.
const (
	DefaultCommandPort      = 5000
	DefaultDataPort         = 5001
	DefaultSpeedDegsPerSec  = 60.0
	DefaultAccelDegsPerSec2 = 200.0
	DefaultMaxSpeedDegs     = 180.0
	DefaultStatePollHz      = 100
	DefaultMoveHz           = 100
	DefaultServoGain        = 0.5
	DefaultServoAlpha       = 0.5
	DefaultServoHoldSecs    = 0.03
	DefaultJointBatchBlend  = 0.5
	DefaultRejectThreshDeg  = 10.0
	DefaultMotionService    = "builtin"
	DefaultCommandTimeout   = 2.0
	DefaultActivateTimeout  = 60.0
	MaxPollHz               = 500
	MinPollHz               = 10
	JointCount              = 6
	snapshotMaxAge          = 500 * time.Millisecond
	arrivalToleranceDeg     = 0.5
)

// Config is the rb10-1300 model's attribute schema. Attribute names follow the
// UFactory xArm module so users see one vocabulary across Viam arms.
type Config struct {
	Host        string `json:"host"`
	CommandPort int    `json:"command_port,omitempty"`
	DataPort    int    `json:"data_port,omitempty"`

	SpeedDegsPerSec            float64 `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec2    float64 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
	MaxSpeedDegsPerSec         float64 `json:"max_speed_degs_per_sec,omitempty"`
	MaxAccelerationDegsPerSec2 float64 `json:"max_acceleration_degs_per_sec_per_sec,omitempty"`

	OperationMode       string   `json:"operation_mode,omitempty"`
	ActivateOnStart     *bool    `json:"activate_on_start,omitempty"`
	ActivateTimeoutSecs float64  `json:"activate_timeout_secs,omitempty"`
	SpeedBar            *float64 `json:"speed_bar,omitempty"`

	StatePollHz   int      `json:"state_poll_hz,omitempty"`
	MoveHz        int      `json:"move_hz,omitempty"`
	ServoGain     *float64 `json:"servo_gain,omitempty"`
	ServoAlpha    *float64 `json:"servo_alpha,omitempty"`
	ServoHoldSecs *float64 `json:"servo_hold_secs,omitempty"`

	JointBatchBlend               *float64 `json:"joint_batch_blend,omitempty"`
	JointBatchChunkSize           int      `json:"joint_batch_chunk_size,omitempty"`
	RejectMoveRequestThresholdDeg *float64 `json:"reject_move_request_threshold_deg,omitempty"`

	CollisionDetection   *bool     `json:"collision_detection,omitempty"`
	CollisionSensitivity *float64  `json:"collision_sensitivity,omitempty"`
	PayloadKg            *float64  `json:"payload_kg,omitempty"`
	PayloadCoGMM         []float64 `json:"payload_cog_mm,omitempty"`

	Motion               string    `json:"motion,omitempty"`
	UseURDF              bool      `json:"use_urdf,omitempty"`
	MeshDecimationRatios []float64 `json:"mesh_decimation_ratios,omitempty"`

	CommandTimeoutSecs float64 `json:"command_timeout_secs,omitempty"`
	PowerOffOnClose    bool    `json:"power_off_on_close,omitempty"`
	AllowRawCommands   bool    `json:"allow_raw_commands,omitempty"`
}

// Validate checks ranges and returns the motion service as an optional dependency.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	check := func(ok bool, format string, args ...any) error {
		if ok {
			return nil
		}
		return resource.NewConfigValidationError(path, fmt.Errorf(format, args...))
	}
	c := cfg.withDefaults()
	checks := []error{
		check(c.CommandPort >= 1 && c.CommandPort <= 65535, "command_port must be 1-65535, got %d", c.CommandPort),
		check(c.DataPort >= 1 && c.DataPort <= 65535, "data_port must be 1-65535, got %d", c.DataPort),
		check(c.MaxSpeedDegsPerSec > 0, "max_speed_degs_per_sec must be > 0"),
		check(c.SpeedDegsPerSec > 0 && c.SpeedDegsPerSec <= c.MaxSpeedDegsPerSec,
			"speed_degs_per_sec must be in (0, %v], got %v", c.MaxSpeedDegsPerSec, c.SpeedDegsPerSec),
		check(c.AccelerationDegsPerSec2 > 0, "acceleration_degs_per_sec_per_sec must be > 0, got %v", c.AccelerationDegsPerSec2),
		check(c.MaxAccelerationDegsPerSec2 == 0 || c.AccelerationDegsPerSec2 <= c.MaxAccelerationDegsPerSec2,
			"acceleration_degs_per_sec_per_sec %v exceeds max_acceleration_degs_per_sec_per_sec %v",
			c.AccelerationDegsPerSec2, c.MaxAccelerationDegsPerSec2),
		check(c.OperationMode == ModeReal || c.OperationMode == ModeSimulation,
			"operation_mode must be %q or %q, got %q", ModeReal, ModeSimulation, c.OperationMode),
		check(c.SpeedBar == nil || (*c.SpeedBar >= 0 && *c.SpeedBar <= 1), "speed_bar must be 0-1"),
		check(c.StatePollHz >= MinPollHz && c.StatePollHz <= MaxPollHz, "state_poll_hz must be %d-%d, got %d", MinPollHz, MaxPollHz, c.StatePollHz),
		check(c.MoveHz >= MinPollHz && c.MoveHz <= MaxPollHz, "move_hz must be %d-%d, got %d", MinPollHz, MaxPollHz, c.MoveHz),
		check(*c.ServoHoldSecs >= 0.02 && *c.ServoHoldSecs <= 0.2, "servo_hold_secs must be 0.02-0.2"),
		check(*c.JointBatchBlend >= 0, "joint_batch_blend must be >= 0"),
		check(c.JointBatchChunkSize >= 0, "joint_batch_chunk_size must be >= 0"),
		check(*c.RejectMoveRequestThresholdDeg >= 0, "reject_move_request_threshold_deg must be >= 0"),
		check(c.CollisionSensitivity == nil || (*c.CollisionSensitivity >= 0 && *c.CollisionSensitivity <= 1), "collision_sensitivity must be 0-1"),
		check(c.PayloadKg == nil || *c.PayloadKg >= 0, "payload_kg must be >= 0"),
		check(len(c.PayloadCoGMM) == 0 || len(c.PayloadCoGMM) == 3, "payload_cog_mm must have 3 values"),
		check(c.PayloadCoGMM == nil || c.PayloadKg != nil, "payload_cog_mm requires payload_kg"),
		check(c.CommandTimeoutSecs > 0, "command_timeout_secs must be > 0"),
		check(c.ActivateTimeoutSecs > 0, "activate_timeout_secs must be > 0"),
	}
	for _, err := range checks {
		if err != nil {
			return nil, nil, err
		}
	}
	return nil, []string{motion.Named(c.Motion).String()}, nil
}

func ptr[T any](v T) *T { return &v }

// withDefaults returns a copy with every unset attribute filled in.
func (cfg *Config) withDefaults() Config {
	c := *cfg
	if c.CommandPort == 0 {
		c.CommandPort = DefaultCommandPort
	}
	if c.DataPort == 0 {
		c.DataPort = DefaultDataPort
	}
	if c.SpeedDegsPerSec == 0 {
		c.SpeedDegsPerSec = DefaultSpeedDegsPerSec
	}
	if c.AccelerationDegsPerSec2 == 0 {
		c.AccelerationDegsPerSec2 = DefaultAccelDegsPerSec2
	}
	if c.MaxSpeedDegsPerSec == 0 {
		c.MaxSpeedDegsPerSec = DefaultMaxSpeedDegs
	}
	if c.OperationMode == "" {
		c.OperationMode = ModeReal
	}
	if c.ActivateOnStart == nil {
		c.ActivateOnStart = ptr(true)
	}
	if c.ActivateTimeoutSecs == 0 {
		c.ActivateTimeoutSecs = DefaultActivateTimeout
	}
	if c.StatePollHz == 0 {
		c.StatePollHz = DefaultStatePollHz
	}
	if c.MoveHz == 0 {
		c.MoveHz = DefaultMoveHz
	}
	if c.ServoGain == nil {
		c.ServoGain = ptr(DefaultServoGain)
	}
	if c.ServoAlpha == nil {
		c.ServoAlpha = ptr(DefaultServoAlpha)
	}
	if c.ServoHoldSecs == nil {
		c.ServoHoldSecs = ptr(DefaultServoHoldSecs)
	}
	if c.JointBatchBlend == nil {
		c.JointBatchBlend = ptr(DefaultJointBatchBlend)
	}
	if c.RejectMoveRequestThresholdDeg == nil {
		c.RejectMoveRequestThresholdDeg = ptr(DefaultRejectThreshDeg)
	}
	if c.Motion == "" {
		c.Motion = DefaultMotionService
	}
	if c.CommandTimeoutSecs == 0 {
		c.CommandTimeoutSecs = DefaultCommandTimeout
	}
	return c
}

// SimConfig is the rb10-1300-simulated model's attribute schema.
type SimConfig struct {
	SpeedDegsPerSec         float64   `json:"speed_degs_per_sec,omitempty"`
	AccelerationDegsPerSec2 float64   `json:"acceleration_degs_per_sec_per_sec,omitempty"`
	SimulateTime            bool      `json:"simulate_time,omitempty"`
	UseURDF                 bool      `json:"use_urdf,omitempty"`
	MeshDecimationRatios    []float64 `json:"mesh_decimation_ratios,omitempty"`
}

// Validate checks the simulated model's attributes.
func (cfg *SimConfig) Validate(path string) ([]string, []string, error) {
	if cfg.SpeedDegsPerSec < 0 || cfg.SpeedDegsPerSec > DefaultMaxSpeedDegs {
		return nil, nil, resource.NewConfigValidationError(path,
			fmt.Errorf("speed_degs_per_sec must be in (0, %v]", DefaultMaxSpeedDegs))
	}
	if cfg.AccelerationDegsPerSec2 < 0 {
		return nil, nil, resource.NewConfigValidationError(path, errors.New("acceleration_degs_per_sec_per_sec must be > 0"))
	}
	return nil, nil, nil
}
