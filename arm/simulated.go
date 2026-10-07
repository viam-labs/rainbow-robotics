package arm

import (
	"context"
	"errors"
	"math"
	"sync"
	"time"

	commonpb "go.viam.com/api/common/v1"
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/grpc"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// SimulatedModel is the rb10-1300-simulated twin: same kinematics, geometry
// and DoCommand vocabulary, no sockets. It lets the motion stack be exercised
// without hardware.
var SimulatedModel = resource.NewModel(Namespace, Family, "rb10-1300-simulated")

func init() {
	resource.RegisterComponent(rdkarm.API, SimulatedModel, resource.Registration[rdkarm.Arm, *SimConfig]{
		Constructor: newSimulated,
	})
}

type simulated struct {
	resource.Named
	resource.AlwaysRebuild

	logger       logging.Logger
	model        referenceframe.Model
	opMgr        *operation.SingleOperationManager
	simulateTime bool
	motion       motion.Service // nil when the optional dependency did not resolve

	mu     sync.Mutex
	joints [6]float64 // degrees
	speed  float64    // deg/s
	accel  float64
	moving bool
	closed bool
}

func newSimulated(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (rdkarm.Arm, error) {
	cfg, err := resource.NativeConfig[*SimConfig](conf)
	if err != nil {
		return nil, err
	}
	a, err := NewSimulated(conf.ResourceName(), *cfg, logger)
	if err != nil {
		return nil, err
	}
	s := a.(*simulated)
	if conf.Frame == nil {
		logger.Warnw("no frame configured for this arm; MoveToPosition will be refused until one is added (joint moves are unaffected)")
		return s, nil
	}
	m := cfg.Motion
	if m == "" {
		m = "builtin"
	}
	if ms, err := motion.FromProvider(deps, m); err == nil {
		s.motion = ms
	} else {
		logger.Warnw("motion service not available; MoveToPosition will be refused", "motion", m, "error", err)
	}
	return s, nil
}

// NewSimulated constructs the simulated twin.
func NewSimulated(name resource.Name, cfg SimConfig, logger logging.Logger) (rdkarm.Arm, error) {
	speed := cfg.SpeedDegsPerSec
	if speed <= 0 {
		speed = DefaultSpeedDegsPerSec
	}
	accel := cfg.AccelerationDegsPerSec2
	if accel <= 0 {
		accel = DefaultAccelDegsPerSec2
	}
	model, err := buildModel(name.ShortName(), cfg.UseURDF, cfg.MeshDecimationRatios, speed, accel)
	if err != nil {
		return nil, err
	}
	return &simulated{
		Named:        name.AsNamed(),
		logger:       logger,
		model:        model,
		opMgr:        operation.NewSingleOperationManager(),
		simulateTime: cfg.SimulateTime,
		speed:        speed,
		accel:        accel,
	}, nil
}

func (s *simulated) JointPositions(ctx context.Context, extra map[string]any) ([]referenceframe.Input, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return degsToInputs(s.joints), nil
}

func (s *simulated) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *simulated) EndPosition(ctx context.Context, extra map[string]any) (spatialmath.Pose, error) {
	in, _ := s.JointPositions(ctx, extra)
	return s.model.Transform(in)
}

func (s *simulated) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

func (s *simulated) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	in, _ := s.JointPositions(ctx, extra)
	gif, err := s.model.Geometries(in)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (s *simulated) Get3DModels(ctx context.Context, extra map[string]any) (map[string]*commonpb.Mesh, error) {
	return load3DModels(s.logger), nil
}

func (s *simulated) Properties(ctx context.Context, extra map[string]any) (rdkarm.Properties, error) {
	return rdkarm.Properties{SupportManualMode: false, SupportCartesianCommands: true}, nil
}

func (s *simulated) SetManualMode(ctx context.Context, manualMode bool, enabledFor time.Duration, extra map[string]any) error {
	return grpc.UnimplementedError
}

func (s *simulated) ManualMode(ctx context.Context, extra map[string]any) (bool, error) {
	return false, grpc.UnimplementedError
}

func (s *simulated) IsMoving(ctx context.Context) (bool, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.moving, nil
}

func (s *simulated) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]any) error {
	return s.MoveThroughJointPositions(ctx, [][]referenceframe.Input{positions}, nil, extra)
}

// MoveThroughJointPositions interpolates linearly at the configured speed
// (or jumps instantly when simulate_time is false).
func (s *simulated) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *rdkarm.MoveOptions, extra map[string]any) error {
	targets := make([][6]float64, 0, len(positions))
	for _, wp := range positions {
		if err := rdkarm.CheckDesiredJointPositions(ctx, s, wp); err != nil {
			return err
		}
		t, err := inputsToDegs(wp)
		if err != nil {
			return err
		}
		targets = append(targets, t)
	}
	ctx, done := s.opMgr.New(ctx)
	defer done()

	s.mu.Lock()
	if s.closed {
		s.mu.Unlock()
		return errors.New("arm is closed")
	}
	speed := s.speed
	if v, ok := numberFromExtra(extra, "speed_degs_per_sec"); ok && v > 0 {
		speed = min(v, DefaultMaxSpeedDegs)
	}
	s.moving = true
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		s.moving = false
		s.mu.Unlock()
	}()

	for _, target := range targets {
		if !s.simulateTime {
			s.mu.Lock()
			s.joints = target
			s.mu.Unlock()
			continue
		}
		ticker := time.NewTicker(5 * time.Millisecond)
		last := time.Now()
		for {
			select {
			case <-ctx.Done():
				ticker.Stop()
				return ctx.Err()
			case now := <-ticker.C:
				step := speed * now.Sub(last).Seconds()
				last = now
				s.mu.Lock()
				reached := true
				for i := range s.joints {
					d := target[i] - s.joints[i]
					if math.Abs(d) <= step {
						s.joints[i] = target[i]
					} else {
						reached = false
						s.joints[i] += math.Copysign(step, d)
					}
				}
				s.mu.Unlock()
				if reached {
					ticker.Stop()
					goto next
				}
			}
		}
	next:
	}
	return nil
}

func (s *simulated) MoveThroughJointPositionsStreamed(ctx context.Context, batches <-chan []rdkarm.TrajectoryPoint, responses chan<- rdkarm.Response, extra map[string]any) error {
	for batch := range batches {
		for _, p := range batch {
			if err := s.MoveToJointPositions(ctx, p.Positions, extra); err != nil {
				return err
			}
		}
		select {
		case responses <- rdkarm.Response{}:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	return nil
}

// MoveToPosition goes through the motion service for the same reason as the
// real arm: planning in-process would link nlopt into the module binary.
func (s *simulated) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]any) error {
	if s.motion == nil {
		return errNoMotionService
	}
	_, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.Name().ShortName(),
		Destination:   referenceframe.NewPoseInFrame(s.Name().ShortName()+"_origin", pose),
		Extra:         extra,
	})
	return err
}

func (s *simulated) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *simulated) Stop(ctx context.Context, extra map[string]any) error {
	s.opMgr.CancelRunning(ctx)
	return nil
}

func (s *simulated) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	out := map[string]any{}
	for key, arg := range cmd {
		switch key {
		case "set_speed":
			v, err := toFloat(arg)
			if err != nil || v <= 0 {
				return nil, errors.New("set_speed expects a positive number")
			}
			s.mu.Lock()
			s.speed = min(v, DefaultMaxSpeedDegs)
			out["speed_degs_per_sec"] = s.speed
			s.mu.Unlock()
		case "set_acceleration":
			v, err := toFloat(arg)
			if err != nil || v <= 0 {
				return nil, errors.New("set_acceleration expects a positive number")
			}
			s.mu.Lock()
			s.accel = v
			out["acceleration_degs_per_sec_per_sec"] = v
			s.mu.Unlock()
		case "get_speed":
			s.mu.Lock()
			out["speed_degs_per_sec"] = s.speed
			s.mu.Unlock()
		case "get_acceleration":
			s.mu.Lock()
			out["acceleration_degs_per_sec_per_sec"] = s.accel
			s.mu.Unlock()
		case "get_state", "get_state_description":
			st, _ := s.Status(ctx)
			for k, v := range st {
				out[k] = v
			}
		case "set_joints":
			f, err := toFloats(arg)
			if err != nil || len(f) != JointCount {
				return nil, errors.New("set_joints expects 6 angles in degrees")
			}
			s.mu.Lock()
			copy(s.joints[:], f)
			s.mu.Unlock()
			out["joints_deg"] = f
		default:
			return nil, errors.New("unknown command " + key + "; supported: set_speed, set_acceleration, get_speed, get_acceleration, get_state, get_state_description, set_joints")
		}
	}
	return out, nil
}

func (s *simulated) Status(ctx context.Context) (map[string]any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	joints := make([]any, JointCount)
	for i, v := range s.joints {
		joints[i] = v
	}
	return map[string]any{
		"description":    "simulated / activated / idle",
		"simulated":      true,
		"joints_deg":     joints,
		"moving":         s.moving,
		"operation_mode": ModeSimulation,
		"activated":      true,
	}, nil
}

func (s *simulated) Close(ctx context.Context) error {
	s.opMgr.CancelRunning(ctx)
	s.mu.Lock()
	s.closed = true
	s.mu.Unlock()
	return nil
}
