package arm

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// ModelName is the kinematic model name and the directory under 3d_models/.
const ModelName = "rb10-1300"

// rb10KinematicsJSON is the SVA kinematics file written from Rainbow's
// rbpodo_ros2 URDF (rb10_1300e). Zero pose is the arm pointing straight up.
//
//go:embed rb10_1300_kinematics.json
var rb10KinematicsJSON []byte

// urdfRelPath is where the opt-in URDF lives relative to the module root.
const urdfRelPath = "arm/rb10_1300e.urdf"

// JointNames are the joint ids in the SVA file, in wire order.
var JointNames = []string{"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"}

// LinkNames are the link ids in the SVA file, which also key Get3DModels.
var LinkNames = []string{"link0", "link1", "link2", "link3", "link4", "link5", "link6"}

// buildSVAModel parses the embedded SVA file, writes the configured velocity
// and acceleration limits into every joint so GetKinematics clients and the
// planner see them, and returns the model named after the resource.
func buildSVAModel(name string, speedDegs, accelDegs float64) (referenceframe.Model, error) {
	var cfg referenceframe.ModelConfigJSON
	if err := json.Unmarshal(rb10KinematicsJSON, &cfg); err != nil {
		return nil, fmt.Errorf("parsing embedded kinematics: %w", err)
	}
	limits := make(map[string]referenceframe.JointLimits, len(JointNames))
	for _, j := range JointNames {
		limits[j] = referenceframe.JointLimits{
			MaxVelocity:     ptr(speedDegs),
			MaxAcceleration: ptr(accelDegs),
		}
	}
	patched, err := referenceframe.SetJointLimits(&cfg, limits)
	if err != nil {
		return nil, fmt.Errorf("applying joint limits: %w", err)
	}
	model, err := patched.ParseConfig(name)
	if err != nil {
		return nil, fmt.Errorf("building kinematic model: %w", err)
	}
	return model, nil
}

// moduleRoot returns the directory data files are shipped under. Candidates,
// in order: VIAM_MODULE_ROOT (the unpacked tarball for registry modules, but
// the executable's directory for local modules), the executable's directory,
// and its parent (the packaged layout is <root>/bin/rainbow-robotics next to
// <root>/arm/). The first one holding the URDF wins.
func moduleRoot() string {
	var candidates []string
	if root := os.Getenv("VIAM_MODULE_ROOT"); root != "" {
		candidates = append(candidates, root)
	}
	if exe, err := os.Executable(); err == nil {
		exeDir := filepath.Dir(exe)
		candidates = append(candidates, exeDir, filepath.Dir(exeDir))
	}
	for _, dir := range candidates {
		if _, err := os.Stat(filepath.Join(dir, urdfRelPath)); err == nil {
			return dir
		}
	}
	if len(candidates) > 0 {
		return candidates[0]
	}
	return "."
}

// buildURDFModel loads the opt-in URDF with STL collision meshes.
func buildURDFModel(name string, ratios []float64) (referenceframe.Model, error) {
	path := filepath.Join(moduleRoot(), urdfRelPath)
	model, err := referenceframe.ParseModelXMLFile(path, name, ratios)
	if err != nil {
		return nil, fmt.Errorf("loading URDF %s: %w", path, err)
	}
	return model, nil
}

// buildModel picks the SVA or URDF model per config.
func buildModel(name string, useURDF bool, ratios []float64, speedDegs, accelDegs float64) (referenceframe.Model, error) {
	if useURDF {
		return buildURDFModel(name, ratios)
	}
	return buildSVAModel(name, speedDegs, accelDegs)
}

// load3DModels reads one GLB per link from <root>/arm/3d_models/rb10-1300/.
// Missing files are logged, not fatal, so a source checkout still runs.
func load3DModels(logger logging.Logger) map[string]*commonpb.Mesh {
	dir := filepath.Join(moduleRoot(), "arm", "3d_models", ModelName)
	out := make(map[string]*commonpb.Mesh, len(LinkNames))
	for _, link := range LinkNames {
		path := filepath.Join(dir, link+".glb")
		data, err := os.ReadFile(path) //nolint:gosec
		if err != nil {
			logger.Debugw("3D model not available", "link", link, "path", path, "error", err)
			continue
		}
		out[link] = &commonpb.Mesh{ContentType: "model/gltf-binary", Mesh: data}
	}
	return out
}

// degsToInputs converts wire degrees to referenceframe inputs (radians).
func degsToInputs(j [6]float64) []referenceframe.Input {
	out := make([]referenceframe.Input, JointCount)
	for i, v := range j {
		out[i] = v * math.Pi / 180
	}
	return out
}

// inputsToDegs converts radians to wire degrees; it errors on the wrong DoF.
func inputsToDegs(in []referenceframe.Input) ([6]float64, error) {
	var out [6]float64
	if len(in) != JointCount {
		return out, fmt.Errorf("expected %d joint positions, got %d", JointCount, len(in))
	}
	for i, v := range in {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return out, fmt.Errorf("joint %d position is not finite", i)
		}
		// Round to a micro-degree so the wire never carries float noise like 29.999999999999996.
		out[i] = math.Round(v*180/math.Pi*1e6) / 1e6
	}
	return out, nil
}

// RainbowPoseToSpatial converts a controller TCP tuple (x,y,z mm; rx,ry,rz deg
// as Euler Z-Y'-X”) to a Pose.
func RainbowPoseToSpatial(p [6]float64) spatialmath.Pose {
	return spatialmath.NewPose(
		r3.Vector{X: p[0], Y: p[1], Z: p[2]},
		&spatialmath.EulerAngles{
			Roll:  p[3] * math.Pi / 180,
			Pitch: p[4] * math.Pi / 180,
			Yaw:   p[5] * math.Pi / 180,
		},
	)
}

// SpatialToRainbowPose is the inverse of RainbowPoseToSpatial.
func SpatialToRainbowPose(pose spatialmath.Pose) [6]float64 {
	pt := pose.Point()
	ea := pose.Orientation().EulerAngles()
	return [6]float64{pt.X, pt.Y, pt.Z, ea.Roll * 180 / math.Pi, ea.Pitch * 180 / math.Pi, ea.Yaw * 180 / math.Pi}
}

// maxAbsDiffDeg returns the L-infinity distance between two joint vectors.
func maxAbsDiffDeg(a, b [6]float64) float64 {
	var m float64
	for i := range a {
		if d := math.Abs(a[i] - b[i]); d > m {
			m = d
		}
	}
	return m
}

// poseToMap renders a pose as {x,y,z (mm), o_x,o_y,o_z,theta (deg)} for DoCommand replies.
func poseToMap(p spatialmath.Pose) map[string]any {
	pt := p.Point()
	ov := p.Orientation().OrientationVectorDegrees()
	return map[string]any{
		"x": pt.X, "y": pt.Y, "z": pt.Z,
		"o_x": ov.OX, "o_y": ov.OY, "o_z": ov.OZ, "theta": ov.Theta,
	}
}
