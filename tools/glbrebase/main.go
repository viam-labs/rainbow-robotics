// Command glbrebase re-expresses the Get3DModels GLBs in the frame the Viam
// visualizer places them in.
//
// dae2glb writes each link's visual mesh in its URDF link frame (metres, Z up,
// origin at the link's proximal joint). The visualizer, however, anchors a
// link's GLB at the link's collision geometry: it takes the frame named
// <arm>:<link id>, offsets by that link's geometry centre (translation and
// orientation from the kinematics file), and drops the GLB's origin there
// (viamrobotics/visualization: GeometryModel.svelte composes WorldMatrix ×
// Center; kinematicsFrames.ts derives Center from the link's geometry). A
// mesh must therefore have its origin at the geometry centre and its axes
// aligned with the geometry's orientation, so a capsule's axis is the GLB's Z.
//
// This tool applies the inverse of each link's geometry pose to the vertex
// positions and normals in place, rewrites the accessor bounds, and marks the
// file in asset.extras so a second run is a no-op.
//
//	go run ./tools/glbrebase -kinematics arm/rb10_1300_kinematics.json -dir arm/3d_models/rb10-1300
package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

const rebasedMarker = "geometry-center"

func main() {
	kin := flag.String("kinematics", "arm/rb10_1300_kinematics.json", "SVA kinematics file holding the link geometries")
	dir := flag.String("dir", "arm/3d_models/rb10-1300", "directory of <link id>.glb files to rewrite in place")
	force := flag.Bool("force", false, "rebase even if the file is already marked as rebased")
	flag.Parse()

	data, err := os.ReadFile(*kin)
	if err != nil {
		fatal(err)
	}
	var cfg struct {
		Links []referenceframe.LinkConfig `json:"links"`
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		fatal(fmt.Errorf("parsing %s: %w", *kin, err))
	}

	for _, link := range cfg.Links {
		path := filepath.Join(*dir, link.ID+".glb")
		if _, err := os.Stat(path); err != nil {
			fmt.Printf("%s: no GLB, skipped\n", link.ID)
			continue
		}
		if link.Geometry == nil {
			fmt.Printf("%s: link has no geometry, left in link frame\n", link.ID)
			continue
		}
		geom, err := link.Geometry.ParseConfig()
		if err != nil {
			fatal(fmt.Errorf("%s: geometry: %w", link.ID, err))
		}
		// Geometry poses are in millimetres; the GLB is in metres.
		gp := geom.Pose()
		inv := spatialmath.PoseInverse(spatialmath.NewPose(gp.Point().Mul(0.001), gp.Orientation()))

		glb, err := os.ReadFile(path)
		if err != nil {
			fatal(err)
		}
		out, done, err := rebase(glb, inv, *force)
		if err != nil {
			fatal(fmt.Errorf("%s: %w", path, err))
		}
		if !done {
			fmt.Printf("%s: already rebased, skipped (use -force to redo)\n", link.ID)
			continue
		}
		if err := os.WriteFile(path, out, 0o644); err != nil {
			fatal(err)
		}
		t := inv.Point()
		fmt.Printf("%s: rebased by %s translation (%.4f, %.4f, %.4f) m\n", link.ID, orientationDesc(gp), t.X, t.Y, t.Z)
	}
}

func orientationDesc(p spatialmath.Pose) string {
	ov := p.Orientation().OrientationVectorDegrees()
	if ov.Theta == 0 && math.Abs(ov.OZ-1) < 1e-9 {
		return "identity rotation,"
	}
	return fmt.Sprintf("OV(%.3f, %.3f, %.3f, th %.1f°) rotation,", ov.OX, ov.OY, ov.OZ, ov.Theta)
}

// rebase applies pose to every POSITION (and NORMAL) accessor of a GLB
// produced by dae2glb: one buffer, float32 VEC3 attributes, no node transforms.
func rebase(glb []byte, pose spatialmath.Pose, force bool) ([]byte, bool, error) {
	if len(glb) < 20 || binary.LittleEndian.Uint32(glb[0:4]) != 0x46546C67 {
		return nil, false, fmt.Errorf("not a GLB")
	}
	jsonLen := int(binary.LittleEndian.Uint32(glb[12:16]))
	if binary.LittleEndian.Uint32(glb[16:20]) != 0x4E4F534A || 20+jsonLen+8 > len(glb) {
		return nil, false, fmt.Errorf("unexpected chunk layout")
	}
	var doc map[string]any
	if err := json.Unmarshal(glb[20:20+jsonLen], &doc); err != nil {
		return nil, false, err
	}
	binStart := 20 + jsonLen + 8
	binLen := int(binary.LittleEndian.Uint32(glb[20+jsonLen : 24+jsonLen]))
	if binary.LittleEndian.Uint32(glb[24+jsonLen:28+jsonLen]) != 0x004E4942 || binStart+binLen > len(glb) {
		return nil, false, fmt.Errorf("missing BIN chunk")
	}
	bin := append([]byte(nil), glb[binStart:binStart+binLen]...)

	asset, _ := doc["asset"].(map[string]any)
	if asset == nil {
		asset = map[string]any{"version": "2.0"}
		doc["asset"] = asset
	}
	extras, _ := asset["extras"].(map[string]any)
	if extras != nil && extras["frame"] == rebasedMarker && !force {
		return nil, false, nil
	}
	if len(doc["nodes"].([]any)) != 1 || len(doc["buffers"].([]any)) != 1 {
		return nil, false, fmt.Errorf("expected a single node and buffer (dae2glb output)")
	}
	if n := doc["nodes"].([]any)[0].(map[string]any); n["matrix"] != nil || n["translation"] != nil || n["rotation"] != nil || n["scale"] != nil {
		return nil, false, fmt.Errorf("node carries a transform; bake it first")
	}

	accessors := doc["accessors"].([]any)
	views := doc["bufferViews"].([]any)
	rot := pose.Orientation().RotationMatrix()
	seen := map[int]bool{}
	transform := func(accIdx int, withTranslation bool) error {
		if seen[accIdx] {
			return nil
		}
		seen[accIdx] = true
		acc := accessors[accIdx].(map[string]any)
		if acc["componentType"].(float64) != 5126 || acc["type"] != "VEC3" {
			return fmt.Errorf("accessor %d is not float32 VEC3", accIdx)
		}
		view := views[int(acc["bufferView"].(float64))].(map[string]any)
		off := int(view["byteOffset"].(float64))
		if o, ok := acc["byteOffset"].(float64); ok {
			off += int(o)
		}
		if _, ok := view["byteStride"]; ok {
			return fmt.Errorf("interleaved buffer views are not supported")
		}
		count := int(acc["count"].(float64))
		minV := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
		maxV := [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
		for i := range count {
			p := off + i*12
			v := r3.Vector{
				X: float64(math.Float32frombits(binary.LittleEndian.Uint32(bin[p:]))),
				Y: float64(math.Float32frombits(binary.LittleEndian.Uint32(bin[p+4:]))),
				Z: float64(math.Float32frombits(binary.LittleEndian.Uint32(bin[p+8:]))),
			}
			if withTranslation {
				v = spatialmath.Compose(pose, spatialmath.NewPoseFromPoint(v)).Point()
			} else {
				v = rot.Mul(v)
			}
			out := [3]float64{v.X, v.Y, v.Z}
			for k := range 3 {
				binary.LittleEndian.PutUint32(bin[p+4*k:], math.Float32bits(float32(out[k])))
				minV[k] = min(minV[k], float64(float32(out[k])))
				maxV[k] = max(maxV[k], float64(float32(out[k])))
			}
		}
		if withTranslation && count > 0 {
			acc["min"] = minV[:]
			acc["max"] = maxV[:]
		}
		return nil
	}
	for _, m := range doc["meshes"].([]any) {
		for _, p := range m.(map[string]any)["primitives"].([]any) {
			attrs := p.(map[string]any)["attributes"].(map[string]any)
			if pos, ok := attrs["POSITION"].(float64); ok {
				if err := transform(int(pos), true); err != nil {
					return nil, false, err
				}
			}
			if nrm, ok := attrs["NORMAL"].(float64); ok {
				if err := transform(int(nrm), false); err != nil {
					return nil, false, err
				}
			}
		}
	}
	if extras == nil {
		extras = map[string]any{}
		asset["extras"] = extras
	}
	extras["frame"] = rebasedMarker

	jsonBytes, err := json.Marshal(doc)
	if err != nil {
		return nil, false, err
	}
	for len(jsonBytes)%4 != 0 {
		jsonBytes = append(jsonBytes, ' ')
	}
	var out bytes.Buffer
	total := 12 + 8 + len(jsonBytes) + 8 + len(bin)
	for _, v := range []uint32{0x46546C67, 2, uint32(total), uint32(len(jsonBytes)), 0x4E4F534A} { //nolint:gosec
		_ = binary.Write(&out, binary.LittleEndian, v)
	}
	out.Write(jsonBytes)
	_ = binary.Write(&out, binary.LittleEndian, uint32(len(bin))) //nolint:gosec
	_ = binary.Write(&out, binary.LittleEndian, uint32(0x004E4942))
	out.Write(bin)
	return out.Bytes(), true, nil
}

func fatal(err error) {
	fmt.Fprintln(os.Stderr, "glbrebase:", err)
	os.Exit(1)
}
