package arm

import (
	"encoding/binary"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// glbBounds returns the union of the POSITION accessor bounds of a GLB, in metres.
func glbBounds(t *testing.T, path string) (minV, maxV [3]float64, frame string) {
	t.Helper()
	glb, err := os.ReadFile(path)
	test.That(t, err, test.ShouldBeNil)
	jsonLen := int(binary.LittleEndian.Uint32(glb[12:16]))
	var doc struct {
		Asset struct {
			Extras struct {
				Frame string `json:"frame"`
			} `json:"extras"`
		} `json:"asset"`
		Accessors []struct {
			Min []float64 `json:"min"`
			Max []float64 `json:"max"`
		} `json:"accessors"`
		Meshes []struct {
			Primitives []struct {
				Attributes map[string]int `json:"attributes"`
			} `json:"primitives"`
		} `json:"meshes"`
	}
	test.That(t, json.Unmarshal(glb[20:20+jsonLen], &doc), test.ShouldBeNil)
	minV = [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
	maxV = [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for _, m := range doc.Meshes {
		for _, p := range m.Primitives {
			a := doc.Accessors[p.Attributes["POSITION"]]
			for k := range 3 {
				minV[k] = math.Min(minV[k], a.Min[k])
				maxV[k] = math.Max(maxV[k], a.Max[k])
			}
		}
	}
	return minV, maxV, doc.Asset.Extras.Frame
}

// TestGLBsSitInsideCollisionGeometry pins the Get3DModels frame convention:
// the visualizer anchors each link's GLB at the link's collision geometry
// centre, so every visual mesh must be expressed with its origin there and
// fit inside that geometry (with a little slack for the real part's shape).
func TestGLBsSitInsideCollisionGeometry(t *testing.T) {
	var cfg struct {
		Links []referenceframe.LinkConfig `json:"links"`
	}
	test.That(t, json.Unmarshal(rb10KinematicsJSON, &cfg), test.ShouldBeNil)
	const slackM = 0.03

	for _, link := range cfg.Links {
		path := filepath.Join("3d_models", ModelName, link.ID+".glb")
		if _, err := os.Stat(path); err != nil {
			t.Skipf("%s not present in this checkout", path)
		}
		g := link.Geometry
		test.That(t, g, test.ShouldNotBeNil)

		// Half-extents of the geometry in its own frame, metres.
		var half [3]float64
		switch g.Type {
		case spatialmath.CapsuleType:
			half = [3]float64{g.R / 1000, g.R / 1000, g.L / 2000}
		case spatialmath.BoxType:
			half = [3]float64{g.X / 2000, g.Y / 2000, g.Z / 2000}
		case spatialmath.SphereType:
			half = [3]float64{g.R / 1000, g.R / 1000, g.R / 1000}
		default:
			t.Fatalf("%s: unhandled geometry type %q", link.ID, g.Type)
		}

		minV, maxV, frame := glbBounds(t, path)
		test.That(t, frame, test.ShouldEqual, "geometry-center")
		for k := range 3 {
			if minV[k] < -half[k]-slackM || maxV[k] > half[k]+slackM {
				t.Errorf("%s: GLB axis %d spans [%.3f, %.3f] m but geometry half-extent is %.3f m; run `go run ./tools/glbrebase`",
					link.ID, k, minV[k], maxV[k], half[k])
			}
		}
	}
}
