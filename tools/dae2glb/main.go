// Command dae2glb converts the COLLADA (.dae) visual meshes from
// rbpodo_ros2/rbpodo_description into one binary glTF (.glb) per link for
// Get3DModels. It bakes each scene node's matrix into the vertices, keeps the
// URDF link frame (metres, Z up) and carries the diffuse colour of every
// material as an unlit-ish PBR base colour. No external dependencies.
//
//	go run ./tools/dae2glb -in path/to/meshes/rb10_1300e/visual -out arm/3d_models/rb10-1300
package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"encoding/xml"
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

type collada struct {
	Effects []struct {
		ID      string `xml:"id,attr"`
		Diffuse string `xml:"profile_COMMON>technique>lambert>diffuse>color"`
		Phong   string `xml:"profile_COMMON>technique>phong>diffuse>color"`
	} `xml:"library_effects>effect"`
	Materials []struct {
		ID     string `xml:"id,attr"`
		Effect struct {
			URL string `xml:"url,attr"`
		} `xml:"instance_effect"`
	} `xml:"library_materials>material"`
	Geometries []geometry `xml:"library_geometries>geometry"`
	Nodes      []node     `xml:"library_visual_scenes>visual_scene>node"`
}

type node struct {
	ID       string `xml:"id,attr"`
	Matrix   string `xml:"matrix"`
	Instance []struct {
		URL string `xml:"url,attr"`
	} `xml:"instance_geometry"`
	Children []node `xml:"node"`
}

type geometry struct {
	ID   string `xml:"id,attr"`
	Mesh struct {
		Sources []struct {
			ID    string `xml:"id,attr"`
			Array string `xml:"float_array"`
		} `xml:"source"`
		Vertices struct {
			ID     string  `xml:"id,attr"`
			Inputs []input `xml:"input"`
		} `xml:"vertices"`
		Triangles []struct {
			Material string  `xml:"material,attr"`
			Count    int     `xml:"count,attr"`
			Inputs   []input `xml:"input"`
			P        string  `xml:"p"`
		} `xml:"triangles"`
		Polylists []struct {
			Material string  `xml:"material,attr"`
			Count    int     `xml:"count,attr"`
			Inputs   []input `xml:"input"`
			VCount   string  `xml:"vcount"`
			P        string  `xml:"p"`
		} `xml:"polylist"`
	} `xml:"mesh"`
}

type input struct {
	Semantic string `xml:"semantic,attr"`
	Source   string `xml:"source,attr"`
	Offset   int    `xml:"offset,attr"`
}

type primitive struct {
	positions []float32
	normals   []float32
	indices   []uint32
	color     [4]float32
	name      string
}

func main() {
	in := flag.String("in", "", "directory of .dae files")
	out := flag.String("out", "", "output directory for .glb files")
	flag.Parse()
	if *in == "" || *out == "" {
		flag.Usage()
		os.Exit(2)
	}
	if err := os.MkdirAll(*out, 0o755); err != nil {
		fatal(err)
	}
	entries, err := os.ReadDir(*in)
	if err != nil {
		fatal(err)
	}
	for _, e := range entries {
		if !strings.HasSuffix(e.Name(), ".dae") {
			continue
		}
		src := filepath.Join(*in, e.Name())
		dst := filepath.Join(*out, strings.TrimSuffix(e.Name(), ".dae")+".glb")
		prims, err := convert(src)
		if err != nil {
			fatal(fmt.Errorf("%s: %w", src, err))
		}
		glb := writeGLB(strings.TrimSuffix(e.Name(), ".dae"), prims)
		if err := os.WriteFile(dst, glb, 0o644); err != nil {
			fatal(err)
		}
		var tris int
		for _, p := range prims {
			tris += len(p.indices) / 3
		}
		fmt.Printf("%s -> %s (%d primitives, %d triangles, %d bytes)\n", src, dst, len(prims), tris, len(glb))
	}
}

func fatal(err error) {
	fmt.Fprintln(os.Stderr, "dae2glb:", err)
	os.Exit(1)
}

func parseFloats(s string) []float64 {
	fields := strings.Fields(s)
	out := make([]float64, 0, len(fields))
	for _, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil {
			fatal(fmt.Errorf("bad float %q", f))
		}
		out = append(out, v)
	}
	return out
}

func parseInts(s string) []int {
	fields := strings.Fields(s)
	out := make([]int, 0, len(fields))
	for _, f := range fields {
		v, err := strconv.Atoi(f)
		if err != nil {
			fatal(fmt.Errorf("bad int %q", f))
		}
		out = append(out, v)
	}
	return out
}

// mat4 is a row-major 4x4 matrix as COLLADA writes it.
type mat4 [16]float64

func identity() mat4 { return mat4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1} }

func (m mat4) mul(o mat4) mat4 {
	var r mat4
	for i := range 4 {
		for j := range 4 {
			for k := range 4 {
				r[i*4+j] += m[i*4+k] * o[k*4+j]
			}
		}
	}
	return r
}

func (m mat4) point(x, y, z float64) (float64, float64, float64) {
	return m[0]*x + m[1]*y + m[2]*z + m[3],
		m[4]*x + m[5]*y + m[6]*z + m[7],
		m[8]*x + m[9]*y + m[10]*z + m[11]
}

func (m mat4) normal(x, y, z float64) (float64, float64, float64) {
	// Rotation part only; the node matrices here are rigid so no inverse-transpose is needed.
	nx := m[0]*x + m[1]*y + m[2]*z
	ny := m[4]*x + m[5]*y + m[6]*z
	nz := m[8]*x + m[9]*y + m[10]*z
	l := math.Sqrt(nx*nx + ny*ny + nz*nz)
	if l == 0 {
		return 0, 0, 1
	}
	return nx / l, ny / l, nz / l
}

func convert(path string) ([]primitive, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var c collada
	if err := xml.Unmarshal(data, &c); err != nil {
		return nil, err
	}
	colors := map[string][4]float32{}
	effects := map[string][4]float32{}
	for _, e := range c.Effects {
		s := e.Diffuse
		if s == "" {
			s = e.Phong
		}
		col := [4]float32{0.7, 0.7, 0.7, 1}
		if v := parseFloats(s); len(v) >= 3 {
			col = [4]float32{float32(v[0]), float32(v[1]), float32(v[2]), 1}
		}
		effects["#"+e.ID] = col
	}
	for _, m := range c.Materials {
		if col, ok := effects[m.Effect.URL]; ok {
			colors[m.ID] = col
		}
	}
	geoms := map[string]geometry{}
	for _, g := range c.Geometries {
		geoms["#"+g.ID] = g
	}

	var prims []primitive
	var walk func(n node, parent mat4)
	walk = func(n node, parent mat4) {
		m := parent
		if strings.TrimSpace(n.Matrix) != "" {
			v := parseFloats(n.Matrix)
			if len(v) == 16 {
				var local mat4
				copy(local[:], v)
				m = parent.mul(local)
			}
		}
		for _, inst := range n.Instance {
			g, ok := geoms[inst.URL]
			if !ok {
				fatal(fmt.Errorf("geometry %s not found", inst.URL))
			}
			prims = append(prims, geometryPrimitives(g, m, colors)...)
		}
		for _, ch := range n.Children {
			walk(ch, m)
		}
	}
	for _, n := range c.Nodes {
		walk(n, identity())
	}
	if len(prims) == 0 {
		// No scene: convert every geometry untransformed.
		for _, g := range c.Geometries {
			prims = append(prims, geometryPrimitives(g, identity(), colors)...)
		}
	}
	return prims, nil
}

func geometryPrimitives(g geometry, m mat4, colors map[string][4]float32) []primitive {
	sources := map[string][]float64{}
	for _, s := range g.Mesh.Sources {
		sources["#"+s.ID] = parseFloats(s.Array)
	}
	// Resolve the <vertices> indirection to the POSITION source.
	posSource := ""
	for _, in := range g.Mesh.Vertices.Inputs {
		if in.Semantic == "POSITION" {
			posSource = in.Source
		}
	}
	verticesID := "#" + g.Mesh.Vertices.ID

	type tri struct {
		material string
		inputs   []input
		p        []int
		vcount   []int
	}
	var groups []tri
	for _, t := range g.Mesh.Triangles {
		groups = append(groups, tri{material: t.Material, inputs: t.Inputs, p: parseInts(t.P)})
	}
	for _, pl := range g.Mesh.Polylists {
		groups = append(groups, tri{material: pl.Material, inputs: pl.Inputs, p: parseInts(pl.P), vcount: parseInts(pl.VCount)})
	}

	var prims []primitive
	for _, grp := range groups {
		stride := 0
		posOff, nrmOff := -1, -1
		var nrmSource string
		for _, in := range grp.inputs {
			if in.Offset+1 > stride {
				stride = in.Offset + 1
			}
			switch in.Semantic {
			case "VERTEX":
				posOff = in.Offset
				if in.Source != verticesID {
					posSource = in.Source
				}
			case "NORMAL":
				nrmOff = in.Offset
				nrmSource = in.Source
			}
		}
		if posOff < 0 || stride == 0 {
			continue
		}
		pos := sources[posSource]
		nrm := sources[nrmSource]

		// Expand polylist polygons to triangle fans; triangles are already flat.
		var corners [][2]int // (posIdx, nrmIdx) per corner, 3 per triangle
		addCorner := func(base int) {
			pi := grp.p[base+posOff]
			ni := -1
			if nrmOff >= 0 {
				ni = grp.p[base+nrmOff]
			}
			corners = append(corners, [2]int{pi, ni})
		}
		if grp.vcount == nil {
			for base := 0; base+stride*3 <= len(grp.p); base += stride * 3 {
				addCorner(base)
				addCorner(base + stride)
				addCorner(base + 2*stride)
			}
		} else {
			base := 0
			for _, n := range grp.vcount {
				for k := 1; k+1 < n; k++ {
					addCorner(base)
					addCorner(base + k*stride)
					addCorner(base + (k+1)*stride)
				}
				base += n * stride
			}
		}

		prim := primitive{name: grp.material}
		if col, ok := colors[grp.material]; ok {
			prim.color = col
		} else {
			prim.color = [4]float32{0.7, 0.7, 0.7, 1}
		}
		index := map[[2]int]uint32{}
		for _, c := range corners {
			idx, ok := index[c]
			if !ok {
				idx = uint32(len(prim.positions) / 3)
				index[c] = idx
				x, y, z := m.point(pos[c[0]*3], pos[c[0]*3+1], pos[c[0]*3+2])
				prim.positions = append(prim.positions, float32(x), float32(y), float32(z))
				if c[1] >= 0 && c[1]*3+2 < len(nrm) {
					nx, ny, nz := m.normal(nrm[c[1]*3], nrm[c[1]*3+1], nrm[c[1]*3+2])
					prim.normals = append(prim.normals, float32(nx), float32(ny), float32(nz))
				} else {
					prim.normals = append(prim.normals, 0, 0, 1)
				}
			}
			prim.indices = append(prim.indices, idx)
		}
		if len(prim.indices) > 0 {
			prims = append(prims, prim)
		}
	}
	return prims
}

// writeGLB emits a glTF 2.0 binary with one mesh and one primitive per material.
func writeGLB(name string, prims []primitive) []byte {
	var bin bytes.Buffer
	var bufferViews []map[string]any
	var accessors []map[string]any
	var materials []map[string]any
	var gltfPrims []map[string]any

	pad := func() {
		for bin.Len()%4 != 0 {
			bin.WriteByte(0)
		}
	}
	addView := func(data []byte, target int) int {
		pad()
		off := bin.Len()
		bin.Write(data)
		v := map[string]any{"buffer": 0, "byteOffset": off, "byteLength": len(data)}
		if target != 0 {
			v["target"] = target
		}
		bufferViews = append(bufferViews, v)
		return len(bufferViews) - 1
	}

	for _, p := range prims {
		// positions
		var pb bytes.Buffer
		_ = binary.Write(&pb, binary.LittleEndian, p.positions)
		minV := []float32{math.MaxFloat32, math.MaxFloat32, math.MaxFloat32}
		maxV := []float32{-math.MaxFloat32, -math.MaxFloat32, -math.MaxFloat32}
		for i := 0; i+2 < len(p.positions); i += 3 {
			for k := range 3 {
				minV[k] = min(minV[k], p.positions[i+k])
				maxV[k] = max(maxV[k], p.positions[i+k])
			}
		}
		pv := addView(pb.Bytes(), 34962)
		accessors = append(accessors, map[string]any{
			"bufferView": pv, "componentType": 5126, "count": len(p.positions) / 3, "type": "VEC3", "min": minV, "max": maxV,
		})
		posAcc := len(accessors) - 1

		var nb bytes.Buffer
		_ = binary.Write(&nb, binary.LittleEndian, p.normals)
		nv := addView(nb.Bytes(), 34962)
		accessors = append(accessors, map[string]any{
			"bufferView": nv, "componentType": 5126, "count": len(p.normals) / 3, "type": "VEC3",
		})
		nrmAcc := len(accessors) - 1

		var ib bytes.Buffer
		_ = binary.Write(&ib, binary.LittleEndian, p.indices)
		iv := addView(ib.Bytes(), 34963)
		accessors = append(accessors, map[string]any{
			"bufferView": iv, "componentType": 5125, "count": len(p.indices), "type": "SCALAR",
		})
		idxAcc := len(accessors) - 1

		materials = append(materials, map[string]any{
			"name": p.name,
			"pbrMetallicRoughness": map[string]any{
				"baseColorFactor": p.color, "metallicFactor": 0.1, "roughnessFactor": 0.7,
			},
		})
		gltfPrims = append(gltfPrims, map[string]any{
			"attributes": map[string]any{"POSITION": posAcc, "NORMAL": nrmAcc},
			"indices":    idxAcc,
			"material":   len(materials) - 1,
			"mode":       4,
		})
	}
	pad()

	doc := map[string]any{
		"asset":       map[string]any{"version": "2.0", "generator": "rainbow-robotics dae2glb"},
		"scene":       0,
		"scenes":      []map[string]any{{"nodes": []int{0}}},
		"nodes":       []map[string]any{{"mesh": 0, "name": name}},
		"meshes":      []map[string]any{{"name": name, "primitives": gltfPrims}},
		"materials":   materials,
		"accessors":   accessors,
		"bufferViews": bufferViews,
		"buffers":     []map[string]any{{"byteLength": bin.Len()}},
	}
	jsonBytes, err := json.Marshal(doc)
	if err != nil {
		fatal(err)
	}
	for len(jsonBytes)%4 != 0 {
		jsonBytes = append(jsonBytes, ' ')
	}

	var out bytes.Buffer
	total := 12 + 8 + len(jsonBytes) + 8 + bin.Len()
	_ = binary.Write(&out, binary.LittleEndian, uint32(0x46546C67)) // "glTF"
	_ = binary.Write(&out, binary.LittleEndian, uint32(2))
	_ = binary.Write(&out, binary.LittleEndian, uint32(total))
	_ = binary.Write(&out, binary.LittleEndian, uint32(len(jsonBytes)))
	_ = binary.Write(&out, binary.LittleEndian, uint32(0x4E4F534A)) // JSON
	out.Write(jsonBytes)
	_ = binary.Write(&out, binary.LittleEndian, uint32(bin.Len()))
	_ = binary.Write(&out, binary.LittleEndian, uint32(0x004E4942)) // BIN
	out.Write(bin.Bytes())
	return out.Bytes()
}
