// Command stlbounds prints the axis-aligned bounding box of binary STL files in
// millimetres. It was used to size the capsules in arm/rb10_1300_kinematics.json
// from the rbpodo_ros2 collision meshes.
//
//	go run ./tools/stlbounds arm/meshes/rb10_1300e/collision/*.stl
package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
)

func main() {
	if len(os.Args) < 2 {
		fmt.Fprintln(os.Stderr, "usage: stlbounds file.stl ...")
		os.Exit(2)
	}
	for _, path := range os.Args[1:] {
		data, err := os.ReadFile(path)
		if err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		if len(data) < 84 {
			fmt.Fprintf(os.Stderr, "%s: not a binary STL\n", path)
			continue
		}
		n := int(binary.LittleEndian.Uint32(data[80:84]))
		minV := [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
		maxV := [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
		off := 84
		for i := 0; i < n && off+50 <= len(data); i++ {
			for v := range 3 {
				base := off + 12 + v*12
				for c := range 3 {
					f := float64(math.Float32frombits(binary.LittleEndian.Uint32(data[base+c*4:])))
					minV[c] = min(minV[c], f)
					maxV[c] = max(maxV[c], f)
				}
			}
			off += 50
		}
		fmt.Printf("%s: tris=%d min=[%.1f %.1f %.1f] max=[%.1f %.1f %.1f] extent=[%.1f %.1f %.1f] center=[%.1f %.1f %.1f] (mm)\n",
			path, n,
			minV[0]*1000, minV[1]*1000, minV[2]*1000,
			maxV[0]*1000, maxV[1]*1000, maxV[2]*1000,
			(maxV[0]-minV[0])*1000, (maxV[1]-minV[1])*1000, (maxV[2]-minV[2])*1000,
			(maxV[0]+minV[0])*500, (maxV[1]+minV[1])*500, (maxV[2]+minV[2])*500)
	}
}
