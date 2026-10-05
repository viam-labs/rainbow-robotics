// Package main is the entrypoint of the rainbow-robotics Viam module.
package main

import (
	rdkarm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"

	"github.com/viam-labs/rainbow-robotics/arm"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: rdkarm.API, Model: arm.Model},
		resource.APIModel{API: rdkarm.API, Model: arm.SimulatedModel},
	)
}
