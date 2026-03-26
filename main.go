package main

import (
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: camera.API, Model: cameraModel},
		resource.APIModel{API: movementsensor.API, Model: imuModel},
	)
}
