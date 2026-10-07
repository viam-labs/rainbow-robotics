# Pure Go, statically linked: the module never plans in-process, so it has no
# nlopt/cgo dependency and runs on machines without libnlopt.
GO_BUILD_ENV := CGO_ENABLED=0
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/rainbow-robotics

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	MODULE_BINARY = bin/rainbow-robotics.exe
endif

GO_FILES := $(shell find . -name '*.go' -not -path './tools/*')
DATA_FILES := arm/rb10_1300e.urdf arm/meshes arm/3d_models THIRD_PARTY_NOTICES

.PHONY: build lint test module clean setup update tools

build: $(MODULE_BINARY)

$(MODULE_BINARY): Makefile go.mod go.sum $(GO_FILES) arm/rb10_1300_kinematics.json rb/error_code.csv
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/module

lint:
	gofmt -s -l . | tee /dev/stderr | test -z "$$(cat)"
	go vet ./...
	golangci-lint run -c etc/.golangci.yaml ./...

test:
	go test -race ./...

# Tests that talk to a real control box; set RB_HOST=10.0.2.7 to enable.
test-hardware:
	RB_HOST=$(RB_HOST) go test -race -run Hardware -v ./arm/

module.tar.gz: meta.json $(MODULE_BINARY) $(DATA_FILES)
ifneq ($(VIAM_TARGET_OS), windows)
	strip $(MODULE_BINARY)
endif
	tar czf $@ meta.json $(MODULE_BINARY) $(DATA_FILES)

bin/module.tar.gz: module.tar.gz
	mkdir -p bin && cp module.tar.gz bin/module.tar.gz

module: test bin/module.tar.gz

all: lint module

# Regenerate the GLB viewer meshes from an rbpodo_ros2 checkout.
tools:
	go run ./tools/dae2glb -in $(RBPODO_ROS2)/rbpodo_description/meshes/rb10_1300e/visual -out arm/3d_models/rb10-1300
	go run ./tools/glbrebase -kinematics arm/rb10_1300_kinematics.json -dir arm/3d_models/rb10-1300

clean:
	rm -rf bin module.tar.gz

setup:
	go mod tidy

update:
	go get go.viam.com/rdk@latest
	go mod tidy
