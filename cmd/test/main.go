// Standalone smoke test for the Livox Mid-360 SDK bridge.
// Connects to a lidar, prints point cloud and IMU stats.
// Usage: go run ./cmd/test -sensor-ip 192.168.1.196 -host-ip 192.168.1.10
package main

/*
#cgo CFLAGS: -I${SRCDIR}/../../third_party/Livox-SDK2/include
#cgo CXXFLAGS: -I${SRCDIR}/../../third_party/Livox-SDK2/include -std=c++11
#cgo LDFLAGS: -L${SRCDIR}/../../third_party/Livox-SDK2/build/sdk_core -llivox_lidar_sdk_static -lstdc++ -lpthread
#include <stdlib.h>
#include <stdbool.h>
#include "../../cbridge.h"
#include "livox_lidar_def.h"
*/
import "C"
import (
	"encoding/binary"
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"os/signal"
	"path/filepath"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"
)

type point struct {
	x, y, z      int32
	reflectivity uint8
}

type imu struct {
	gyroX, gyroY, gyroZ float32
	accX, accY, accZ    float32
	timestamp           uint64
}

var (
	mu             sync.Mutex
	writing        []point
	reading        []point
	frameCnt       uint8
	frameReady     = make(chan struct{}, 1)
	initialized    bool
	frameCount     uint64
	totalPoints    uint64
	writeTimestamp uint64
	readTimestamp  uint64

	latestIMU  atomic.Value
	imuCount   uint64
	connected  int32
	deviceHandle uint32
)

//export goPointCloudCallback
func goPointCloudCallback(handle C.uint32_t, devType C.uint8_t, data *C.LivoxLidarEthernetPacket) {
	pkt := unsafe.Pointer(data)
	fc := uint8(C.pkt_frame_cnt(pkt))
	dotNum := uint16(C.pkt_dot_num(pkt))
	dataType := uint8(C.pkt_data_type(pkt))

	var tsBuf [8]byte
	C.pkt_timestamp(pkt, (*C.uint8_t)(unsafe.Pointer(&tsBuf[0])))
	ts := binary.LittleEndian.Uint64(tsBuf[:])

	if dataType != C.kLivoxLidarCartesianCoordinateHighData {
		return
	}

	mu.Lock()
	defer mu.Unlock()

	if initialized && fc != frameCnt {
		reading = writing
		writing = nil
		readTimestamp = writeTimestamp
		atomic.AddUint64(&frameCount, 1)
		atomic.AddUint64(&totalPoints, uint64(len(reading)))
		select {
		case frameReady <- struct{}{}:
		default:
		}
	}

	frameCnt = fc
	initialized = true
	if len(writing) == 0 {
		writeTimestamp = ts
	}

	base := C.pkt_data(pkt)
	pointSize := C.sizeof_LivoxLidarCartesianHighRawPoint
	for i := uint16(0); i < dotNum; i++ {
		p := (*C.LivoxLidarCartesianHighRawPoint)(unsafe.Add(base, uintptr(i)*uintptr(pointSize)))
		writing = append(writing, point{
			x:            int32(p.x),
			y:            int32(p.y),
			z:            int32(p.z),
			reflectivity: uint8(p.reflectivity),
		})
	}
}

//export goImuCallback
func goImuCallback(handle C.uint32_t, devType C.uint8_t, data *C.LivoxLidarEthernetPacket) {
	pkt := unsafe.Pointer(data)
	if uint8(C.pkt_data_type(pkt)) != C.kLivoxLidarImuData {
		return
	}

	var tsBuf [8]byte
	C.pkt_timestamp(pkt, (*C.uint8_t)(unsafe.Pointer(&tsBuf[0])))
	ts := binary.LittleEndian.Uint64(tsBuf[:])

	p := (*C.LivoxLidarImuRawPoint)(C.pkt_data(pkt))
	latestIMU.Store(&imu{
		gyroX:     float32(p.gyro_x),
		gyroY:     float32(p.gyro_y),
		gyroZ:     float32(p.gyro_z),
		accX:      float32(p.acc_x),
		accY:      float32(p.acc_y),
		accZ:      float32(p.acc_z),
		timestamp: ts,
	})
	atomic.AddUint64(&imuCount, 1)
}

//export goInfoChangeCallback
func goInfoChangeCallback(handle C.uint32_t, info *C.LivoxLidarInfo) {
	sn := C.GoStringN(&info.sn[0], 16)
	ip := C.GoStringN(&info.lidar_ip[0], 16)
	fmt.Printf("[CONNECT] handle=%d dev_type=%d sn=%s ip=%s\n",
		uint32(handle), uint8(info.dev_type), sn, ip)
	atomic.StoreInt32(&connected, 1)
	atomic.StoreUint32(&deviceHandle, uint32(handle))

	// Set to normal mode to enable data streaming
	C.bridge_set_work_mode(handle, C.int(C.kLivoxLidarNormal))
	// Enable IMU data
	C.bridge_enable_imu(handle)
}

func generateConfig(sensorIP, hostIP string) (string, error) {
	cfg := map[string]interface{}{
		"MID360": map[string]interface{}{
			"lidar_net_info": map[string]interface{}{
				"cmd_data_port":   56100,
				"push_msg_port":   56200,
				"point_data_port": 56300,
				"imu_data_port":   56400,
				"log_data_port":   56500,
			},
			"host_net_info": []map[string]interface{}{
				{
					"host_ip":         hostIP,
					"multicast_ip":    "224.1.1.5",
					"cmd_data_port":   56101,
					"push_msg_port":   56201,
					"point_data_port": 56301,
					"imu_data_port":   56401,
					"log_data_port":   56501,
					"lidar_ip":        []string{sensorIP},
				},
			},
		},
	}

	dir, err := os.MkdirTemp("", "livox-test-*")
	if err != nil {
		return "", err
	}

	path := filepath.Join(dir, "config.json")
	data, err := json.MarshalIndent(cfg, "", "  ")
	if err != nil {
		return "", err
	}

	fmt.Printf("[CONFIG] %s\n%s\n", path, string(data))
	return path, os.WriteFile(path, data, 0644)
}

func main() {
	sensorIP := flag.String("sensor-ip", "192.168.1.196", "Lidar IP address")
	hostIP := flag.String("host-ip", "192.168.1.10", "Host IP address on lidar subnet")
	duration := flag.Duration("duration", 10*time.Second, "How long to collect data")
	flag.Parse()

	fmt.Printf("Livox Mid-360 smoke test\n")
	fmt.Printf("  Sensor: %s\n  Host:   %s\n  Duration: %s\n\n", *sensorIP, *hostIP, *duration)

	cfgPath, err := generateConfig(*sensorIP, *hostIP)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Failed to generate config: %v\n", err)
		os.Exit(1)
	}

	cPath := C.CString(cfgPath)
	defer C.free(unsafe.Pointer(cPath))
	cHost := C.CString(*hostIP)
	defer C.free(unsafe.Pointer(cHost))

	fmt.Println("[SDK] Initializing...")
	if C.bridge_init(cPath, cHost) != 0 {
		fmt.Fprintf(os.Stderr, "SDK init failed\n")
		os.Exit(1)
	}
	defer func() {
		fmt.Println("[SDK] Shutting down...")
		C.bridge_uninit()
		fmt.Println("[SDK] Done.")
	}()

	fmt.Println("[SDK] Registering callbacks...")
	C.bridge_register_callbacks()

	fmt.Println("[SDK] Starting...")
	if C.bridge_start() != 0 {
		fmt.Fprintf(os.Stderr, "SDK start failed\n")
		os.Exit(1)
	}

	// Wait for connection or timeout
	fmt.Println("[SDK] Waiting for device...")
	connectTimeout := time.After(10 * time.Second)
	connectTick := time.NewTicker(100 * time.Millisecond)
	defer connectTick.Stop()

waitConnect:
	for {
		select {
		case <-connectTimeout:
			fmt.Fprintf(os.Stderr, "Timed out waiting for device connection\n")
			return
		case <-connectTick.C:
			if atomic.LoadInt32(&connected) == 1 {
				fmt.Println("[SDK] Device connected!")
				break waitConnect
			}
		}
	}

	// Collect data for the specified duration
	fmt.Printf("\nCollecting data for %s...\n\n", *duration)

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)

	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()
	deadline := time.After(*duration)
	start := time.Now()
	lastFrameCount := uint64(0)
	lastIMUCount := uint64(0)

	for {
		select {
		case <-sig:
			fmt.Println("\nInterrupted.")
			return
		case <-deadline:
			fmt.Println("\nCollection complete.")
			printSummary(start)
			return
		case <-ticker.C:
			fc := atomic.LoadUint64(&frameCount)
			ic := atomic.LoadUint64(&imuCount)
			tp := atomic.LoadUint64(&totalPoints)

			frameDelta := fc - lastFrameCount
			imuDelta := ic - lastIMUCount
			lastFrameCount = fc
			lastIMUCount = ic

			avgPts := uint64(0)
			if fc > 0 {
				avgPts = tp / fc
			}

			fmt.Printf("[%5.1fs] frames: %d (+%d/s, ~%d pts/frame)  imu: %d (+%d/s)",
				time.Since(start).Seconds(), fc, frameDelta, avgPts, ic, imuDelta)

			// Print latest IMU reading
			if v := latestIMU.Load(); v != nil {
				d := v.(*imu)
				accMag := math.Sqrt(float64(d.accX*d.accX + d.accY*d.accY + d.accZ*d.accZ))
				fmt.Printf("  acc=[%.2f,%.2f,%.2f]g (|%.2f|g)  gyro=[%.3f,%.3f,%.3f]rad/s",
					d.accX, d.accY, d.accZ, accMag, d.gyroX, d.gyroY, d.gyroZ)
			}
			fmt.Println()

			// Print a sample of point data from the latest frame
			if frameDelta > 0 {
				mu.Lock()
				if len(reading) > 0 {
					printPointSample(reading)
				}
				mu.Unlock()
			}
		}
	}
}

func printPointSample(pts []point) {
	n := len(pts)
	indices := []int{0, n / 4, n / 2, 3 * n / 4, n - 1}
	fmt.Printf("        sample points (%d total):", n)
	for _, i := range indices {
		if i < n {
			p := pts[i]
			fmt.Printf(" [%.2f,%.2f,%.2f]m",
				float64(p.x)/1000.0, float64(p.y)/1000.0, float64(p.z)/1000.0)
		}
	}
	fmt.Println()
}

func printSummary(start time.Time) {
	elapsed := time.Since(start)
	fc := atomic.LoadUint64(&frameCount)
	tp := atomic.LoadUint64(&totalPoints)
	ic := atomic.LoadUint64(&imuCount)

	fmt.Printf("\n=== SUMMARY ===\n")
	fmt.Printf("Duration:      %.1fs\n", elapsed.Seconds())
	fmt.Printf("Frames:        %d (%.1f Hz)\n", fc, float64(fc)/elapsed.Seconds())
	fmt.Printf("Total points:  %d\n", tp)
	if fc > 0 {
		fmt.Printf("Avg pts/frame: %d\n", tp/fc)
	}
	fmt.Printf("IMU readings:  %d (%.0f Hz)\n", ic, float64(ic)/elapsed.Seconds())

	mu.Lock()
	ts := readTimestamp
	mu.Unlock()
	fmt.Printf("Last frame ts: %d ns\n", ts)
}
