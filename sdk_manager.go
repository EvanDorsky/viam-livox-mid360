package main

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sync"
)

type sdkManager struct {
	mu       sync.Mutex
	refCount int
	handles  map[uint32]bool
	tmpDir   string
}

var sdkMgr = &sdkManager{
	handles: make(map[uint32]bool),
}

func (m *sdkManager) Acquire(cfg *Config) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.refCount++
	if m.refCount > 1 {
		return nil
	}

	cfgPath, err := m.generateConfig(cfg)
	if err != nil {
		m.refCount--
		return fmt.Errorf("generate SDK config: %w", err)
	}

	hostIP := cfg.HostIP
	if err := bridgeInit(cfgPath, hostIP); err != nil {
		m.refCount--
		return fmt.Errorf("SDK init: %w", err)
	}

	bridgeRegisterCallbacks()

	if err := bridgeStart(); err != nil {
		bridgeUninit()
		m.refCount--
		return fmt.Errorf("SDK start: %w", err)
	}

	return nil
}

func (m *sdkManager) Release() {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.refCount--
	if m.refCount <= 0 {
		bridgeUninit()
		m.refCount = 0
		if m.tmpDir != "" {
			os.RemoveAll(m.tmpDir)
			m.tmpDir = ""
		}
	}
}

func (m *sdkManager) addHandle(handle uint32) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.handles[handle] = true
}

func (m *sdkManager) generateConfig(cfg *Config) (string, error) {
	// Match the SDK2 sample mid360_config.json format
	sdkCfg := map[string]interface{}{
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
					"host_ip":         cfg.HostIP,
					"cmd_data_port":   56101,
					"push_msg_port":   56201,
					"point_data_port": 56301,
					"imu_data_port":   56401,
					"log_data_port":   56501,
					"lidar_ip":        []string{cfg.SensorIP},
				},
			},
		},
	}

	dir, err := os.MkdirTemp("", "livox-mid360-*")
	if err != nil {
		return "", err
	}
	m.tmpDir = dir

	path := filepath.Join(dir, "config.json")
	data, err := json.MarshalIndent(sdkCfg, "", "  ")
	if err != nil {
		return "", err
	}

	return path, os.WriteFile(path, data, 0644)
}
