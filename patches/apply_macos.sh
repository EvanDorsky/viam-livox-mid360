#!/bin/bash
# Apply macOS compatibility patches to Livox-SDK2 source (build-time only)
set -e

SDK_DIR="$1"
if [ -z "$SDK_DIR" ]; then
    echo "Usage: $0 <sdk-dir>"
    exit 1
fi

DM="$SDK_DIR/sdk_core/device_manager.cpp"
CM="$SDK_DIR/sdk_core/CMakeLists.txt"

# 1. Remove -Werror for Apple clang
sed -i.bak 's/-Werror //g' "$CM"

# 2. Patch device_manager.cpp for macOS
cp "$DM" "$DM.bak2"

python3 - "$DM" << 'PYEOF'
import sys

with open(sys.argv[1], 'r') as f:
    src = f.read()

# Fix 1: Bind detection broadcast socket to 0.0.0.0 instead of 255.255.255.255
src = src.replace(
    'util::CreateSocket(kDetectionPort, true, true, true, "255.255.255.255", "")',
    'util::CreateSocket(kDetectionPort, true, true, true, "0.0.0.0", "")'
)

# Fix 2: Make detection broadcast socket failure non-fatal
src = src.replace(
    '''  if (detection_broadcast_socket_ < 0) {
    LOG_ERROR("Create detection broadcast socket failed.");
    return false;
  }''',
    '''  if (detection_broadcast_socket_ < 0) {
    LOG_ERROR("Create detection broadcast socket failed (non-fatal on macOS).");
    // Non-fatal: continue without broadcast detection
  }'''
)

# Fix 3: Replace Detection() to send directly to configured lidar IPs instead of broadcasting
old_detection = '''void DeviceManager::Detection() {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};

  CommPacket packet;
  packet.protocol = kLidarSdk;
  packet.version = kSdkVer;
  packet.seq_num = GenerateSeq::GetSeq();
  packet.cmd_id = kCommandIDLidarSearch;
  packet.cmd_type = kCommandTypeCmd;
  packet.sender_type = kHostSend;
  packet.data = req_buff;
  packet.data_len = 0;

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
  servaddr.sin_port = htons(kDetectionPort);

  int byte_send = sendto(detection_socket_, (const char*)buf.data(), size, 0,
      (const struct sockaddr *) &servaddr, sizeof(servaddr));
  if (byte_send < 0) {
    LOG_INFO("Detection lidars failed, Send to lidar failed.");
  }
}'''

new_detection = '''void DeviceManager::Detection() {
  uint8_t req_buff[kMaxCommandBufferSize] = {0};

  CommPacket packet;
  packet.protocol = kLidarSdk;
  packet.version = kSdkVer;
  packet.seq_num = GenerateSeq::GetSeq();
  packet.cmd_id = kCommandIDLidarSearch;
  packet.cmd_type = kCommandTypeCmd;
  packet.sender_type = kHostSend;
  packet.data = req_buff;
  packet.data_len = 0;

  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, packet);

  // macOS: send discovery directly to each configured lidar IP (broadcast to 255.255.255.255 fails on macOS)
  if (lidars_cfg_ptr_) {
    for (const auto& cfg : *lidars_cfg_ptr_) {
      if (cfg.lidar_net_info.lidar_ipaddr.empty()) continue;
      struct sockaddr_in servaddr;
      servaddr.sin_family = AF_INET;
      servaddr.sin_addr.s_addr = inet_addr(cfg.lidar_net_info.lidar_ipaddr.c_str());
      servaddr.sin_port = htons(kDetectionPort);
      int byte_send = sendto(detection_socket_, (const char*)buf.data(), size, 0,
          (const struct sockaddr *) &servaddr, sizeof(servaddr));
      if (byte_send < 0) {
        LOG_INFO("Detection send to {} failed.", cfg.lidar_net_info.lidar_ipaddr.c_str());
      }
    }
  }
}'''

if old_detection in src:
    src = src.replace(old_detection, new_detection)
    print("Patched Detection() to use direct IP send")
else:
    print("WARNING: Could not find Detection() to patch - may need manual fix")

with open(sys.argv[1], 'w') as f:
    f.write(src)

PYEOF
echo "$DM"

echo "macOS patches applied."
