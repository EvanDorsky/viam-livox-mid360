#include <string.h>
#include "cbridge.h"
#include "livox_lidar_api.h"
#include "_cgo_export.h"

static void pointCloudCb(const uint32_t handle, const uint8_t dev_type,
                          LivoxLidarEthernetPacket *data, void *client_data) {
    if (data) {
        goPointCloudCallback(handle, dev_type, data);
    }
}

static void imuCb(const uint32_t handle, const uint8_t dev_type,
                   LivoxLidarEthernetPacket *data, void *client_data) {
    if (data) {
        goImuCallback(handle, dev_type, data);
    }
}

static void infoChangeCb(const uint32_t handle, const LivoxLidarInfo *info,
                          void *client_data) {
    if (info) {
        goInfoChangeCallback(handle, (LivoxLidarInfo *)info);
    }
}

int bridge_init(const char *cfg_path, const char *host_ip) {
    return LivoxLidarSdkInit(cfg_path, host_ip, nullptr) ? 0 : -1;
}

int bridge_start(void) {
    return LivoxLidarSdkStart() ? 0 : -1;
}

void bridge_register_callbacks(void) {
    SetLivoxLidarPointCloudCallBack(pointCloudCb, nullptr);
    SetLivoxLidarImuDataCallback(imuCb, nullptr);
    SetLivoxLidarInfoChangeCallback(infoChangeCb, nullptr);
}

void bridge_uninit(void) {
    LivoxLidarSdkUninit();
}

int bridge_set_work_mode(uint32_t handle, int mode) {
    return SetLivoxLidarWorkMode(handle, (LivoxLidarWorkMode)mode, nullptr, nullptr);
}

int bridge_enable_imu(uint32_t handle) {
    return EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

uint8_t pkt_frame_cnt(void *pkt) {
    return ((LivoxLidarEthernetPacket *)pkt)->frame_cnt;
}

uint16_t pkt_dot_num(void *pkt) {
    return ((LivoxLidarEthernetPacket *)pkt)->dot_num;
}

uint8_t pkt_data_type(void *pkt) {
    return ((LivoxLidarEthernetPacket *)pkt)->data_type;
}

uint8_t pkt_time_type(void *pkt) {
    return ((LivoxLidarEthernetPacket *)pkt)->time_type;
}

void pkt_timestamp(void *pkt, uint8_t *out) {
    memcpy(out, ((LivoxLidarEthernetPacket *)pkt)->timestamp, 8);
}

void *pkt_data(void *pkt) {
    return ((LivoxLidarEthernetPacket *)pkt)->data;
}
