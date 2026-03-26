#ifndef CBRIDGE_H
#define CBRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// SDK lifecycle
int bridge_init(const char* cfg_path, const char* host_ip);
int bridge_start(void);
void bridge_register_callbacks(void);
void bridge_uninit(void);

// Device control
int bridge_set_work_mode(uint32_t handle, int mode);
int bridge_enable_imu(uint32_t handle);

// Packet field accessors (packed struct fields aren't accessible via cgo)
uint8_t  pkt_frame_cnt(void* pkt);
uint16_t pkt_dot_num(void* pkt);
uint8_t  pkt_data_type(void* pkt);
uint8_t  pkt_time_type(void* pkt);
void     pkt_timestamp(void* pkt, uint8_t* out);
void*    pkt_data(void* pkt);

#ifdef __cplusplus
}
#endif

#endif // CBRIDGE_H
