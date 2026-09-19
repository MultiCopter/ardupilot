/*
  Custom Serial IMU backend for AP_InertialSensor

  Protocol: RS-422, 921600 bps, 8N1, 1000 Hz output
  Frame: 38 bytes (measured; doc said 39 but that includes a phantom
  trailing CRC16 byte that does not exist on the wire)
    [0]     0xAA            header byte 0
    [1]     0x55            header byte 1
    [2]     0xEB            header byte 2
    [3]     0x90            header byte 3
    [4-7]   uint32  LE      output counter
    [8-11]  float   LE      Gx  (deg/s)
    [12-15] float   LE      Gy  (deg/s)
    [16-19] float   LE      Gz  (deg/s)
    [20-23] float   LE      Ax  (m/s^2)
    [24-27] float   LE      Ay  (m/s^2)
    [28-31] float   LE      Az  (m/s^2)
    [32-33] int16   LE      Temp (degC * 100)
    [34-36] --             reserved
    [37]    uint8           SUM8 = sum(bytes[0..36]) & 0xFF
                           (doc said CRC16 Modbus 0xA001; measured SUM8 over 2172 frames)
*/

#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_InertialSensor_CustomSerialIMU : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_CustomSerialIMU(AP_InertialSensor &imu, int8_t serial_port);

    /* probe function - returns nullptr if sensor not detected */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu);

    /* probe-failure broadcast state.
       When probe() fails, the failure text is stashed here so that
       AP_InertialSensor::update() can re-emit it via GCS_SEND_TEXT
       at 1Hz for up to 60s. The boot-time STATUSTEXT from probe()
       is often discarded because the GCS hasn't connected yet.
    */
    static const char *get_pending_probe_failure();
    static bool tick_probe_failure_broadcast(uint32_t now_ms);
    static void clear_pending_probe_failure();

    /* sensor frontend interface */
    bool update() override;
    void start() override;
    void accumulate() override;

    bool get_output_banner(char *banner, uint8_t banner_len) override;

private:
    static constexpr uint8_t FRAME_SIZE = 38;  // measured: 4 hdr + 4 cnt + 24 data + 2 temp + 3 reserved + 1 bytesum(SUM8)
    static constexpr float TEMP_SCALE  = 0.01f;  // int16 / 100 = degC

    int8_t serial_port;
    AP_HAL::UARTDriver *uart = nullptr;
    bool started;

    /* receive buffer */
    uint8_t rxbuf[FRAME_SIZE];
    uint16_t rxbuf_pos;

    /* accumulated samples for decimation to IMU rate */
    struct {
        uint64_t last_update_us;
        Vector3f gyro;
        Vector3f accel;
        float temp;
    } accum;

    /* pending probe-failure message for GCS_SEND_TEXT re-broadcast */
    static char _probe_failure_msg[128];
    static uint32_t _probe_failure_last_send_ms;
    static uint8_t _probe_failure_send_count;
    static bool _probe_failure_pending;

    /* parse one complete frame, return true if valid */
    bool parse_frame(const uint8_t *frame);

    /* push a byte into the ring buffer and look for frame sync */
    void handle_byte(uint8_t b);

    /* decimation counter - 1 = "at least one new frame since last update" */
    uint16_t decimate_counter;
};
