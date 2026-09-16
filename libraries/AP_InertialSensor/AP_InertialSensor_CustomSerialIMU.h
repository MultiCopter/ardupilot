/*
  Custom Serial IMU backend for AP_InertialSensor

  Protocol: RS-422, 921600 bps, 8N1, 1000 Hz output
  Frame: 39 bytes
    [0-1]   0xAA55          header byte 0-1
    [2-3]   0xEB90          header byte 2-3
    [4-7]   uint32          output counter
    [8-11]  float           Gx  (°/s)
    [12-15] float           Gy  (°/s)
    [16-19] float           Gz  (°/s)
    [20-23] float           Ax  (m/s²)
    [24-27] float           Ay  (m/s²)
    [28-31] float           Az  (m/s²)
    [32-33] int16           Temp (℃ × 100)
    [34-36] --             reserved
    [37-38] uint16         CRC16 (bytes 0-36, Modbus/RTU polynomial 0x8005)
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

    /* sensor frontend interface */
    bool update() override;
    void start() override;
    void accumulate() override;

    bool get_output_banner(char *banner, uint8_t banner_len) override;

private:
    static constexpr uint8_t FRAME_SIZE   = 39;
    static constexpr uint16_t FRAME_HEADER = 0x55AA;  // bytes 0-1
    static constexpr uint16_t FRAME_HEADER2 = 0x90EB; // bytes 2-3 (little-endian)

    /* protocol constants */
    static constexpr float GYRO_SCALE   = 1.0f;    // already °/s
    static constexpr float ACCEL_SCALE  = 1.0f;    // already m/s²
    static constexpr float TEMP_SCALE   = 0.01f;   // int16 ÷ 100 = ℃

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

    /* CRC16 (Modbus RTU polynomial 0x8005) */
    uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

    /* parse one complete frame, return true if valid */
    bool parse_frame(const uint8_t *frame);

    /* push a byte into the ring buffer and look for frame sync */
    void handle_byte(uint8_t b);

    /* decimation counter */
    uint16_t decimate_counter;
    static constexpr uint16_t DECIMATION = 10; // 1000Hz / 10 = 100 Hz to frontend
};
