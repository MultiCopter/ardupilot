/*
  Custom Serial IMU backend for AP_InertialSensor

  Protocol: RS-422, 921600 bps, 8N1, 1000 Hz output
  Frame: 38 bytes
    [0]     0xAA           header byte 0
    [1]     0x55           header byte 1
    [2]     0xEB           header byte 2
    [3]     0x90           header byte 3
    [4-7]   uint32 LE      output counter
    [8-11]  float LE       Gx  (°/s)
    [12-15] float LE       Gy  (°/s)
    [16-19] float LE       Gz  (°/s)
    [20-23] float LE       Ax  (m/s²)
    [24-27] float LE       Ay  (m/s²)
    [28-31] float LE       Az  (m/s²)
    [32-33] int16 LE       Temp (℃ × 100)
    [34-36] --            reserved (00H)
    [37]    uint8          SUM8 = (sum of bytes 0..36) & 0xFF
                          NOTE: doc says "CRC16 0xA001" but IMU emits a 1-byte 8-bit sum
*/

#include "AP_InertialSensor_CustomSerialIMU.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// =====================================================================
// Static probe-failure broadcast state
// When probe() fails, the failure text is stashed here so that
// AP_InertialSensor::update() can re-emit it via GCS_SEND_TEXT at 1Hz
// for up to 60s. The boot-time STATUSTEXT from probe() is often
// discarded because the GCS hasn't connected yet.
// =====================================================================
char AP_InertialSensor_CustomSerialIMU::_probe_failure_msg[128] = {0};
uint32_t AP_InertialSensor_CustomSerialIMU::_probe_failure_last_send_ms = 0;
uint8_t AP_InertialSensor_CustomSerialIMU::_probe_failure_send_count = 0;
bool AP_InertialSensor_CustomSerialIMU::_probe_failure_pending = false;

const char *AP_InertialSensor_CustomSerialIMU::get_pending_probe_failure()
{
    return _probe_failure_pending ? _probe_failure_msg : nullptr;
}

bool AP_InertialSensor_CustomSerialIMU::tick_probe_failure_broadcast(uint32_t now_ms)
{
    if (!_probe_failure_pending) {
        return false;
    }
    if (_probe_failure_send_count >= 60) {  // stop after 60 ticks (~60s)
        return false;
    }
    if (now_ms - _probe_failure_last_send_ms < 1000) {
        return false;
    }
    _probe_failure_last_send_ms = now_ms;
    _probe_failure_send_count++;
    return true;
}

void AP_InertialSensor_CustomSerialIMU::clear_pending_probe_failure()
{
    _probe_failure_pending = false;
    _probe_failure_msg[0] = 0;
    _probe_failure_send_count = 0;
    _probe_failure_last_send_ms = 0;
}

/*
  Read a little-endian float from a byte buffer.
  Avoids aliasing UB by using memcpy.
*/
static inline float read_float_le(const uint8_t *p)
{
    float v;
    memcpy(&v, p, sizeof(v));
    return v;
}

/*
  Read a little-endian int16 from a byte buffer.
*/
static inline int16_t read_int16_le(const uint8_t *p)
{
    return int16_t(p[0] | (p[1] << 8));
}

/*
  Parse one complete 38-byte frame.
  Frame is little-endian (LSB first).
  Returns true if frame is valid and data is extracted into accum.
*/
bool AP_InertialSensor_CustomSerialIMU::parse_frame(const uint8_t *frame)
{
    // Header: 0xAA 0x55 0xEB 0x90
    if (frame[0] != 0xAA || frame[1] != 0x55 ||
        frame[2] != 0xEB || frame[3] != 0x90) {
        return false;
    }

    // Checksum (SUM8): sum of bytes 0..36, lowest byte
    // Measured from real IMU traffic: the doc's "CRC16 Modbus 0xA001" is actually
    // a 1-byte 8-bit sum (verified over 2172 captured frames at 100% match).
    uint8_t sum = 0;
    for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {  // 0..36
        sum += frame[i];
    }
    if (sum != frame[FRAME_SIZE - 1]) {  // frame[37]
        return false;
    }

    // Extract fields (all little-endian)
    float gx = read_float_le(&frame[8]);
    float gy = read_float_le(&frame[12]);
    float gz = read_float_le(&frame[16]);
    float ax = read_float_le(&frame[20]);
    float ay = read_float_le(&frame[24]);
    float az = read_float_le(&frame[28]);
    int16_t temp_raw = read_int16_le(&frame[32]);

    // Basic range sanity check (gyro: ±4000 °/s full scale typical)
    if (fabsf(gx) > 4000.0f || fabsf(gy) > 4000.0f || fabsf(gz) > 4000.0f) {
        return false;
    }

    // This IMU already outputs PHYSICAL units (gyro deg/s, accel m/s^2),
    // unlike ICM20689/ICM20602/BMI055 which output raw counts that need
    // scale-factor multiply + bias subtraction. So: no scale, no bias here.
    // Only convert gyro deg/s -> rad/s (EKF/AHRS expect rad/s and m/s^2).
    //
    // IMU->body axis remap (C1 fix): IMU +X=body +Z, IMU +Y=body +Y,
    // IMU +Z=body +X. The transform is det=-1 (improper reflection + a
    // permutation), so the framework's Rotation enum (det=+1 orthogonal
    // matrices only) cannot express it. We keep this manual remap and
    // declare the orientation as ROTATION_NONE in start(); if the IMU is
    // ever re-mounted so the mapping becomes det=+1, replace the manual
    // mapping with set_gyro_orientation/set_accel_orientation(<rot>).
    // Verified static: IMU ax=-1g -> body az=-1g, roll~0 (was 178 deg
    // before the az sign fix).
    accum.gyro  = Vector3f(gz, gy, gx) * DEG_TO_RAD;
    accum.accel = Vector3f(az, ay, ax);
    accum.temp  = temp_raw * TEMP_SCALE;
    accum.last_update_us = AP_HAL::micros64();

    return true;
}

/*
  Handle one incoming byte.
  Searches for frame sync (AA 55 EB 90) and fills rxbuf.
  When rxbuf is full (38 bytes), calls parse_frame().
*/
void AP_InertialSensor_CustomSerialIMU::handle_byte(uint8_t b)
{
    if (rxbuf_pos == 0) {
        // Waiting for 0xAA
        if (b != 0xAA) {
            return;
        }
        rxbuf[rxbuf_pos++] = b;
        return;
    }

    if (rxbuf_pos == 1) {
        // Waiting for 0x55
        if (b != 0x55) {
            rxbuf_pos = 0;
            if (b == 0xAA) { rxbuf[rxbuf_pos++] = b; }
            return;
        }
        rxbuf[rxbuf_pos++] = b;
        return;
    }

    if (rxbuf_pos == 2) {
        // Waiting for 0xEB
        if (b != 0xEB) {
            rxbuf_pos = 0;
            if (b == 0xAA) { rxbuf[rxbuf_pos++] = b; }
            return;
        }
        rxbuf[rxbuf_pos++] = b;
        return;
    }

    if (rxbuf_pos == 3) {
        // Waiting for 0x90
        if (b != 0x90) {
            rxbuf_pos = 0;
            if (b == 0xAA) { rxbuf[rxbuf_pos++] = b; }
            return;
        }
        rxbuf[rxbuf_pos++] = b;
        return;
    }

    // Fill remaining 34 bytes of frame
    rxbuf[rxbuf_pos++] = b;
    if (rxbuf_pos < FRAME_SIZE) {
        return;
    }

    // Frame complete (38 bytes received)
    if (parse_frame(rxbuf)) {
        decimate_counter++;
    }
    rxbuf_pos = 0;
}

/*
  Accumulate samples from UART and notify frontend of new data.
  Called frequently by the frontend accumulate() loop (run inside
  wait_for_sample()).

  IMPORTANT (C5): the _notify_new_*_raw_sample calls live HERE, not in
  update(). If notify/clear were both in update(), wait_for_sample()
  would always observe a cleared flag between cycles and deadlock when
  only one IMU is enabled (INS_ENABLE_MASK keeps only this bit).
*/
void AP_InertialSensor_CustomSerialIMU::accumulate()
{
    if (!started || uart == nullptr) {
        return;
    }

    int16_t n = uart->available();
    while (n-- > 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        handle_byte(b);
    }

    if (decimate_counter > 0) {
        // Notify frontend of new sample. _rotate_and_correct_* applies
        // board orientation + per-axis scale/bias calibration; then
        // _notify_new_*_raw_sample sets the per-instance _new_*_data
        // flag that wait_for_sample() polls.
        Vector3f accel = accum.accel;
        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel, accum.last_update_us);

        Vector3f gyro = accum.gyro;
        _rotate_and_correct_gyro(gyro_instance, gyro);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, accum.last_update_us);

        decimate_counter = 0;
    }
}

/*
  Publish accumulated sample to frontend.

  Note: _notify_new_*_raw_sample is called from accumulate(), NOT here
  (C5 fix). update_accel()/update_gyro() advance the frontend filters
  and clear the per-instance _new_*_data flag.
*/
bool AP_InertialSensor_CustomSerialIMU::update()
{
    if (!started) {
        return false;
    }

    _publish_temperature(accel_instance, accum.temp);

    update_accel(accel_instance);
    update_gyro(gyro_instance);

    return true;
}

/*
  Register gyro and accel with the frontend.

  Declared sample rate: 1000 Hz (matches IMU native output).
  Actual publish rate: equal to the main loop rate (~400 Hz on Copter
  with SCHED_LOOP_RATE=400). The frontend AP_InertialSensor::
  _update_sensor_rate() self-corrects low-pass/notch coefficients and
  GyroFFT bin counts within ~20-30 s. Until then, those coefficients
  are slightly off; for static bench testing this is harmless, but for
  flying add a 30 s idle at boot before takeoff.
*/
void AP_InertialSensor_CustomSerialIMU::start()
{
    if (started) {
        return;
    }

    if (!_imu.register_gyro(gyro_instance, 1000,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,
                                                         serial_port, 1,
                                                         DEVTYPE_SERIALIMU))) {
        return;
    }

    if (!_imu.register_accel(accel_instance, 1000,
                             AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,
                                                          serial_port, 2,
                                                          DEVTYPE_SERIALIMU))) {
        return;
    }

    // C1 option B: explicitly declare orientation. The det=-1 remap is
    // applied by hand in parse_frame() because Rotation cannot express
    // an improper transformation; if the IMU is remounted so the mapping
    // becomes det=+1, set a real Rotation here and drop the manual remap.
    set_gyro_orientation(gyro_instance, ROTATION_NONE);
    set_accel_orientation(accel_instance, ROTATION_NONE);

    started = true;
}

/*
  Probe function - detect the IMU on an AHRS-configured serial port.
  Looks for 3 consecutive valid frames within 3 seconds.
  Returns a new backend on success, nullptr on failure.

  The window was 60s in earlier revisions to work around the
  find_serial() instance bug; that bug is fixed so 3s is enough.
*/
AP_InertialSensor_Backend *AP_InertialSensor_CustomSerialIMU::probe(AP_InertialSensor &imu)
{
    AP_SerialManager &SM = AP::serialmanager();

    // Try SERIALx with SerialProtocol_AHRS (protocol 36)
    // find_serial returns the first matching port
    AP_HAL::UARTDriver *uart = SM.find_serial(
        AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        const char *m = "CustomSerialIMU: no SERIALx_PROTOCOL=36 (SerialProtocol_AHRS) configured, probe skipped";
        hal.console->printf("%s\n", m);
        strncpy(_probe_failure_msg, m, sizeof(_probe_failure_msg) - 1);
        _probe_failure_msg[sizeof(_probe_failure_msg) - 1] = 0;
        _probe_failure_pending = true;
        _probe_failure_send_count = 0;
        _probe_failure_last_send_ms = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", m);
        return nullptr;
    }

    // Configure to 921600 8N1
    uart->begin(921600, 0, 0);

    uint8_t rxbuf[FRAME_SIZE];
    uint16_t rxbuf_pos = 0;
    uint8_t valid_frames = 0;
    uint32_t deadline = AP_HAL::millis() + 3000;

    while (AP_HAL::millis() < deadline) {
        int16_t n = uart->available();
        while (n-- > 0) {
            uint8_t b;
            if (!uart->read(b)) break;

            if (rxbuf_pos == 0 && b != 0xAA) continue;
            rxbuf[rxbuf_pos++] = b;

            if (rxbuf_pos == FRAME_SIZE) {
                if (rxbuf[0] == 0xAA && rxbuf[1] == 0x55 &&
                    rxbuf[2] == 0xEB && rxbuf[3] == 0x90) {
                    // Header matches - do checksum check (SUM8)
                    uint8_t sum = 0;
                    for (uint8_t i = 0; i < FRAME_SIZE - 1; i++) {  // 0..36
                        sum += rxbuf[i];
                    }
                    if (sum == rxbuf[FRAME_SIZE - 1]) {  // rxbuf[37]
                        valid_frames++;
                        if (valid_frames >= 3) {
                            // Detected: 3 valid frames received
                            uart->discard_input();
                            // Capture the actual SERIAL port index so bus_id / HW ID is correct
                            const int8_t port_idx = SM.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
                            return NEW_NOTHROW AP_InertialSensor_CustomSerialIMU(imu, port_idx);
                        }
                    }
                }
                rxbuf_pos = 0;
            }
        }
        hal.scheduler->delay(1);
    }

    uart->discard_input();
    {
        char m[128];
        snprintf(m, sizeof(m), "CustomSerialIMU: only %u/3 valid frames in 3s on AHRS UART (check RS-422 wiring/baud/power)", (unsigned)valid_frames);
        hal.console->printf("%s\n", m);
        strncpy(_probe_failure_msg, m, sizeof(_probe_failure_msg) - 1);
        _probe_failure_msg[sizeof(_probe_failure_msg) - 1] = 0;
        _probe_failure_pending = true;
        _probe_failure_send_count = 0;
        _probe_failure_last_send_ms = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", m);
    }
    return nullptr;
}

/*
  Constructor - captures UART pointer from SerialManager.
  serial_port_id is a logical port number for bus_id only.
*/
AP_InertialSensor_CustomSerialIMU::AP_InertialSensor_CustomSerialIMU(
        AP_InertialSensor &imu, int8_t serial_port_id) :
    AP_InertialSensor_Backend(imu),
    serial_port(serial_port_id),
    started(false),
    rxbuf_pos(0),
    decimate_counter(0)
{
    accum.gyro.zero();
    accum.accel.zero();
    accum.temp = 25.0f;   // default 25℃
    accum.last_update_us = 0;

    // Retrieve the UART from SerialManager (first AHRS-configured port)
    AP_SerialManager &SM = AP::serialmanager();
    uart = SM.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
}

/*
  Startup banner for GCS console.
*/
bool AP_InertialSensor_CustomSerialIMU::get_output_banner(char *banner, uint8_t banner_len)
{
    snprintf(banner, banner_len,
             "IMU%u: CustomSerialIMU RS-422@921600 1000Hz (AA55EB90)",
             gyro_instance);
    return true;
}
