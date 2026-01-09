#include "TercioBridge.h"
#include <string.h>

namespace tercio
{

    Bridge::Bridge() {}

    bool Bridge::begin(Stream &s)
    {
        _s = &s;
        _rx_head = _rx_tail = 0;
        for (auto &a : _axis)
        {
            a.id = 0xFFFF;
            a.valid = false;
        }
        for (auto &i : _imu)
        {
            i.id = 0xFFFF;
            i.valid = false;
        }
        return true;
    }

    void Bridge::end() { _s = nullptr; }

    size_t Bridge::rxAvailable() const
    {
        if (_rx_head >= _rx_tail)
            return _rx_head - _rx_tail;
        return RX_BUF_SIZE - (_rx_tail - _rx_head);
    }

    bool Bridge::rxPeek(size_t offset, uint8_t &out) const
    {
        if (offset >= rxAvailable())
            return false;
        size_t idx = (_rx_tail + offset) % RX_BUF_SIZE;
        out = _rx[idx];
        return true;
    }

    bool Bridge::rxRead(uint8_t &out)
    {
        if (rxAvailable() == 0)
            return false;
        out = _rx[_rx_tail];
        _rx_tail = (_rx_tail + 1) % RX_BUF_SIZE;
        return true;
    }

    bool Bridge::rxReadBytes(uint8_t *dst, size_t n)
    {
        if (rxAvailable() < n)
            return false;
        for (size_t i = 0; i < n; i++)
            rxRead(dst[i]);
        return true;
    }

    void Bridge::rxWriteBytes(const uint8_t *src, size_t n)
    {
        for (size_t i = 0; i < n; i++)
        {
            _rx[_rx_head] = src[i];
            _rx_head = (_rx_head + 1) % RX_BUF_SIZE;
            if (_rx_head == _rx_tail)
            {
                _rx_tail = (_rx_tail + 1) % RX_BUF_SIZE;
            }
        }
    }

    void Bridge::update()
    {
        if (!_s)
            return;

        // Pull available bytes into ring
        while (_s->available() > 0)
        {
            uint8_t b = (uint8_t)_s->read();
            rxWriteBytes(&b, 1);
        }

        // Parse frames: [can_id lo][can_id hi][cmd][len][payload...]
        while (true)
        {
            if (rxAvailable() < HDR_SIZE)
                return;

            uint8_t b0, b1, cmd, ln;
            if (!rxPeek(0, b0) || !rxPeek(1, b1) || !rxPeek(2, cmd) || !rxPeek(3, ln))
                return;

            uint16_t can_id = (uint16_t)b0 | ((uint16_t)b1 << 8);
            size_t total = HDR_SIZE + (size_t)ln;
            if (rxAvailable() < total)
                return;

            // consume header
            uint8_t hdr[HDR_SIZE];
            rxReadBytes(hdr, HDR_SIZE);

            uint8_t payload[MAX_PAYLOAD];
            if (ln > 0)
                rxReadBytes(payload, ln);

            handleFrame(can_id, cmd, payload, ln);
        }
    }

    void Bridge::pack_u16(uint8_t *dst, uint16_t v)
    {
        dst[0] = (uint8_t)(v & 0xFF);
        dst[1] = (uint8_t)((v >> 8) & 0xFF);
    }

    void Bridge::pack_f32(uint8_t *dst, float v)
    {
        // assumes IEEE754 float
        uint32_t u;
        memcpy(&u, &v, sizeof(u));
        dst[0] = (uint8_t)(u & 0xFF);
        dst[1] = (uint8_t)((u >> 8) & 0xFF);
        dst[2] = (uint8_t)((u >> 16) & 0xFF);
        dst[3] = (uint8_t)((u >> 24) & 0xFF);
    }

    bool Bridge::send(uint16_t can_id, uint8_t cmd, const uint8_t *payload, uint8_t len)
    {
        if (!_s)
            return false;
        can_id &= 0x7FF;
        if (len > MAX_PAYLOAD)
            return false;

        uint8_t hdr[HDR_SIZE];
        hdr[0] = (uint8_t)(can_id & 0xFF);
        hdr[1] = (uint8_t)((can_id >> 8) & 0xFF);
        hdr[2] = cmd;
        hdr[3] = len;

        _s->write(hdr, HDR_SIZE);
        if (len && payload)
            _s->write(payload, len);
        return true;
    }

    // ---------- high-level wrappers ----------
    bool Bridge::request_config(uint16_t can_id)
    {
        return send(can_id, (uint8_t)Cmd::GET_CONFIG, nullptr, 0);
    }

    bool Bridge::set_target_angle(uint16_t can_id, float angle)
    {
        uint8_t p[4];
        pack_f32(p, angle);
        return send(can_id, (uint8_t)Cmd::TARGET_ANGLE, p, 4);
    }

    bool Bridge::set_current_ma(uint16_t can_id, uint16_t ma)
    {
        uint8_t p[2];
        pack_u16(p, ma);
        return send(can_id, (uint8_t)Cmd::SET_CURRENT_MA, p, 2);
    }

    bool Bridge::set_speed_limit_rps(uint16_t can_id, float rps)
    {
        uint8_t p[4];
        pack_f32(p, rps);
        return send(can_id, (uint8_t)Cmd::SET_SPEED_LIMIT, p, 4);
    }

    bool Bridge::set_accel_limit_rps2(uint16_t can_id, float rps2)
    {
        uint8_t p[4];
        pack_f32(p, rps2);
        return send(can_id, (uint8_t)Cmd::SET_ACCEL_LIMIT, p, 4);
    }

    bool Bridge::set_pid(uint16_t can_id, float kp, float ki, float kd)
    {
        uint8_t p[12];
        pack_f32(p + 0, kp);
        pack_f32(p + 4, ki);
        pack_f32(p + 8, kd);
        return send(can_id, (uint8_t)Cmd::SET_PID, p, 12);
    }

    bool Bridge::set_can_id(uint16_t can_id, uint16_t new_id)
    {
        uint8_t p[2];
        pack_u16(p, (uint16_t)(new_id & 0x7FF));
        return send(can_id, (uint8_t)Cmd::SET_ID, p, 2);
    }

    bool Bridge::set_microsteps(uint16_t can_id, uint16_t microsteps)
    {
        uint8_t p[2];
        pack_u16(p, microsteps);
        return send(can_id, (uint8_t)Cmd::SET_MICROSTEPS, p, 2);
    }

    bool Bridge::set_stealthchop(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_STEALTHCHOP, p, 1);
    }

    bool Bridge::set_external_mode(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_EXT_MODE, p, 1);
    }

    bool Bridge::set_units_degrees(uint16_t can_id, bool use_degrees)
    {
        uint8_t p[1] = {(uint8_t)(use_degrees ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_UNITS, p, 1);
    }

    bool Bridge::set_encoder_invert(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_ENC_INVERT, p, 1);
    }

    bool Bridge::set_direction_invert(uint16_t can_id, bool invert)
    {
        uint8_t p[1] = {(uint8_t)(invert ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_DIR_INVERT, p, 1);
    }

    bool Bridge::enable_motor(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_ENABLED, p, 1);
    }

    bool Bridge::set_steps_per_rev(uint16_t can_id, uint16_t steps_per_rev)
    {
        uint8_t p[2];
        pack_u16(p, steps_per_rev);
        return send(can_id, (uint8_t)Cmd::SET_STEPS_PER_REV, p, 2);
    }

    bool Bridge::set_external_encoder(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_EXT_ENCODER, p, 1);
    }

    bool Bridge::set_external_spi(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_EXT_SPI, p, 1);
    }

    bool Bridge::set_endstop(uint16_t can_id, bool enable)
    {
        uint8_t p[1] = {(uint8_t)(enable ? 1 : 0)};
        return send(can_id, (uint8_t)Cmd::SET_ENDSTOP, p, 1);
    }

    bool Bridge::do_calibrate(uint16_t can_id)
    {
        return send(can_id, (uint8_t)Cmd::DO_CALIBRATE, nullptr, 0);
    }

    bool Bridge::do_homing(uint16_t can_id, const HomingParams &p)
    {
        HomingWire w{};

        w.use_in1 = (uint8_t)((p.useMinTrigger && !p.sensorlessHoming) ? 1 : 0);
        w.sensorless = (uint8_t)(p.sensorlessHoming ? 1 : 0);
        w.homingCurrent = p.homingCurrent;
        w.offset = p.offset;
        w.active_low = (uint8_t)((p.activeLow && !p.sensorlessHoming) ? 1 : 0);
        w.speed = p.speed;
        w.direction = (uint8_t)(p.direction ? 1 : 0);

        return send(can_id, (uint8_t)Cmd::DO_HOMING, (const uint8_t *)&w, (uint8_t)sizeof(w));
    }

    bool Bridge::do_auto_tune(uint16_t can_id, float min_angle, float max_angle)
    {
        uint8_t p[8];
        pack_f32(p + 0, min_angle);
        pack_f32(p + 4, max_angle);
        return send(can_id, (uint8_t)Cmd::DO_AUTO_TUNE, p, 8);
    }

    bool Bridge::set_imu_id(uint16_t current_id, uint16_t new_id)
    {
        uint8_t p[2];
        pack_u16(p, (uint16_t)(new_id & 0x7FF));
        return send(current_id, (uint8_t)Cmd::SET_IMU_ID, p, 2);
    }

    bool Bridge::reset_orientation(uint16_t imu_control_id)
    {
        return send(imu_control_id, (uint8_t)Cmd::RESET_ORIENT, nullptr, 0);
    }

    // ---------- slots ----------
    int Bridge::findAxisSlot(uint16_t can_id) const
    {
        for (size_t i = 0; i < MAX_AXIS; i++)
            if (_axis[i].id == can_id)
                return (int)i;
        return -1;
    }
    int Bridge::findImuSlot(uint16_t can_id) const
    {
        for (size_t i = 0; i < MAX_IMU; i++)
            if (_imu[i].id == can_id)
                return (int)i;
        return -1;
    }
    int Bridge::allocAxisSlot(uint16_t can_id)
    {
        int idx = findAxisSlot(can_id);
        if (idx >= 0)
            return idx;
        for (size_t i = 0; i < MAX_AXIS; i++)
        {
            if (_axis[i].id == 0xFFFF)
            {
                _axis[i].id = can_id;
                _axis[i].valid = false;
                return (int)i;
            }
        }
        return -1;
    }
    int Bridge::allocImuSlot(uint16_t can_id)
    {
        int idx = findImuSlot(can_id);
        if (idx >= 0)
            return idx;
        for (size_t i = 0; i < MAX_IMU; i++)
        {
            if (_imu[i].id == 0xFFFF)
            {
                _imu[i].id = can_id;
                _imu[i].valid = false;
                return (int)i;
            }
        }
        return -1;
    }
    int Bridge::registerAxis(uint16_t can_id) { return allocAxisSlot(can_id & 0x7FF); }
    int Bridge::registerImu(uint16_t can_id) { return allocImuSlot(can_id & 0x7FF); }

    const AxisState *Bridge::getAxis(uint16_t can_id) const
    {
        int idx = findAxisSlot(can_id & 0x7FF);
        if (idx < 0)
            return nullptr;
        return _axis[idx].valid ? &_axis[idx].st : nullptr;
    }
    const ImuState *Bridge::getImu(uint16_t can_id) const
    {
        int idx = findImuSlot(can_id & 0x7FF);
        if (idx < 0)
            return nullptr;
        return _imu[idx].valid ? &_imu[idx].st : nullptr;
    }

    // ---------- decoding ----------
    void Bridge::decodeAxisConfig(const AxisConfigWire &w, AxisConfig &out)
    {
        out.crc32 = w.crc32;
        out.microsteps = w.microsteps;
        out.stepsPerRev = w.stepsPerRev;
        out.units = w.units;

        out.flags.encInvert = (w.flags_u8 & 0x01) != 0;
        out.flags.dirInvert = (w.flags_u8 & 0x02) != 0;
        out.flags.stealthChop = (w.flags_u8 & 0x04) != 0;
        out.flags.externalMode = (w.flags_u8 & 0x08) != 0;
        out.flags.enableEndstop = (w.flags_u8 & 0x10) != 0;
        out.flags.externalEncoder = (w.flags_u8 & 0x20) != 0;
        out.flags.calibratedOnce = (w.flags_u8 & 0x40) != 0;

        out.encZeroCounts = w.encZeroCounts;
        out.driver_mA = w.driver_mA;
        out.maxRPS = w.maxRPS;
        out.maxRPS2 = w.maxRPS2;
        out.Kp = w.Kp;
        out.Ki = w.Ki;
        out.Kd = w.Kd;
        out.canArbId = w.canArbId & 0x7FF;
    }

    void Bridge::handleFrame(uint16_t can_id, uint8_t cmd, const uint8_t *payload, uint8_t len)
    {
        const uint32_t nowMs = millis();

        // 1) Stepper telemetry
        if (cmd == TELEMETRY_CMD)
        {
            if (len < (AXIS_CONFIG_SIZE + TELEM_TAIL_SIZE))
                return;

            AxisConfigWire cw;
            TelemTailWire tw;

            memcpy(&cw, payload, AXIS_CONFIG_SIZE);
            memcpy(&tw, payload + AXIS_CONFIG_SIZE, TELEM_TAIL_SIZE);

            AxisConfig cfg;
            decodeAxisConfig(cw, cfg);

            int slot = allocAxisSlot(cfg.canArbId);
            if (slot < 0)
                return;

            AxisState &st = _axis[slot].st;
            st.config = cfg;
            st.currentSpeed = tw.curSpd;
            st.currentAngle = tw.curAng;
            st.targetAngle = tw.tgtAng;
            st.temperature = tw.temp;
            st.stalled = (tw.stalled_u8 != 0);

            uint8_t ts = tw.tuneState_u8;
            st.tuneState = (ts <= (uint8_t)TuningState::DONE) ? (TuningState)ts : TuningState::IDLE;

            st.minTriggered = (tw.minT_u8 != 0);
            st.maxTriggered = (tw.maxT_u8 != 0);
            st.timestampMs = nowMs;
            _axis[slot].valid = true;
            return;
        }

        // 2) Stepper config response
        if (cmd == GET_CONFIG_CMD)
        {
            if (len < AXIS_CONFIG_SIZE)
                return;
            AxisConfigWire cw;
            memcpy(&cw, payload, AXIS_CONFIG_SIZE);

            AxisConfig cfg;
            decodeAxisConfig(cw, cfg);

            int slot = allocAxisSlot(cfg.canArbId);
            if (slot < 0)
                return;

            AxisState &st = _axis[slot].st;
            st.config = cfg;
            st.timestampMs = nowMs;
            _axis[slot].valid = true;
            return;
        }

        // 3) IMU telemetry
        if (cmd == IMU_TELEMETRY_CMD)
        {
            if (len < IMU_PAYLOAD_SIZE)
                return;
            ImuWire iw;
            memcpy(&iw, payload, IMU_PAYLOAD_SIZE);

            int slot = allocImuSlot(can_id & 0x7FF);
            if (slot < 0)
                return;

            ImuState &st = _imu[slot].st;
            st.roll = iw.roll;
            st.pitch = iw.pitch;
            st.yaw = iw.yaw;
            st.ax = iw.ax;
            st.ay = iw.ay;
            st.az = iw.az;
            st.temp = iw.temp;
            st.timestampMs = nowMs;
            _imu[slot].valid = true;
            return;
        }
    }
}