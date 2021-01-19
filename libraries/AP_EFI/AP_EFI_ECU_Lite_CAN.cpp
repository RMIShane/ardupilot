/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_Common/AP_Common.h>

#include <AP_HAL/utility/sparse-endian.h>

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>

#include "AP_EFI_ECU_Lite_CAN.h"

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

AP_EFI_ECU_Lite_CAN::AP_EFI_ECU_Lite_CAN(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    debug_can(2, "ECU Lite CAN: constructed\n");
}

constexpr struct AP_EFI_ECU_Lite_CAN::error_message AP_EFI_ECU_Lite_CAN::error_messages[];

void AP_EFI_ECU_Lite_CAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(2, "ECU Lite CAN: starting init\n");

    if (_initialized) {
        debug_can(1, "ECU Lite CAN: already initialized\n");
        return;
    }

    // get CAN manager instance
    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "ECU Lite CAN: no mgr for this driver\n");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "ECU Lite CAN: mgr not initialized\n");
        return;
    }

    // store pointer to CAN driver
    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "ECU Lite CAN: no CAN driver\n");
        return;
    }

    snprintf(_thread_name, sizeof(_thread_name), "ecu_lite_can_%u", driver_index);

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_EFI_ECU_Lite_CAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(1, "ECU Lite CAN: couldn't create thread\n");
        return;
    }

    _initialized = true;

    debug_can(2, "ECU Lite CAN: init done\n");

    return;
}

void AP_EFI_ECU_Lite_CAN::loop() {
    uavcan::MonotonicTime timeout;
    uavcan::CanFrame empty_frame { (0 | uavcan::CanFrame::FlagEFF), nullptr, 0 };
    const uavcan::CanFrame* select_frames[uavcan::MaxCanIfaces] { };
    select_frames[CAN_IFACE_INDEX] = &empty_frame;

    const uint32_t LOOP_INTERVAL_US = AP::scheduler().get_loop_period_us();
    while (true) {
        uavcan::CanSelectMasks inout_mask;
        uint64_t now = AP_HAL::micros64();

        // always look for received frames
        inout_mask.read = 1 << CAN_IFACE_INDEX;
        timeout = uavcan::MonotonicTime::fromUSec(now + LOOP_INTERVAL_US);

        // wait to receive frame
        uavcan::CanSelectMasks in_mask = inout_mask;
        _can_driver->select(inout_mask, select_frames, timeout);

        if (in_mask.read & inout_mask.read) {
            uavcan::CanFrame frame;
            uavcan::MonotonicTime time;
            uavcan::UtcTime utc_time;
            uavcan::CanIOFlags io_flags {};

            int16_t res = _can_driver->getIface(CAN_IFACE_INDEX)->receive(frame, time, utc_time, io_flags);

            if (res == 1) {
                const uint32_t id =  frame.id & uavcan::CanFrame::MaskExtID;

                switch ((ECU_Lite_CAN_ID)id) {
                    case ECU_Lite_CAN_ID::Engine_Time:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            ecu_engine_timing_t *data = (ecu_engine_timing_t *)frame.data;
                            internal_state.run_time = data->running_time;
                            internal_state.lifetime_run_time = data->engine_time;
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for Engine_Time");
                        }
                        break;
                    case ECU_Lite_CAN_ID::Engine_Details:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            ecu_engine_details_t *data = (ecu_engine_details_t *)frame.data;
                            internal_state.engine_speed_rpm = data->rpm;
                            internal_state.cylinder_status[0].cylinder_head_temperature = (((data->engine_temp_f * 1e-1f) - 32.0f) * (5.0f / 9.0f)) + C_TO_KELVIN;
                            internal_state.fuel_remaining_pct = data->fuel * (100.0f / 0xFFFF);
                            ecu_state.flags = data->flags;
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for Engine_Details");
                        }
                        break;
                    case ECU_Lite_CAN_ID::Charging_Details:
                        if (sem.take(1)) {
                            // FIXME: we haven't actually updated internal state, but this is still the most convient tool to flag everything though
                            internal_state.last_updated_ms = AP_HAL::millis();
                            ecu_charging_details_t *data = (ecu_charging_details_t *)frame.data;
                            ecu_state.voltage = data->voltage * 1e-3f;
                            ecu_state.amperage = data->amperage;
                            ecu_state.mah = data->mah;
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for Charging_Details");
                        }
                        break;
                    case ECU_Lite_CAN_ID::Logging_Data:
                        if (sem.take(1)) {
                            // FIXME: we haven't actually updated internal state, but this is still the most convient tool to flag everything though
                            internal_state.last_updated_ms = AP_HAL::millis();
                            ecu_internal_data_t *data = (ecu_internal_data_t *)frame.data;
                            ecu_state.pwm = data->pwm;
                            ecu_state.charge_trim = data->charge_trim;
                            ecu_state.esc_position = data->esc_position;
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for Engine_Time");
                        }
                        break;
                }
            }
        }

    }
}

void AP_EFI_ECU_Lite_CAN::update()
{
    // copy the data to the front end
    copy_to_frontend();

    const uint32_t now = AP_HAL::millis();
    if ((now - last_notification_time_ms) > notification_interval_ms) {
        last_notification_time_ms = now;

        // handle error messaging
        if (ecu_state.error_state !=  Error_State::None) {
            bool found_error = false;
            for (auto message : error_messages) {
                if (message.error == ecu_state.error_state) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "ECU: %s", message.message);
                    found_error = true;
                    break;
                }
            }
            if (!found_error) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "ECU: Unknown error state %d:", (int)ecu_state.error_state);
            }
        }
    }

    log();
}

bool AP_EFI_ECU_Lite_CAN::get_battery(float &voltage, float &current, float &mah) const {
    voltage = ecu_state.voltage;
    current = ecu_state.amperage;
    mah     = ecu_state.mah;
    return internal_state.last_updated_ms != 0;
}

void AP_EFI_ECU_Lite_CAN::log(void) {
    const struct Log_EFI_ECU_Lite pkt{
        LOG_PACKET_HEADER_INIT(LOG_EFI_ECU_LITE_MSG),
        time_us       : AP_HAL::micros64(),
        running_time  : internal_state.run_time,
        rpm           : (float)internal_state.engine_speed_rpm,
        voltage       : ecu_state.voltage,
        amperage      : ecu_state.amperage,
        mah           : (float)ecu_state.mah,
        fuel          : internal_state.fuel_remaining_pct,
        pwm           : ecu_state.pwm,
        charging      : ecu_state.flags,
        charge_trim   : ecu_state.charge_trim,
        esc_position  : ecu_state.esc_position,
        error_state   : (int16_t)ecu_state.error_state, // FIXME: this is 8 bits unsigned now, save the logging bandwidth
        engine_time   : internal_state.lifetime_run_time
    };    
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

AP_EFI_ECU_Lite_CAN *AP_EFI_ECU_Lite_CAN::get_singleton(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_ECU_Lite_CAN) {
        return nullptr;
    }
    return static_cast<AP_EFI_ECU_Lite_CAN*>(AP::can().get_driver(driver_index));
}

#endif // HAL_WITH_UAVCAN

