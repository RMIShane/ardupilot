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
 
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"
#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

class AP_EFI_ECU_Lite_CAN : public AP_HAL::CANProtocol, public AP_EFI_Backend {
public:
    AP_EFI_ECU_Lite_CAN(AP_EFI &_frontend);
    
    /* Do not allow copies */
    AP_EFI_ECU_Lite_CAN(const AP_EFI_ECU_Lite_CAN &other) = delete;
    AP_EFI_ECU_Lite_CAN &operator=(const AP_EFI_ECU_Lite_CAN&) = delete;

    void init(uint8_t driver_index, bool enable_filters) override;

    void update() override;

    bool get_battery(float &voltage, float &current, float &mah) const override;

    static AP_EFI_ECU_Lite_CAN *get_singleton(uint8_t driver_index);
    
private:
    void loop();

    void log();

    bool _initialized;
    bool _emitted_version;
    char _thread_name[10];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;

    enum class ECU_Lite_CAN_ID {
        Engine_Time      = 0x1010, // run time, engine time
        Engine_Details   = 0x1020, // rpm, temp, fuel, errors, flags
        Charging_Details = 0x1030, // voltage, amperage, capacity
        Logging_Data     = 0x1040, // data to log internally (pwm, charge trim, esc position)
    };

    typedef struct PACKED {
        int32_t running_time; // in seconds
        int32_t engine_time; // in seconds
    } ecu_engine_timing_t;

    typedef struct PACKED {
        uint16_t rpm; // 0-65535 rpm
        int16_t engine_temp_f; // -3276.7 - 3276.7 F
        uint16_t fuel; // 0 - 100%
        uint8_t error_state;
        uint8_t flags;
    } ecu_engine_details_t;

    typedef struct PACKED {
        uint16_t voltage; // 0 - 65.35V
        float amperage;
        int16_t mah; // 0 - 65.535 Ah
    } ecu_charging_details_t;

    typedef struct PACKED {
        int16_t pwm;
        int16_t charge_trim;
        int16_t esc_position;
    } ecu_internal_data_t;

    enum class Error_State : uint8_t {
        None                = 0,
        Power_Bus_Anomaly   = 1,
        HCU_Error_Cleared   = 2,
        Engine_RPM_Anomaly  = 3,
        Starting_Failure    = 4,
        Engine_Health_Poor  = 5,
        Fuel_Sensor_Anomaly = 6,
    };

    struct error_message {
      Error_State error;
      const char * message;
    };

    const uint32_t notification_interval_ms = 5000;
    uint32_t last_notification_time_ms;

    static constexpr struct error_message error_messages [] = {
                                                                {Error_State::Power_Bus_Anomaly, "Power Bus Anomaly"},
                                                                {Error_State::HCU_Error_Cleared, "HCU_Error_Cleared"},
                                                                {Error_State::Engine_RPM_Anomaly, "Engine RPM Anomaly"},
                                                                {Error_State::Starting_Failure, "Starting Failure"},
                                                                {Error_State::Engine_Health_Poor, "Engine Health Poor"},
                                                                {Error_State::Fuel_Sensor_Anomaly, "Fuel Sensor Anomaly"},
                                                              };

    static const uint8_t CAN_IFACE_INDEX = 0; // FIXME: why do we need this, what does it mean?

    struct {
        float voltage;
        float amperage;
        uint16_t mah;
        int16_t pwm;
        int16_t charge_trim;
        int16_t esc_position;
        Error_State error_state;
        uint8_t flags;
        uint8_t old_flags;
    } ecu_state;

};

