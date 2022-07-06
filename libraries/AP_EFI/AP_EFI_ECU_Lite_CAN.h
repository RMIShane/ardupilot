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
	
	uint8_t read_can_to_internal_state(void* internal_state_var, uint8_t *msg_data);
	

    void log();

    bool _initialized;
    bool _emitted_version;
    char _thread_name[10];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;
	
	
	// MIKE CAN PACKET FORMAT
	// These enums describe which parameter is in the message and what data type it is.
	enum class param_data_types_t
	{
		TYPE_UINT8_T = 0,
		TYPE_INT8_T,
		TYPE_UINT16_T,
		TYPE_INT16_T,
		TYPE_UINT32_T,
		TYPE_INT32_T,
		TYPE_FLOAT32_T
	};

	enum class ecu_parameters_t
	{
		ECU_PARAM_RT = 0,
		ECU_PARAM_RPM,
		ECU_PARAM_V,
		ECU_PARAM_A,
		ECU_PARAM_MAH,
		ECU_PARAM_F,
		ECU_PARAM_PWM,
		ECU_PARAM_CH,
		ECU_PARAM_ESC,
		ECU_PARAM_CT,
		ECU_PARAM_ES,
		ECU_PARAM_ET,
		// Extended params
		ECU_PARAM_ETHR,
		ECU_PARAM_CSER,
		ECU_PARAM_CHT,
		ECU_PARAM_GEN,
		ECU_PARAM_CR,
		ECU_PARAM_EH,
	};


    enum class Error_State : uint8_t {
        None                    = 0,
        engine_restart          = 1,
        engine_health_poor      = 2,
        engine_health_critical  = 3,
        starting_failure        = 4,
        power_bus_anomaly       = 5,
        engine_rpm_anomaly      = 6,
        e_thrust                = 7,
        charging_anomaly        = 8,
        fuel_sensor_anomaly     = 9,
        battery_critical        = 10,
        hcu_rally               = 98,
        hcu_error_cleared       = 99,
    };

    struct error_message {
      Error_State error;
      const char * message;
    };

    // Status Varables
    const uint32_t notification_interval_ms = 5000;
    uint32_t last_notification_time_ms;
    bool send_engine_time_message = true;
    bool send_charge_message = true;
    bool send_charge_complete_message;
    bool send_error_state_message = true;
    uint32_t charge_start_millis;
    uint32_t last_charge_millis;

    // SuperVolo timer for min RPM and very basic synthetic air speed
    uint32_t synthetic_arspd_ms;
    uint32_t synthetic_arspd_message_ms;
    int8_t last_synthetic_arspd;

    static constexpr struct error_message error_messages [] = {
                                                                {Error_State::engine_restart, "ENGINE RESTART"},
                                                                {Error_State::engine_health_poor, "ENGINE HEALTH POOR"},
                                                                {Error_State::engine_health_critical, "ENGINE HEALTH CRITICAL"},
                                                                {Error_State::starting_failure, "STARTING FAILURE"},
                                                                {Error_State::power_bus_anomaly, "POWER BUS ANOMALY"},
                                                                {Error_State::engine_rpm_anomaly, "ENGINE RPM ANOMALY"},
                                                                {Error_State::e_thrust, "E-THRUST"},
                                                                {Error_State::charging_anomaly, "CHARGING ANOMALY"},
                                                                {Error_State::fuel_sensor_anomaly, "FUEL SENSOR ANOMALY"},
                                                                {Error_State::battery_critical, "BATTERY CRITICAL"},
                                                                {Error_State::hcu_rally, "HCU-RALLY"},
                                                                {Error_State::hcu_error_cleared, "HCU ERROR CLEARED"},
                                                              };

    static const uint8_t CAN_IFACE_INDEX = 0; // FIXME: why do we need this, what does it mean?

    struct {
        int32_t running_time;
        float rpm;
        float voltage;
        float amperage;
        float mah;
        float fuel;
        int16_t pwm;
        int16_t charging;
        int16_t charge_trim;
        int16_t esc_position;
        Error_State error_state;
        int32_t engine_time;
        int16_t e_thrust;
        int16_t carb_servo;
        float engine_temp;
        int16_t generator;
        float charge_rate;
        float engine_health;

        uint8_t flags;
        uint8_t old_flags;
    } ecu_state;
};

