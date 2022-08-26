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

    //snprintf(_thread_name, sizeof(_thread_name), "ecu_lite_can_%u", driver_index);
    strcpy(_thread_name, "ecu_lite_can");

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_EFI_ECU_Lite_CAN::loop, void), _thread_name, 4096, 
AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(1, "ECU Lite CAN: couldn't create thread\n");
        return;
    }

    _initialized = true;

    debug_can(2, "ECU Lite CAN: init done\n");

    return;
}

// MIKE ADDED THESE FUNCTIONS

// Read a value from a can packet and update the internal_state with it.
uint8_t AP_EFI_ECU_Lite_CAN::read_can_to_internal_state(void* internal_state_var, uint8_t *msg_data)
{

    // read the data type from the can message
    uint8_t data_type = msg_data[4];
    
    // the data is just copied into the frame data starting at [0]
    // figure out what type it is and copy it into the internal state.
    if((param_data_types_t)data_type == param_data_types_t::TYPE_INT16_T) // parse a int16
    {
        memcpy(internal_state_var, msg_data, 2); // 2 bytes
    }
    else if((param_data_types_t)data_type == param_data_types_t::TYPE_INT32_T) // parse a int32
    {
        memcpy(internal_state_var, msg_data, 4); // 4 bytes					
    }
    else if((param_data_types_t)data_type == param_data_types_t::TYPE_FLOAT32_T) // parse a float
    {
        memcpy(internal_state_var, msg_data, 4); // Float is 4 bytes (hopefully?)
    }
    else
    {
        debug_can(2, "Unknown data type!");
    }

	return 1;
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
				
				// MIKE'S NEW CODE HERE
                //gcs().send_text(MAV_SEVERITY_WARNING, "ANY CAN MESSAGE!!!");


				// make sure ID contains our magic number (0x22xx)
				if((id & 0xFFFFFF00) == 0x2200)
				{
          
                    // ECU Packet was detected
					uint8_t param_id = id & 0xFF;

                    //Debug Message Here
                    //gcs().send_text(MAV_SEVERITY_WARNING, "ECU CAN MESSAGE!!!");
					
					// switch off the param id then extract the data
					switch((ecu_parameters_t)param_id)

					{
						case ecu_parameters_t::ECU_PARAM_RT:
							read_can_to_internal_state(&ecu_state.running_time, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_RPM:
							read_can_to_internal_state(&ecu_state.rpm, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_V:
							read_can_to_internal_state(&ecu_state.voltage, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_A:
							read_can_to_internal_state(&ecu_state.amperage, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_MAH:
							read_can_to_internal_state(&ecu_state.mah, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_F:
							read_can_to_internal_state(&ecu_state.fuel, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_PWM:
							read_can_to_internal_state(&ecu_state.pwm, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_CH:
							read_can_to_internal_state(&ecu_state.charging, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_ESC:
							read_can_to_internal_state(&ecu_state.esc_position, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_CT:
							read_can_to_internal_state(&ecu_state.charge_trim, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_ES:
							read_can_to_internal_state(&ecu_state.error_state, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_ET:
							read_can_to_internal_state(&ecu_state.engine_time, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_ETHR:
							read_can_to_internal_state(&ecu_state.e_thrust, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_CSER:
							read_can_to_internal_state(&ecu_state.carb_servo, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_CHT:
							read_can_to_internal_state(&ecu_state.engine_temp, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_GEN:
							read_can_to_internal_state(&ecu_state.generator, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_CR:
							read_can_to_internal_state(&ecu_state.charge_rate, frame.data);
							break;
						case ecu_parameters_t::ECU_PARAM_EH:
							read_can_to_internal_state(&ecu_state.engine_health, frame.data);
							break;
					}
				}
            }
        }
    }
}

void AP_EFI_ECU_Lite_CAN::update()
{
	if (sem.take(1))
	{
        internal_state.last_updated_ms = AP_HAL::millis();
        internal_state.run_time = ecu_state.running_time;
        internal_state.engine_speed_rpm = ecu_state.rpm;
        internal_state.fuel_remaining_pct = ecu_state.fuel;
        internal_state.lifetime_run_time = ecu_state.engine_time;
        internal_state.spark_dwell_time_ms = ecu_state.e_thrust;
        internal_state.throttle_position_percent = ecu_state.carb_servo;
        internal_state.engine_load_percent = ecu_state.generator;
        internal_state.atmospheric_pressure_kpa = ecu_state.charge_rate;
        internal_state.intake_manifold_pressure_kpa = ecu_state.engine_health;

        // Make Synthetic airspeed avaliable to all drivers
        internal_state.synthetic_arspd = last_synthetic_arspd;

        // Cylinder Temp Conversion
        if (get_cyl_tmp_f() != 1) {
            internal_state.cylinder_status[0].cylinder_head_temperature = ecu_state.engine_temp + 273.0f;
        }
        else{
            internal_state.cylinder_status[0].cylinder_head_temperature = ((ecu_state.engine_temp - 32.0f) * 5.0f / 9.0f) + 273.0f;
        }

	    sem.give();
	}
	else
	{
		debug_can(2, "Failed to acquire the lock");
	}


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
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "HCU: %s", message.message);
                    found_error = true;
                    break;
                }
            }
            if (!found_error) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "HCU: Unknown error state %d:", (int)ecu_state.error_state);
            }
        }
    }

    // Engine Time (send once per engine cycle)
    if (internal_state.engine_speed_rpm < 1 && send_engine_time_message) {
        send_engine_time_message = false;

        // Engine Time 
        int16_t hours = internal_state.lifetime_run_time / 3600;
        int16_t tenths = (internal_state.lifetime_run_time % 3600) / 360;
        gcs().send_text(MAV_SEVERITY_INFO, "ENGINE TIME: %d.%d", hours, tenths);
    }
    // Reset Engine Message
    if (internal_state.engine_speed_rpm > 1000) {
        send_engine_time_message = true;
    }


    // Charge Messaging
    //float charge_current_seconds;
    if (ecu_state.charging == 1) {

        //Send charge start message (once)
        if (send_charge_message) {
            send_charge_message = false;
            gcs().send_text(MAV_SEVERITY_INFO, "CHARGE START");
        }

        //Charge Timer
        //charge_current_seconds = (now - charge_start_millis) / 1000;
        //last_charge_millis = now;
        
        //send_charge_complete_message = true;

        //Charge Calibration Messaging (optional)
        //if (plane.g2.supervolo_dev == 1){
        //    gcs().send_text(MAV_SEVERITY_INFO, "CT:%f PWM:%d V:%.1f A:%.1f ESC:%d Trim:%d", charge_current_seconds, _latest.pwm, _latest.voltage, _latest.amperage, _latest.esc_position, _latest.charge_trim);
            //}
    }
    else {
        if (now - last_charge_millis > 200) {
            //Send charge complete message (once)
            if (send_charge_complete_message) {
                send_charge_complete_message = false;
                gcs().send_text(MAV_SEVERITY_INFO, "CHARGE STOP");

                //charge_current_seconds = (now - charge_start_millis) / 1000;
                //int16_t minutes = floorf(charge_current_seconds / 60);
                //int16_t seconds = charge_current_seconds - (minutes * 60);
                //gcs().send_text(MAV_SEVERITY_INFO, "CHARGE TIME %d:%d", minutes, seconds);
            }

            // Reset Current Charge Timer 
            //charge_start_millis = now;
            send_charge_message = true;
        }
    }


    // SuperVolo
    // Very basic synthetic airspeed for transitions
    if (now - synthetic_arspd_ms > 1000){
        synthetic_arspd_ms = now;
        
        if (internal_state.engine_speed_rpm > 7500.0){
           last_synthetic_arspd = last_synthetic_arspd + 4;
        }

        else if (internal_state.engine_speed_rpm > 6500.0){
           last_synthetic_arspd = last_synthetic_arspd + 2;
        }

        else if (internal_state.engine_speed_rpm > 4500.0){
           last_synthetic_arspd = last_synthetic_arspd + 1;
        }

        else if (internal_state.engine_speed_rpm < 2500.0 ){  
            last_synthetic_arspd = last_synthetic_arspd - 2;
        }        
    
        if (last_synthetic_arspd > 30){
            last_synthetic_arspd = 30;
        }

        else if (last_synthetic_arspd < 0){
            last_synthetic_arspd = 0;
        }

        // dev message
        //float dev_message = last_synthetic_arspd;
        
        //if (last_synthetic_arspd > 0 && last_synthetic_arspd < 30){
        //    gcs().send_text(MAV_SEVERITY_INFO, "Synthetic ArSpd: %.1f", dev_message);     
        //}
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
        running_time  : ecu_state.running_time,
        rpm           : ecu_state.rpm,
        fuel          : ecu_state.fuel,
        charge_trim   : ecu_state.charge_trim,
        esc_position  : ecu_state.esc_position,
        error_state   : (int16_t)ecu_state.error_state, // FIXME: this is 8 bits unsigned now, save the logging bandwidth
        engine_time   : ecu_state.engine_time,
        e_thrust      : ecu_state.e_thrust,
        carb_servo    : ecu_state.carb_servo,
        engine_temp   : ecu_state.engine_temp,
        generator     : ecu_state.generator,
        charge_rate   : ecu_state.charge_rate,
        engine_health : ecu_state.engine_health
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

