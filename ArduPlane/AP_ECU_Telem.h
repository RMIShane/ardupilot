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

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_ECU_Telem {
public:
    // constructor
    AP_ECU_Telem(void);

    static const struct AP_Param::GroupInfo var_info[];

    // Initialize ECU
    void init(void);
    
    // Update ECU Telem
    void update(void);
    
    //Parameters
    AP_Int16 airspeed_fuel_comp;
    AP_Int16 supervolo_bd_thr;
    AP_Int16 supervolo_dev;
             
     
    
private:
    AP_Int8 enable;
    
    float ecu_lite_running_time;
    float ecu_lite_rpm = 0;
    float ecu_lite_voltage = 0;
    float ecu_lite_amperage = 0;
    int ecu_lite_fuel = 0;
    int ecu_lite_throttle_min = 0; 
    
    AP_HAL::UARTDriver *ecu_port;
};
