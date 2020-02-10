#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Param/AP_Param.h>
#include <AP_RPM/AP_RPM.h>

class AP_ECU_Telem {

public:
    AP_ECU_Telem();

    //Initialize Port 
    void init();
    
    //Read ECU Telem
    void update();
    
    AP_HAL::UARTDriver *ecu_port;
    
private:
    float ecu_lite_running_time;
    float ecu_lite_rpm = 0;
    float ecu_lite_voltage = 0;
    float ecu_lite_amperage = 0;
    int ecu_lite_fuel = 0;
    int ecu_lite_throttle_min = 0;  
 };
