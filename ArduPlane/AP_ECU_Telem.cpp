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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "AP_ECU_Telem.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ECU_Telem::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable button reporting
    // @Description: This enables ECU Telemetry Input checking module.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_ECU_Telem, enable, 0, AP_PARAM_FLAG_ENABLE),
    
    // @Param: 
    // @DisplayName: First button Pin
    // @Description: Digital pin number for first button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("ARSPD_FUEL_COMP", 1, AP_ECU_Telem, airspeed_fuel_comp, 0),
    
    // @Param: SUPERVOLO_BD_THR
    // @DisplayName: Battery Disconnect Throttle Hold
    // @Description: Sets Throttle position to hold for quad modes in the event batery connection is lost.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SUPERVOLO_BD_THR", 2, AP_ECU_Telem, supervolo_bd_thr, 20),
    
    // @Param: SUPERVOLO_DEV
    // @DisplayName: supervolo_dev
    // @Description: Turns on dev mavlink messaging
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("SUPERVOLO_DEV", 3, AP_ECU_Telem, supervolo_dev, 1),

    AP_GROUPEND   
};


// constructor
AP_ECU_Telem::AP_ECU_Telem(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


//ECU Port Init   
void AP_ECU_Telem::init(void)
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    ecu_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ECU_Telem, 0);
    
    if (ecu_port != nullptr) {
        ecu_port->begin(57600);
    }
}       
      

void AP_ECU_Telem::update(void)
    {
    // This code assumes that the ECU is sending us lines of text.  We buffer
    // up the data until we see the newline character come from the ECU, which
    // lets us know that we've received a whole line that we can send to the log.

    // If the ECU sends a line longer than this, we'll treat it as if it were two
    // lines.
    const int MAX_LINE = 100;

    // Making this variable "static" means that it doesn't get wiped out when the
    // function returns.  So it's like having a global variable in that it remembers
    // its value, but without being visible to everything outside this function.
    //static char line_buffer[MAX_LINE] = {0};
    
    static int line_index = 0; // Where we currently are in the line.
    static int ecu_lite_message_interval = 0;
    static int ecu_lite_charge_message = 0; //Only send one message at begining of each charge cycle.
    static int ecu_lite_charge_start_millis = 0;
    static int ecu_lite_charge_current_seconds = 0;
         
    
    // don't run if we don't have a port
    if (ecu_port == nullptr) {
        return;
    }
    
    int n = ecu_port->available();
    for (int i = 0; i < n; i++) {
        char received = ecu_port->read();

        //line_buffer[line_index] = received;

        // If we get a newline, we know we're at the end of the line but we
        // probably don't want to put the newline in the log because the log
        // will probably have a newline of its own, so you'd get an empty line
        // after log messages.  So by not incrementing line_index, we'll end
        // up trimming off the newline.
        
        if (received != '\n') {
           line_index++;
        }

        //if (line_index >= MAX_LINE - 1 || received == '\n') {
        //    line_buffer[line_index] = 0; // Null-terminate the line to make a proper C string 
        //   line_index = 0;         
        //}  
    }
            
    //SSCANF parse incoming serial data
    static int ecu_lite_pwm = 0;
    static int ecu_lite_charging = 0;
    static int ecu_lite_charge_trim = 0;
    static int ecu_lite_esc_position = 0;
    static int ecu_lite_batt_disconnect = 0;
    static int32_t ecu_lite_hobbs = 0;
    static int ecu_lite_hobbs_message = 0;
    
    //sscanf(line_buffer, "RT:%f RPM:%f V:%f A:%f F:%d PWM:%d CH:%d ESC:%d CT:%d OV:%d H:%d",
    //&ecu_lite_running_time, &ecu_lite_rpm, &ecu_lite_voltage, &ecu_lite_amperage, &ecu_lite_fuel, &ecu_lite_pwm, &ecu_lite_charging, &ecu_lite_esc_position, &ecu_lite_charge_trim, 
    //&ecu_lite_batt_disconnect, &ecu_lite_hobbs);

    
    //Fuel Level Clamping
    if (ecu_lite_fuel < 0){
     ecu_lite_fuel = 0;
     }
     
    else if (ecu_lite_fuel > 100){
    ecu_lite_fuel = 100;
    }


    //Battery Disconnect Throttle Hold
    if (ecu_lite_batt_disconnect == 1){
    ecu_lite_throttle_min = supervolo_bd_thr;
    }
    
    
    //**********Messaging**********
    if ((AP_HAL::millis() - ecu_lite_message_interval) > 1000){
        ecu_lite_message_interval = AP_HAL::millis();
        
        char log_message[MAX_LINE + 100];
     
        
        if (ecu_lite_batt_disconnect == 1){
              snprintf(log_message, 100, "BATTERIES DISCONNECTED");
              //gcs().send_text(MAV_SEVERITY_INFO, log_message);
        }
     
        
        //Engine Time (send once per engine cycle)
        if ((ecu_lite_rpm < 1) && (ecu_lite_hobbs_message == 1)){
        
            //Engine Time      
            int hours = ecu_lite_hobbs / 3600;
            int tenths = (ecu_lite_hobbs % 3600) / 360;      
            snprintf(log_message, 100,  "Engine Time: %d.%d", hours, tenths);
            //gcs().send_text(MAV_SEVERITY_INFO, log_message);
            ecu_lite_hobbs_message = 0;
        
            //Airframe Time      
            //hours = ecu_lite_hobbs / 3600;
            //tenths = (ecu_lite_hobbs % 3600) / 360;      
            //snprintf(log_message, "Airframe Time: %d.%d", hours, tenths);
            //gcs().send_text(MAV_SEVERITY_INFO, log_message);
            //ecu_lite_hobbs_message = 0;
        }
        
        //Reset Time Message      
        if (ecu_lite_rpm > 3000){
            ecu_lite_hobbs_message = 1;
        }
        
        
        //If charging
        if (ecu_lite_charging == 1){    
             
            //Send charge start message (once)
            if (ecu_lite_charge_message == 0){
                 snprintf(log_message, 100, "Charge Start");
                 //gcs().send_text(MAV_SEVERITY_INFO, log_message);
                 ecu_lite_charge_message = 1;
            }
               
            //Charge Timer
           ecu_lite_charge_current_seconds = ((AP_HAL::millis() - ecu_lite_charge_start_millis) / 1000);
            
            //Charge Calibration Messaging (optional)
            //if (plane.g2.supervolo_dev == 1){      
                snprintf(log_message, 100, "CT:%d PWM:%d V:%.1f A:%.1f ESC:%d Trim:%d", ecu_lite_charge_current_seconds, ecu_lite_pwm, (double)ecu_lite_voltage, (double)ecu_lite_amperage,      ecu_lite_esc_position, ecu_lite_charge_trim); 
                //gcs().send_text(MAV_SEVERITY_INFO, log_message);
            //}
            
        }
     
        //Send charge complete message (once)
        else{
            if (ecu_lite_charge_message == 1){
                //gcs().send_text(MAV_SEVERITY_INFO, "Charge Stop");
                int minutes = ecu_lite_charge_current_seconds / 60;
                int seconds = ecu_lite_charge_current_seconds % 60;
                snprintf(log_message, 100, "Charging Time %d.%d", minutes, seconds);
                //gcs().send_text(MAV_SEVERITY_INFO, log_message);
                ecu_lite_charge_message = 0;
            }
         
            //Reset Current Charge Timer 
            ecu_lite_charge_start_millis = AP_HAL::millis();
            ecu_lite_charge_current_seconds = 0;    
        }
      } 
    }
