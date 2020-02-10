#include "AP_ECU_Telem.h"
#include "Plane.h"
    
//ECU Port Init   
void AP_ECU_Telem::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    ecu_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ECU_Telem, 0);
    
    if (ecu_port != nullptr) {
        ecu_port->begin(57600);
    }
}       
      

    void AP_ECU_Telem::update()
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
    static char line_buffer[MAX_LINE] = {0};
    
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

        line_buffer[line_index] = received;

        // If we get a newline, we know we're at the end of the line but we
        // probably don't want to put the newline in the log because the log
        // will probably have a newline of its own, so you'd get an empty line
        // after log messages.  So by not incrementing line_index, we'll end
        // up trimming off the newline.
        if (received != '\n') {
           line_index++;
        }

        if (line_index >= MAX_LINE - 1 || received == '\n') {
            line_buffer[line_index] = 0; // Null-terminate the line to make a proper C string 
            line_index = 0;         
        }  
    }
            
    //SSCANF parse incoming serial data
    static int ecu_lite_pwm = 0;
    static int ecu_lite_charging = 0;
    static int ecu_lite_charge_trim = 0;
    static int ecu_lite_esc_position = 0;
    static int ecu_lite_overvoltage = 0;
    static int32_t ecu_lite_hobbs = 0;
    static int ecu_lite_hobbs_message = 0;
    
    //sscanf(line_buffer, "RT:%f RPM:%f V:%f A:%f F:%d PWM:%d CH:%d ESC:%d CT:%d OV:%d H:%d",
    //&ecu_lite_running_time, &ecu_lite_rpm, &ecu_lite_voltage, &ecu_lite_amperage, &ecu_lite_fuel, &ecu_lite_pwm, &ecu_lite_charging, &ecu_lite_esc_position, &ecu_lite_charge_trim, 
    //&ecu_lite_overvoltage, &ecu_lite_hobbs);

    
    //Fuel Level Clamping
    if (ecu_lite_fuel < 0){
     ecu_lite_fuel = 0;
     }
     
    else if (ecu_lite_fuel > 100){
    ecu_lite_fuel = 100;
    }


    //Overvoltage Throttle Hold
    if (ecu_lite_overvoltage == 1){
    //ecu_lite_throttle_min = plane.g2.supervolo_ov_thr;
    }
    
    
    //**********Messaging**********
    if ((millis() - ecu_lite_message_interval) > 1000){
        ecu_lite_message_interval = millis();
        
        char log_message[MAX_LINE + 100];
     
        
        if (ecu_lite_overvoltage == 1){
              sprintf(log_message, "BATTERIES DISCONNECTED");
              //gcs().send_text(MAV_SEVERITY_INFO, log_message);
        }
     
        
        //Engine Time (send once per engine cycle)
        if ((ecu_lite_rpm < 1) && (ecu_lite_hobbs_message == 1)){
        
            //Engine Time      
            int hours = ecu_lite_hobbs / 3600;
            int tenths = (ecu_lite_hobbs % 3600) / 360;      
            sprintf(log_message, "Engine Time: %d.%d", hours, tenths);
            //gcs().send_text(MAV_SEVERITY_INFO, log_message);
            ecu_lite_hobbs_message = 0;
        
            //Airframe Time      
            hours = ecu_lite_hobbs / 3600;
            tenths = (ecu_lite_hobbs % 3600) / 360;      
            sprintf(log_message, "Airframe Time: %d.%d", hours, tenths);
            //gcs().send_text(MAV_SEVERITY_INFO, log_message);
            ecu_lite_hobbs_message = 0;
        }
        
        //Reset Time Message      
        if (ecu_lite_rpm > 3000){
            ecu_lite_hobbs_message = 1;
        }
        
        
        //If charging
        if (ecu_lite_charging == 1){    
             
            //Send charge start message (once)
            if (ecu_lite_charge_message == 0){
                 sprintf(log_message, "Charge Start");
                 //gcs().send_text(MAV_SEVERITY_INFO, log_message);
                 ecu_lite_charge_message = 1;
            }
               
            //Charge Timer
            ecu_lite_charge_current_seconds = ((millis() - ecu_lite_charge_start_millis) / 1000);
            
            //Charge Calibration Messaging (optional)
            if (plane.g2.supervolo_dev == 1){      
                sprintf(log_message, "CT:%d PWM:%d V:%.1f A:%.1f ESC:%d Trim:%d", ecu_lite_charge_current_seconds, ecu_lite_pwm, (double)ecu_lite_voltage, (double)ecu_lite_amperage,      ecu_lite_esc_position, ecu_lite_charge_trim); 
                //gcs().send_text(MAV_SEVERITY_INFO, log_message);
            }
            
        }
     
        //Send charge complete message (once)
        else{
            if (ecu_lite_charge_message == 1){
                //gcs().send_text(MAV_SEVERITY_INFO, "Charge Stop");
                int minutes = ecu_lite_charge_current_seconds / 60;
                int seconds = ecu_lite_charge_current_seconds % 60;
                sprintf(log_message, "Charging Time %d.%d", minutes, seconds);
                //gcs().send_text(MAV_SEVERITY_INFO, log_message);
                ecu_lite_charge_message = 0;
            }
         
            //Reset Current Charge Timer 
            ecu_lite_charge_start_millis = millis();
            ecu_lite_charge_current_seconds = 0;    
        }
      } 
    }
