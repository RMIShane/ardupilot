// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


    #include "Plane.h"


    void Plane::update_ECU_Lite()
    {
    // This code assumes that the ECU is sending us lines of text.  We buffer
    // up the data until we see the newline character come from the ECU, which
    // lets us know that we've received a whole line that we can send to the log.

    // If the ECU sends a line longer than this, we'll treat it as if it were two
    // lines.
    const int MAX_LINE = 80;

    // Making this variable "static" means that it doesn't get wiped out when the
    // function returns.  So it's like having a global variable in that it remembers
    // its value, but without being visible to everything outside this function.
    static char line_buffer[MAX_LINE] = {0};
    
    static int line_index = 0; // Where we currently are in the line.
    static int ecu_lite_message_interval = 0;
    static int ecu_lite_charge_message = 0; //Only send one message at begining of each charge cycle.
    static int ecu_lite_charge_start_millis = 0;
    static int ecu_lite_charge_current_seconds = 0;
        
    //SSCANF Testing
    //char line_test[80];
    //char test[]="RT:35 RPM:6542 V:49.8 A:-5.2 B:98 PWM:1223 CH:1 ESC:1290 CT:1 ER:0 CM:1";
    //sscanf(test, "RT:%f RPM:%f V:%f A:%f B:%d CH:%d ER:%d",
    //&ecu_lite_running_time, &ecu_lite_rpm, &ecu_lite_voltage, &ecu_lite_amperage, &ecu_lite_battery, &ecu_lite_charging, &ecu_lite_esc_resets);
    
    int n = hal.uartE->available();
    for (int i = 0; i < n; i++) {
        char received = hal.uartE->read();

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
    static int ecu_lite_esc_resets = 0;
    static int ecu_lite_calibration_mode = 0;
    
    sscanf(line_buffer, "RT:%f RPM:%f V:%f A:%f B:%d PWM:%d CH:%d ESC:%d CT:%d ER:%d CM:%d",
    &ecu_lite_running_time, &ecu_lite_rpm, &ecu_lite_voltage, &ecu_lite_amperage, &ecu_lite_battery, &ecu_lite_pwm, &ecu_lite_charging, &ecu_lite_esc_position, &ecu_lite_charge_trim, 
    &ecu_lite_esc_resets, &ecu_lite_calibration_mode);
    
    //**********Messaging**********
    if ((millis() - ecu_lite_message_interval) > 500){
        ecu_lite_message_interval = millis();
        
        char log_message[MAX_LINE + 100];
     
        // Forward serial into MAVLINK message. (for testing)
        //gcs().send_text(MAV_SEVERITY_INFO, line_buffer);
     
        //If charging
        if (ecu_lite_charging == 1){    
             
            //Send charge start message (once)
            if (ecu_lite_charge_message == 0){
                 sprintf(log_message, "Charge Start");
                 gcs().send_text(MAV_SEVERITY_INFO, log_message);
                 ecu_lite_charge_message = 1;
            }
               
            //Charge Timer
            ecu_lite_charge_current_seconds = ((millis() - ecu_lite_charge_start_millis) / 1000);
            
            //Charge Calibration Messaging (optional)
            if (ecu_lite_calibration_mode == 1){      
                sprintf(log_message, "CT:%d PWM:%d V:%.1f A:%.1f ESC:%d Trim:%d R:%d", ecu_lite_charge_current_seconds, ecu_lite_pwm, ecu_lite_voltage, ecu_lite_amperage, ecu_lite_esc_position, ecu_lite_charge_trim, ecu_lite_esc_resets); 
                gcs().send_text(MAV_SEVERITY_INFO, log_message);
            }
        }
     
        //Send charge complete message (once)      
        else{
            if (ecu_lite_charge_message == 1){
                 gcs().send_text(MAV_SEVERITY_INFO, "Charge Stop %d", ecu_lite_charge_current_seconds);
                 ecu_lite_charge_message = 0;
            }
         
            //Reset Current Charge Timer 
            ecu_lite_charge_start_millis = millis();
            ecu_lite_charge_current_seconds = 0;    
        }
      } 
    }
