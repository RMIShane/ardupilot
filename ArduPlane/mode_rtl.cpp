#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude());
    plane.current_RTL_altitude = plane.get_RTL_altitude();
    plane.last_low_altitude = plane.current_loc.alt;
    plane.low_altitude_count = 0;

    return true;
}

void ModeRTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
    
    
    // Altitude Monitor     
    if (AP_HAL::millis() - plane.last_altitude_check_ms > 1000){            
        plane.last_altitude_check_ms = AP_HAL::millis();
        
        // Are we below our RTL Altitude?
        if (plane.current_loc.alt < plane.current_RTL_altitude - 200) {
            
            //dev messaging
            float current_altitude = plane.current_loc.alt;
            float current_distance = plane.current_loc.get_distance(plane.next_WP_loc);
            float low_alt_cnt = plane.low_altitude_count;
            float rally_alt = plane.current_RTL_altitude;
            gcs().send_text(MAV_SEVERITY_INFO, "ALT: %.2f Dist: %.2f LAC: %.2f RALT: %.2f" ,current_altitude, current_distance, low_alt_cnt, rally_alt);
                       
            // Are we decending?
            if (plane.current_loc.alt < plane.last_low_altitude - 500) {
                plane.last_low_altitude = plane.current_loc.alt;
                plane.low_altitude_count ++;
                     
                // Have we continued to decend for a total of 25 meters / over at least 5 seconds?
                if (plane.low_altitude_count > 6){
                
                    // How far are we from our rally location or home?
                    if (plane.current_loc.get_distance(plane.next_WP_loc) > 500.0){   
                        plane.set_mode(plane.mode_qland, ModeReason::UNKNOWN);
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "LOW ALTITUDE - QLAND");
                    }
                    else {
                        plane.set_mode(plane.mode_qrtl, ModeReason::UNKNOWN);
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "LOW ALTITUDE - QRTL");
                    }
                }
            }
        }
        
        else {      
            plane.low_altitude_count = 0;
        }
    }
}
