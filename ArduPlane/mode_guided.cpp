#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    plane.guided_WP_loc = plane.current_loc;
    plane.set_guided_WP();
    
    // Altitude Monitor
    plane.current_NAV_altitude = plane.next_WP_loc.alt;
    plane.last_low_altitude = plane.current_loc.alt;
    plane.low_altitude_count = 0;
    plane.low_airspeed_count = 0;
    
    //dev messaging
    //float current_altitude = plane.current_loc.alt - plane.home.alt;
    //float NAV_alt = plane.current_NAV_altitude - plane.home.alt;
    //gcs().send_text(MAV_SEVERITY_INFO, "Alt: %.2f NavAlt: %.2f" ,current_altitude, NAV_alt);

    return true;
}

void ModeGuided::update()
{
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
    } else {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
    
    
    // Altitude Monitoring    
    if (AP_HAL::millis() - plane.last_altitude_check_ms > 1000){            
        plane.last_altitude_check_ms = AP_HAL::millis();
        
        // Update current NAV altitude
        plane.current_NAV_altitude = plane.next_WP_loc.alt;     
        
        // Are we below our NAV Altitude?
        if (plane.current_loc.alt < plane.current_NAV_altitude - 200){
            
            //dev messaging
            //float current_altitude = plane.current_loc.alt - plane.home.alt;
            //float current_distance = plane.current_loc.get_distance(plane.next_WP_loc);
            //float low_alt_cnt = plane.low_altitude_count;
            //float NAV_alt = plane.current_NAV_altitude - plane.home.alt;
            //gcs().send_text(MAV_SEVERITY_INFO, "Alt: %.2f Dist: %.2f LAC: %.2f NavAlt: %.2f" ,current_altitude, current_distance, low_alt_cnt, NAV_alt);
                       
            // Are we decending?
            if (plane.current_loc.alt < plane.last_low_altitude - 300){
                plane.last_low_altitude = plane.current_loc.alt;
                plane.low_altitude_count ++;      
                     
                // Have we continued to decend for a total of 18 meters / over at least 6 seconds?
                if (plane.low_altitude_count > 5 && !plane.quadplane.in_vtol_land_sequence()) {
     
                    // Set mode to QRTL then fly torwards our rally or home location until we either get there or reach our critical battery failsafe.
                    plane.set_mode(plane.mode_rtl, ModeReason::UNKNOWN);
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "LOW ALTITUDE - RTL");               
                }
            }
        }
        
        else {     
            plane.low_altitude_count = 0;
            plane.last_low_altitude = plane.current_loc.alt;
        }
    }
}

