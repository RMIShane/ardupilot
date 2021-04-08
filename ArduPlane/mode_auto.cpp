#include "mode.h"
#include "Plane.h"

bool ModeAuto::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }
    
    // Altitude Monitor
    plane.current_NAV_altitude = plane.next_WP_loc.alt;
    plane.last_low_altitude = plane.current_loc.alt;
    plane.low_altitude_count = 0;
    plane.low_airspeed_count = 0;
    
    //dev messaging
    //float current_altitude = plane.current_loc.alt - plane.home.alt;
    //float NAV_alt = plane.current_NAV_altitude - plane.home.alt;
    //gcs().send_text(MAV_SEVERITY_INFO, "Alt: %.2f NavAlt: %.2f" ,current_altitude, NAV_alt);

#if SOARING_ENABLED == ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND &&
            !plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))
        {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
    } else if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        } else {
            plane.calc_throttle();
        }
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
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
            if (plane.current_loc.alt < plane.last_low_altitude - 300 && !plane.quadplane.in_vtol_land_sequence()) {
                plane.last_low_altitude = plane.current_loc.alt;
                plane.low_altitude_count ++;      
                     
                // Have we continued to decend for a total of 18 meters / over at least 6 seconds?
                if (plane.low_altitude_count > 5) {
     
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

