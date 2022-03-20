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
/*
  driver for Synthetic differential airspeed sensor
  https://www.allsensors.com/products/Synthetic-L01D
 */

#include "AP_Airspeed_Synthetic.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_EFI/AP_EFI.h>

extern const AP_HAL::HAL &hal;

#ifdef SYNTHETIC_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


AP_Airspeed_Synthetic::AP_Airspeed_Synthetic(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{}

// probe and initialise the sensor
bool AP_Airspeed_Synthetic::init()
{
    return true;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_Synthetic::get_differential_pressure(float &_pressure)
{
    const AP_EFI *efi = AP_EFI::get_singleton();
    if (efi == nullptr) {
        return false;
    }

    const AP_AHRS *ahrs = AP_AHRS::get_singleton();
    if (ahrs == nullptr) {
        return false;
    }

    _pressure = efi->get_rpm() * 2;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_Synthetic::get_temperature(float &_temperature)
{
    return false;
}
