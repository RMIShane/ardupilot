#include "mode.h"
#include "Plane.h"

bool ModeQLoiter::_enter()
{
    return plane.mode_qstabilize._enter();
    plane.emergency_qrtl_armed = false;
}

void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}


