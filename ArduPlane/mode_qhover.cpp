#include "mode.h"
#include "Plane.h"

bool ModeQHover::_enter()
{
    return plane.mode_qstabilize._enter();
    plane.emergency_qrtl_armed = false;
}

void ModeQHover::update()
{
    plane.mode_qstabilize.update();
}


