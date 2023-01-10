#include "mode.h"
#include "Plane.h"

bool ModeQLand::_enter()
{
    return plane.mode_qstabilize._enter();
    plane.emergency_qrtl_armed = false;
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}

