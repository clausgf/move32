/**
 * Move32 - Naze32 for robotics
 * Control RC Servos via PWM
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include "board.h"
#include "servos.h"


// **************************************************************************

Servos servos;

void servosInit() {
    servos.init();
}


// **************************************************************************
