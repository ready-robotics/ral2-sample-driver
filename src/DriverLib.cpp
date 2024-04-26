/*
 * Copyright 2024 by READY Robotics Corporation.
 * All rights reserved. No person may copy, distribute, publicly display, create derivative works from or otherwise
 * use or modify this software without first obtaining a license from the READY Robotics Corporation.
 */
#include "SampleDriver.hpp"

#include <ral2/DriverLib.h>

extern "C" {
void *acquireDriver(void *config, void *parent)
{
    return ACQUIRE_DRIVER(ready::SampleDriver, config, parent);
}
}
