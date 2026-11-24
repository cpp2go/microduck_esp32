// SPDX-License-Identifier: MIT
#pragma once

#include "microduck_model.h"

/// Alias used in servo goal math (HOME + action_scale × scatter(action)).
#define MICRODUCK_HOME_POSE MICRODUCK_DEFAULT_POSITION

/// Soft position clip before writing Goal Position (rad).
static const float MICRODUCK_JOINT_LIMIT_RAD = 1.85f;