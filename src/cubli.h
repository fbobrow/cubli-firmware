// Include utilities
#include "src/utils/parameters.h"
#include "src/utils/pin_names.h"

// Include drivers
#include "src/drivers/hall.h"
#include "src/drivers/lsm9ds1.h"
#include "src/drivers/motor.h"

// Include modules
#include "src/modules/wheel_estimator.h"
#include "src/modules/attitude_estimator.h"
#include "src/modules/attitude_wheel_controller_2d.h"
#include "src/modules/attitude_wheel_controller_3d.h"