// Include configurations
#include "src/config/pin_names.h"

// Include actuator drivers
#include "src/drivers/hall.h"
#include "src/drivers/lsm9ds1.h"
#include "src/drivers/motor.h"

// Include submodules
#include "src/modules/wheel_estimator.h"
#include "src/modules/attitude_estimator_2d.h"
#include "src/modules/attitude_estimator_3d.h"
#include "src/modules/attitude_estimator_madgwick.h"
#include "src/modules/attitude_wheel_controller_2d.h"