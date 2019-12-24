// Include configurations
#include "src/config/pin_names.h"

// Include actuator drivers
#include "src/drivers/hall.h"
#include "src/drivers/lsm9ds1.h"
#include "src/drivers/motor.h"

// Include submodules
#include "src/modules/submodules/ahrs.h"
#include "src/modules/submodules/we.h"
#include "src/modules/submodules/lqr.h"
#include "src/modules/submodules/fbl.h"
#include "src/modules/submodules/attitude_estimator.h"