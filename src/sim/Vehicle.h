// Vehicle.h — one simulated vehicle: an FMU instance, its shared reference
// trajectory, a start offset onto the track, its task set, and latest outputs.
#pragma once

#include <memory>

#include "fmu/Fmu.h"
#include "fmu/FmuTypes.h"
#include "sched/TaskModel.h"
#include "trace/Trajectory.h"

namespace cps {

struct Vehicle {
    std::unique_ptr<FmuInstance> fmu;
    std::shared_ptr<Trajectory>  traj;
    long           startOffset = 0;   // ticks into the trajectory at sim t=0
    TaskSet        taskSet;
    VehicleOutputs out;               // latest FMU outputs
    double         curVel = 0.0;      // velocity input applied this tick
};

}  // namespace cps
