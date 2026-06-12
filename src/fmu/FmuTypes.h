// FmuTypes.h — Plain-data structs exchanged with the FMU each tick.
// Kept in the fmu layer so both Fmu (producer/consumer) and the scheduler
// framework (which emits triggers and reads a VehicleView) can share them
// without a circular dependency.
#pragma once

#include <array>
#include <cstdint>

namespace cps {

// The 16 one-shot boolean triggers that drive the FMU task chain for one
// vehicle in one base step. Field order matches cps::vr::Trigger (VR 100..115).
//
// Semantics (see process_trigger_events in LateralMotionControl.c):
//   *_activated : task samples its inputs and computes into an internal buffer.
//   *_finished  : the previously-computed buffer is published to the output.
//   net_*_sent  : push current upstream data into the network FIFO.
//   net_*_recv  : pop the delayed data from the network FIFO.
// A job therefore pulses `activated` at its start tick and `finished` at
// start+execution_time; a network hop pulses `sent` then `received` after the
// delay. Each flag is true only on its event tick.
struct VehicleTriggers {
    bool sensor_act   = false;
    bool sensor_fin   = false;
    bool net_sc_sent  = false;
    bool net_sc_recv  = false;
    bool est_act      = false;
    bool est_fin      = false;
    bool ctrl_act     = false;
    bool ctrl_fin     = false;
    bool ff_act       = false;
    bool ff_fin       = false;
    bool merge_act    = false;
    bool merge_fin    = false;
    bool net_ca_sent  = false;
    bool net_ca_recv  = false;
    bool act_act      = false;
    bool act_fin      = false;

    // Indexed access in VR order (0 -> sensor_act ... 15 -> act_fin).
    bool&       operator[](int i)       { return (&sensor_act)[i]; }
    const bool& operator[](int i) const { return (&sensor_act)[i]; }
    static constexpr int kCount = 16;

    void clear() { *this = VehicleTriggers{}; }
};

// operator[] above treats the 16 bools as a contiguous array; guard that.
static_assert(sizeof(VehicleTriggers) == 16,
              "VehicleTriggers must be 16 tightly-packed bools for indexed access");

// Compact snapshot of FMU outputs read after each doStep. We deliberately read
// only what the simulation, recording, and context-aware schedulers need.
struct VehicleOutputs {
    double phys[6]       = {};   // full physical state x[0..5]  (VR 1000..1005);
                                 // [yaw rate, slip, steer angle, steer rate, e_y, e_y_dot]
    double e_y_real      = 0.0;  // ground-truth lateral error  (VR 1004, == phys[4])
    double e_y_est       = 0.0;  // estimated lateral error     (VR 1015)
    double act_out       = 0.0;  // applied steering command    (VR 1022)
    double rolling_real  = 0.0;  // real rolling performance    (VR 1030)
    double rolling_remote= 0.0;  // remote rolling performance  (VR 1029)
    double average_real  = 0.0;  // real average performance    (VR 1033)
    int    threshold_cntr_real = 0;  // soft-bound 10ms windows (VR 1036)
    bool   violated_real = false;    // |e_y| over soft bound   (VR 1039)
    bool   critical_real = false;    // in critical maneuver    (VR 1027)
    // The cloud's legitimate (estimator-derived) view of the two flags above,
    // for honest context-aware scheduling. Updated only when the estimator runs.
    bool   violated_remote = false;  // (VR 1038)
    bool   critical_remote = false;  // (VR 1026)
};

}  // namespace cps
