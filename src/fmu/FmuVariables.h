// FmuVariables.h — Value references and structural constants for the
// LateralMotionControl FMU, transcribed from LateralMotionControl/modelDescription.xml.
//
// The FMU is an FMI 2.0 Co-Simulation model of a vehicle lateral-motion control
// chain. It is driven entirely through these value references via the standard
// fmi2Get*/fmi2Set* calls. See modelDescription.xml for full descriptions.
#pragma once

#include <cstdint>

namespace cps::vr {

// --- FMU identity (from modelDescription.xml) ---
inline constexpr const char* kModelName = "LateralMotionControl";
inline constexpr const char* kGuid      = "{ec101913-52ec-40d8-afe6-5fbb52430f74}";

// Physical base step of the FMU: 0.1 ms. Every doStep advances a multiple of this.
inline constexpr double kBaseStepSeconds = 1e-4;

// Soft (comfort) and hard (safety) lateral-error bounds, in meters.
// See examples/constraints.md. ERROR_THRESHOLD is the soft bound.
inline constexpr double kSoftBound = 0.2;
inline constexpr double kHardBound = 0.8;

// Performance metrics are evaluated on a 10 ms granularity (in-car and remote).
inline constexpr double kMetricPeriodSeconds = 1e-2;

// --- Physical state vector layout (x[0..5]) ---
enum State : int {
    kYawRate        = 0,  // phi_dot
    kSlipAngle      = 1,  // beta
    kSteeringAngle  = 2,  // delta
    kSteeringRate   = 3,  // delta_dot
    kLateralError   = 4,  // e_y  <-- "the error" we visualize
    kLateralErrorDot= 5,  // e_y_dot
    kNumStates      = 6,
};

// ============================ Parameters (1..34) ============================
// Settable between fmi2SetupExperiment and fmi2ExitInitializationMode.
inline constexpr uint32_t kK_trc_fb_start   = 1;   // 1x6  -> 1..6
inline constexpr uint32_t kK_trc_ff_start   = 7;   // 1x2  -> 7..8
inline constexpr uint32_t kK_yrc_x_start    = 9;   // 1x6  -> 9..14
inline constexpr uint32_t kK_yrc_psi_start  = 15;  // 1x1  -> 15
inline constexpr uint32_t kX0_start         = 16;  // 6    -> 16..21
inline constexpr uint32_t kNoiseMean_start  = 22;  // 6    -> 22..27
inline constexpr uint32_t kNoiseStdDev_start= 28;  // 6    -> 28..33
inline constexpr uint32_t kInitVelocity     = 34;

// ============================ Inputs (100..118) ============================
// 16 boolean triggers, in protocol order, VR 100..115. The ordering here MUST
// match cps::VehicleTriggers (see sched/Scheduler.h).
enum Trigger : uint32_t {
    kSensorActivated    = 100,
    kSensorFinished     = 101,
    kNetworkScSent      = 102,  // sensor -> cloud, send
    kNetworkScReceived  = 103,  // sensor -> cloud, receive
    kEstimatorActivated = 104,
    kEstimatorFinished  = 105,
    kControllerActivated= 106,
    kControllerFinished = 107,
    kFeedforwardActivated=108,
    kFeedforwardFinished= 109,
    kMergerActivated    = 110,
    kMergerFinished     = 111,
    kNetworkCaSent      = 112,  // cloud -> actuator, send
    kNetworkCaReceived  = 113,  // cloud -> actuator, receive
    kActuatorActivated  = 114,
    kActuatorFinished   = 115,
};
inline constexpr uint32_t kTriggerFirst = 100;
inline constexpr int      kNumTriggers  = 16;

// Real inputs.
inline constexpr uint32_t kFfRef0   = 116;  // feedforward reference component 0
inline constexpr uint32_t kFfRef1   = 117;  // feedforward reference component 1
inline constexpr uint32_t kVelocity = 118;  // current vehicle velocity [m/s]

// ============================ Outputs (1000..1039) =========================
inline constexpr uint32_t kPhysState_start = 1000;  // 6 -> 1000..1005
inline constexpr uint32_t kSensOut_start   = 1006;  // 5 -> 1006..1010
inline constexpr uint32_t kEstStates_start = 1011;  // 6 -> 1011..1016
inline constexpr uint32_t kFbPsiDotOut     = 1017;
inline constexpr uint32_t kFfOut_start     = 1018;  // 2 -> 1018..1019
inline constexpr uint32_t kFfPsiDotOut     = 1020;
inline constexpr uint32_t kAggDeltaOut     = 1021;
inline constexpr uint32_t kActOut          = 1022;
inline constexpr uint32_t kCurrentTimeOut  = 1023;  // Real
inline constexpr uint32_t kCurrentStepOut  = 1024;  // Integer

// Direct output of the lateral error (== current_phys_state_4).
inline constexpr uint32_t kLateralErrorOut    = kPhysState_start + kLateralError;     // 1004
inline constexpr uint32_t kEstLateralErrorOut = kEstStates_start + kLateralError;     // 1015

// Critical section flags (Boolean).
inline constexpr uint32_t kCriticalLocal  = 1025;
inline constexpr uint32_t kCriticalRemote = 1026;
inline constexpr uint32_t kCriticalReal   = 1027;

// Rolling performance (Real).
inline constexpr uint32_t kRollingLocal  = 1028;
inline constexpr uint32_t kRollingRemote = 1029;
inline constexpr uint32_t kRollingReal   = 1030;

// Average performance (Real).
inline constexpr uint32_t kAverageLocal  = 1031;
inline constexpr uint32_t kAverageRemote = 1032;
inline constexpr uint32_t kAverageReal   = 1033;

// Threshold-error counter (Integer): count of 10 ms windows with |e_y| > soft bound.
inline constexpr uint32_t kThresholdCntrLocal  = 1034;
inline constexpr uint32_t kThresholdCntrRemote = 1035;
inline constexpr uint32_t kThresholdCntrReal   = 1036;

// Violated-constraint flag (Boolean): |e_y| currently over the soft bound.
inline constexpr uint32_t kViolatedLocal  = 1037;
inline constexpr uint32_t kViolatedRemote = 1038;
inline constexpr uint32_t kViolatedReal   = 1039;

}  // namespace cps::vr
