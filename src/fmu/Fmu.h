// Fmu.h — Thin C++ wrapper around the LateralMotionControl FMI 2.0 Co-Simulation FMU.
//
// FmuLibrary loads the shared library once (dlopen) and resolves the FMI
// function pointers. FmuInstance wraps a single fmi2Component; many instances
// can share one library (the FMU declares canBeInstantiatedOnlyOncePerProcess
// = false and keeps all mutable state per-instance), which is what lets us run
// N vehicles in one process.
#pragma once

#include <memory>
#include <string>

#include "fmi2Functions.h"        // brings in fmi2FunctionTypes / fmi2TypesPlatform
#include "fmu/FmuTypes.h"

namespace cps {

// Resolved FMI 2.0 CS entry points (function-type pointers from the FMI header).
struct FmuApi {
    fmi2InstantiateTYPE*              instantiate          = nullptr;
    fmi2FreeInstanceTYPE*             freeInstance         = nullptr;
    fmi2SetupExperimentTYPE*          setupExperiment      = nullptr;
    fmi2EnterInitializationModeTYPE*  enterInit            = nullptr;
    fmi2ExitInitializationModeTYPE*   exitInit             = nullptr;
    fmi2TerminateTYPE*                terminate            = nullptr;
    fmi2ResetTYPE*                    reset                = nullptr;
    fmi2DoStepTYPE*                   doStep               = nullptr;
    fmi2SetRealTYPE*                  setReal              = nullptr;
    fmi2SetIntegerTYPE*               setInteger           = nullptr;
    fmi2SetBooleanTYPE*               setBoolean           = nullptr;
    fmi2GetRealTYPE*                  getReal              = nullptr;
    fmi2GetIntegerTYPE*               getInteger           = nullptr;
    fmi2GetBooleanTYPE*               getBoolean           = nullptr;
};

class FmuInstance;

// Owns the dlopen handle and resolved symbols. Move-only.
class FmuLibrary {
public:
    // Loads the dylib. Defaults to the compile-time FMU_DYLIB_PATH (set by CMake);
    // pass an explicit path to override. Throws std::runtime_error on failure.
    explicit FmuLibrary(const std::string& dylibPath = defaultDylibPath());

    ~FmuLibrary();
    FmuLibrary(FmuLibrary&&) noexcept;
    FmuLibrary& operator=(FmuLibrary&&) noexcept;
    FmuLibrary(const FmuLibrary&) = delete;
    FmuLibrary& operator=(const FmuLibrary&) = delete;

    // Instantiate one FMU component. `instanceName` is for logging only.
    std::unique_ptr<FmuInstance> instantiate(const std::string& instanceName) const;

    const FmuApi& api() const { return api_; }
    static std::string defaultDylibPath();

private:
    void*  handle_ = nullptr;
    FmuApi api_{};
};

// Wraps a single fmi2Component and provides typed, change-detected I/O.
class FmuInstance {
public:
    FmuInstance(const FmuApi& api, void* component, std::string name);
    ~FmuInstance();
    FmuInstance(const FmuInstance&) = delete;
    FmuInstance& operator=(const FmuInstance&) = delete;

    // setupExperiment -> set init_velocity parameter -> enter/exit init mode.
    void initialize(double startTime, double stopTime, double initVelocity);

    // Set the real inputs for the upcoming step. Redundant writes are skipped
    // (only changed values are pushed to the FMU).
    void setInputs(double ffRef0, double ffRef1, double velocity);

    // Set the 16 boolean triggers for the upcoming step. Only flags that changed
    // since the previous call are written, so steady all-false ticks cost nothing
    // and one-shot pulses are cleared automatically on the next tick.
    void applyTriggers(const VehicleTriggers& t);

    // Advance the FMU by `stepSize` seconds (we call this with one base step).
    bool doStep(double currentTime, double stepSize);

    // Batched read of the outputs the simulation/recording/scheduler need.
    VehicleOutputs readOutputs() const;

    // Read a single real output by value reference (diagnostics).
    double readReal(unsigned int vr) const;

    void terminate();

private:
    const FmuApi& api_;
    void*         comp_ = nullptr;
    std::string   name_;
    bool          terminated_ = false;

    // Change-detection caches.
    VehicleTriggers lastTriggers_{};
    bool   triggersInitialized_ = false;
    double lastFf0_ = 0.0, lastFf1_ = 0.0, lastVel_ = 0.0;
    bool   inputsInitialized_ = false;
};

}  // namespace cps
