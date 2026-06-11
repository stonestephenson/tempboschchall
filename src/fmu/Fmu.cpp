#include "fmu/Fmu.h"

#include <dlfcn.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "fmu/FmuVariables.h"

namespace cps {
namespace {

// --- FMI callbacks. Stateless, so a single static instance can be shared by
// every FMU component and outlives them all (the FMU keeps these pointers). ---
void fmiLogger(fmi2ComponentEnvironment, fmi2String instance, fmi2Status status,
               fmi2String category, fmi2String message, ...) {
    if (status == fmi2OK || status == fmi2Pending) return;  // quiet on success
    char buf[1024];
    va_list args;
    va_start(args, message);
    std::vsnprintf(buf, sizeof(buf), message ? message : "", args);
    va_end(args);
    std::fprintf(stderr, "[FMU:%s][%s] %s\n", instance ? instance : "?",
                 category ? category : "?", buf);
}
void* fmiAlloc(size_t n, size_t sz) { return std::calloc(n, sz); }
void  fmiFree(void* p)              { std::free(p); }

const fmi2CallbackFunctions kCallbacks = {
    fmiLogger, fmiAlloc, fmiFree, /*stepFinished=*/nullptr,
    /*componentEnvironment=*/nullptr};

// Resolve a prefixed symbol or throw.
template <typename T>
T resolve(void* handle, const char* suffix) {
    std::string name = std::string("LateralMotionControl_") + suffix;
    void* sym = dlsym(handle, name.c_str());
    if (!sym) throw std::runtime_error("FMU symbol not found: " + name);
    return reinterpret_cast<T>(sym);
}

}  // namespace

// ----------------------------------------------------------------------------
// FmuLibrary
// ----------------------------------------------------------------------------
std::string FmuLibrary::defaultDylibPath() {
#ifdef FMU_DYLIB_PATH
    return FMU_DYLIB_PATH;
#else
    return "LateralMotionControl/binaries/darwin64/LateralMotionControl.dylib";
#endif
}

FmuLibrary::FmuLibrary(const std::string& dylibPath) {
    handle_ = dlopen(dylibPath.c_str(), RTLD_LAZY | RTLD_LOCAL);
    if (!handle_)
        throw std::runtime_error("dlopen failed for '" + dylibPath + "': " +
                                 (dlerror() ? dlerror() : "unknown"));

    api_.instantiate     = resolve<fmi2InstantiateTYPE*>(handle_, "fmi2Instantiate");
    api_.freeInstance    = resolve<fmi2FreeInstanceTYPE*>(handle_, "fmi2FreeInstance");
    api_.setupExperiment = resolve<fmi2SetupExperimentTYPE*>(handle_, "fmi2SetupExperiment");
    api_.enterInit       = resolve<fmi2EnterInitializationModeTYPE*>(handle_, "fmi2EnterInitializationMode");
    api_.exitInit        = resolve<fmi2ExitInitializationModeTYPE*>(handle_, "fmi2ExitInitializationMode");
    api_.terminate       = resolve<fmi2TerminateTYPE*>(handle_, "fmi2Terminate");
    api_.reset           = resolve<fmi2ResetTYPE*>(handle_, "fmi2Reset");
    api_.doStep          = resolve<fmi2DoStepTYPE*>(handle_, "fmi2DoStep");
    api_.setReal         = resolve<fmi2SetRealTYPE*>(handle_, "fmi2SetReal");
    api_.setInteger      = resolve<fmi2SetIntegerTYPE*>(handle_, "fmi2SetInteger");
    api_.setBoolean      = resolve<fmi2SetBooleanTYPE*>(handle_, "fmi2SetBoolean");
    api_.getReal         = resolve<fmi2GetRealTYPE*>(handle_, "fmi2GetReal");
    api_.getInteger      = resolve<fmi2GetIntegerTYPE*>(handle_, "fmi2GetInteger");
    api_.getBoolean      = resolve<fmi2GetBooleanTYPE*>(handle_, "fmi2GetBoolean");
}

FmuLibrary::~FmuLibrary() {
    if (handle_) dlclose(handle_);
}

FmuLibrary::FmuLibrary(FmuLibrary&& o) noexcept : handle_(o.handle_), api_(o.api_) {
    o.handle_ = nullptr;
}
FmuLibrary& FmuLibrary::operator=(FmuLibrary&& o) noexcept {
    if (this != &o) {
        if (handle_) dlclose(handle_);
        handle_ = o.handle_;
        api_ = o.api_;
        o.handle_ = nullptr;
    }
    return *this;
}

std::unique_ptr<FmuInstance> FmuLibrary::instantiate(const std::string& instanceName) const {
    void* comp = api_.instantiate(instanceName.c_str(), fmi2CoSimulation, vr::kGuid,
                                  /*resourceLocation=*/"", &kCallbacks,
                                  /*visible=*/fmi2False, /*loggingOn=*/fmi2False);
    if (!comp)
        throw std::runtime_error("fmi2Instantiate failed for '" + instanceName + "'");
    return std::unique_ptr<FmuInstance>(new FmuInstance(api_, comp, instanceName));
}

// ----------------------------------------------------------------------------
// FmuInstance
// ----------------------------------------------------------------------------
FmuInstance::FmuInstance(const FmuApi& api, void* component, std::string name)
    : api_(api), comp_(component), name_(std::move(name)) {}

FmuInstance::~FmuInstance() {
    if (comp_) {
        if (!terminated_) api_.terminate(comp_);
        api_.freeInstance(comp_);
    }
}

void FmuInstance::initialize(double startTime, double stopTime, double initVelocity) {
    api_.setupExperiment(comp_, fmi2False, 0.0, startTime, fmi2True, stopTime);
    api_.enterInit(comp_);

    // IMPORTANT: this FMU initializes every parameter to ZERO and relies on the
    // importer to push the start values from modelDescription.xml (it does not
    // self-apply them). Without this the control gains are 0 -> the controller
    // produces no steering -> the vehicle runs open-loop and diverges. Set the
    // documented start values for VR 1..33, then init_velocity (VR 34).
    static const fmi2Real kParamDefaults[33] = {
        -0.987, 0.0, 0.0, 0.0, 8.00, 1.75,   // K_trc_fb   (VR 1..6)
        1.987, 0.1974,                        // K_trc_ff   (VR 7..8)
        -0.2876, 0.0, 1.0, 0.0, 0.0, 0.0,     // K_yrc_x    (VR 9..14)
        0.2876,                               // K_yrc_psi  (VR 15)
        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,         // x0         (VR 16..21)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         // noise_mean (VR 22..27)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         // noise_std  (VR 28..33)
    };
    fmi2ValueReference paramVrs[33];
    for (unsigned i = 0; i < 33; ++i) paramVrs[i] = i + 1;
    api_.setReal(comp_, paramVrs, 33, kParamDefaults);

    fmi2ValueReference vrVel = vr::kInitVelocity;  // VR 34
    fmi2Real valVel = initVelocity;
    api_.setReal(comp_, &vrVel, 1, &valVel);

    api_.exitInit(comp_);
}

void FmuInstance::setInputs(double ffRef0, double ffRef1, double velocity) {
    fmi2ValueReference vrs[3];
    fmi2Real vals[3];
    int n = 0;
    if (!inputsInitialized_ || ffRef0 != lastFf0_)   { vrs[n] = vr::kFfRef0;   vals[n] = ffRef0;   ++n; }
    if (!inputsInitialized_ || ffRef1 != lastFf1_)   { vrs[n] = vr::kFfRef1;   vals[n] = ffRef1;   ++n; }
    if (!inputsInitialized_ || velocity != lastVel_) { vrs[n] = vr::kVelocity; vals[n] = velocity; ++n; }
    if (n > 0) api_.setReal(comp_, vrs, static_cast<size_t>(n), vals);
    lastFf0_ = ffRef0;
    lastFf1_ = ffRef1;
    lastVel_ = velocity;
    inputsInitialized_ = true;
}

void FmuInstance::applyTriggers(const VehicleTriggers& t) {
    fmi2ValueReference vrs[VehicleTriggers::kCount];
    fmi2Boolean vals[VehicleTriggers::kCount];
    int n = 0;
    for (int i = 0; i < VehicleTriggers::kCount; ++i) {
        const bool nv = t[i];
        if (!triggersInitialized_ || nv != lastTriggers_[i]) {
            vrs[n] = vr::kTriggerFirst + static_cast<fmi2ValueReference>(i);
            vals[n] = nv ? fmi2True : fmi2False;
            ++n;
        }
    }
    if (n > 0) api_.setBoolean(comp_, vrs, static_cast<size_t>(n), vals);
    lastTriggers_ = t;
    triggersInitialized_ = true;
}

bool FmuInstance::doStep(double currentTime, double stepSize) {
    return api_.doStep(comp_, currentTime, stepSize, fmi2False) == fmi2OK;
}

VehicleOutputs FmuInstance::readOutputs() const {
    VehicleOutputs out;

    const fmi2ValueReference realVrs[6] = {
        vr::kLateralErrorOut, vr::kEstLateralErrorOut, vr::kActOut,
        vr::kRollingReal, vr::kRollingRemote, vr::kAverageReal};
    fmi2Real rv[6];
    api_.getReal(comp_, realVrs, 6, rv);
    out.e_y_real       = rv[0];
    out.e_y_est        = rv[1];
    out.act_out        = rv[2];
    out.rolling_real   = rv[3];
    out.rolling_remote = rv[4];
    out.average_real   = rv[5];

    const fmi2ValueReference intVrs[1] = {vr::kThresholdCntrReal};
    fmi2Integer iv[1];
    api_.getInteger(comp_, intVrs, 1, iv);
    out.threshold_cntr_real = iv[0];

    const fmi2ValueReference boolVrs[2] = {vr::kViolatedReal, vr::kCriticalReal};
    fmi2Boolean bv[2];
    api_.getBoolean(comp_, boolVrs, 2, bv);
    out.violated_real = bv[0] != fmi2False;
    out.critical_real = bv[1] != fmi2False;

    return out;
}

double FmuInstance::readReal(unsigned int vr) const {
    fmi2ValueReference v = vr;
    fmi2Real out = 0.0;
    api_.getReal(comp_, &v, 1, &out);
    return out;
}

void FmuInstance::terminate() {
    if (comp_ && !terminated_) {
        api_.terminate(comp_);
        terminated_ = true;
    }
}

}  // namespace cps
