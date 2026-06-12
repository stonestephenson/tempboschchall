// Trajectory.h — loads one velocity profile's reference traces (the closed
// racetrack centerline plus the per-tick FMU inputs) from the examples CSVs.
//
// The (x,y) trace is the *expected/reference* path. The FMU is fed velocity and
// feedforward references from the same time base (1:1 by tick at 0.1 ms). The
// actual driven path is reconstructed elsewhere as point + e_y * normal.
//
// All accessors wrap modulo the lap length, so the track loops seamlessly and a
// vehicle can be staggered onto the track with an integer start offset.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/Vec2.h"

namespace cps {

enum class Profile { V10, V12_5, V15 };

struct ProfileInfo {
    const char* dirName;
    double      peakVelocity;  // m/s
    long        lapSteps;      // ticks (0.1 ms) for one full lap
};
ProfileInfo profileInfo(Profile p);
const char* profileName(Profile p);

class Trajectory {
public:
    // Load all six CSVs for `profile` from `examplesDir`. Throws on failure.
    static std::shared_ptr<Trajectory> load(
        Profile profile, const std::string& examplesDir = defaultExamplesDir());

    static std::string defaultExamplesDir();

    Profile profile()      const { return profile_; }
    long    lapSteps()     const { return lapSteps_; }
    double  peakVelocity() const { return peakVel_; }

    // Map any (possibly offset/overrun) tick into [0, lapSteps).
    long wrap(long idx) const {
        idx %= lapSteps_;
        return idx < 0 ? idx + lapSteps_ : idx;
    }

    struct Inputs { float ff0, ff1, vel; };
    Inputs inputsAt(long step) const {
        const long i = wrap(step);
        return {ff0_[i], ff1_[i], vel_[i]};
    }
    Vec2 pointAt(long step)  const { const long i = wrap(step); return {x_[i], y_[i]}; }
    Vec2 normalAt(long step) const { const long i = wrap(step); return {nx_[i], ny_[i]}; }

    // AABB of one lap, for camera framing.
    Vec2 minBound() const { return min_; }
    Vec2 maxBound() const { return max_; }

private:
    Trajectory() = default;
    void computeNormalsAndBounds();

    Profile profile_ = Profile::V10;
    long    lapSteps_ = 0;
    double  peakVel_  = 0.0;
    std::vector<float> x_, y_, vel_, ff0_, ff1_;  // length == lapSteps_
    std::vector<float> nx_, ny_;                  // unit path normal per tick
    Vec2 min_{}, max_{};
};

}  // namespace cps
