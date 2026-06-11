#include "trace/Trajectory.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace cps {
namespace {

// Half-width (in ticks) of the finite-difference stencil used to estimate the
// path tangent. The (x,y) trace is piecewise-constant (it jumps every few ms),
// so a ±1 stencil is degenerate; ±200 ticks spans several position updates and
// yields a smooth, robust direction without washing out track curvature.
constexpr long kNormalHalfStencil = 200;

// Read up to `count` whitespace/comma-separated floats from a one-column CSV.
std::vector<float> loadColumn(const std::string& path, long count) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) throw std::runtime_error("Trajectory: cannot open " + path);
    std::string data((std::istreambuf_iterator<char>(ifs)),
                     std::istreambuf_iterator<char>());

    std::vector<float> out;
    out.reserve(static_cast<size_t>(count));
    const char* p = data.c_str();
    char* end = nullptr;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ',') ++p;
        if (!*p) break;
        const double v = std::strtod(p, &end);
        if (end == p) break;  // no progress -> stop
        out.push_back(static_cast<float>(v));
        p = end;
        if (static_cast<long>(out.size()) >= count) break;
    }
    return out;
}

}  // namespace

ProfileInfo profileInfo(Profile p) {
    switch (p) {
        case Profile::V10:   return {"example_v_10",   10.0, 1178000};
        case Profile::V12_5: return {"example_v_12_5", 12.5,  944000};
        case Profile::V15:   return {"example_v_15",   15.0,  786000};
    }
    return {"example_v_10", 10.0, 1178000};
}

const char* profileName(Profile p) {
    switch (p) {
        case Profile::V10:   return "v10";
        case Profile::V12_5: return "v12.5";
        case Profile::V15:   return "v15";
    }
    return "v10";
}

std::string Trajectory::defaultExamplesDir() {
#ifdef EXAMPLES_DIR
    return EXAMPLES_DIR;
#else
    return "examples";
#endif
}

std::shared_ptr<Trajectory> Trajectory::load(Profile profile,
                                             const std::string& examplesDir) {
    const ProfileInfo info = profileInfo(profile);
    const std::string dir = examplesDir + "/" + info.dirName;

    std::shared_ptr<Trajectory> t(new Trajectory());
    t->profile_  = profile;
    t->lapSteps_ = info.lapSteps;
    t->peakVel_  = info.peakVelocity;

    t->x_   = loadColumn(dir + "/x_position_track.csv",     info.lapSteps);
    t->y_   = loadColumn(dir + "/y_position_track.csv",     info.lapSteps);
    t->vel_ = loadColumn(dir + "/velocity.csv",             info.lapSteps);
    t->ff0_ = loadColumn(dir + "/feedforward_sequence_0.csv", info.lapSteps);
    t->ff1_ = loadColumn(dir + "/feedforward_sequence_1.csv", info.lapSteps);

    const long need = info.lapSteps;
    auto check = [&](const std::vector<float>& v, const char* name) {
        if (static_cast<long>(v.size()) < need)
            throw std::runtime_error(std::string("Trajectory: ") + name + " in " + dir +
                                     " has fewer rows than the lap length");
    };
    check(t->x_, "x_position_track");
    check(t->y_, "y_position_track");
    check(t->vel_, "velocity");
    check(t->ff0_, "feedforward_sequence_0");
    check(t->ff1_, "feedforward_sequence_1");

    t->computeNormalsAndBounds();
    return t;
}

void Trajectory::computeNormalsAndBounds() {
    const long n = lapSteps_;
    nx_.resize(static_cast<size_t>(n));
    ny_.resize(static_cast<size_t>(n));

    min_ = {x_[0], y_[0]};
    max_ = {x_[0], y_[0]};
    Vec2 prevNormal{0.0f, 1.0f};

    for (long i = 0; i < n; ++i) {
        min_.x = std::min(min_.x, x_[i]);
        min_.y = std::min(min_.y, y_[i]);
        max_.x = std::max(max_.x, x_[i]);
        max_.y = std::max(max_.y, y_[i]);

        const Vec2 a = pointAt(i - kNormalHalfStencil);  // wraps around the lap
        const Vec2 b = pointAt(i + kNormalHalfStencil);
        Vec2 nrm = normalized(perp(b - a));
        if (length(nrm) < 0.5f) nrm = prevNormal;  // degenerate -> carry forward
        nx_[i] = nrm.x;
        ny_[i] = nrm.y;
        prevNormal = nrm;
    }
}

}  // namespace cps
