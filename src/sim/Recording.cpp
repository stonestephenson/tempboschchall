#include "sim/Recording.h"

#include <cstring>
#include <fstream>
#include <stdexcept>

namespace cps {
namespace {

constexpr char    kMagic[4] = {'C', 'P', 'S', 'R'};
constexpr int32_t kVersion  = 2;  // v2: VehicleSummary.max_data_age_ms

template <typename T>
void put(std::ostream& os, const T& v) {
    os.write(reinterpret_cast<const char*>(&v), sizeof(T));
}
template <typename T>
T get(std::istream& is) {
    T v{};
    is.read(reinterpret_cast<char*>(&v), sizeof(T));
    return v;
}
void putStr(std::ostream& os, const std::string& s) {
    put<int32_t>(os, static_cast<int32_t>(s.size()));
    os.write(s.data(), static_cast<std::streamsize>(s.size()));
}
std::string getStr(std::istream& is) {
    const int32_t n = get<int32_t>(is);
    std::string s(static_cast<size_t>(n), '\0');
    if (n > 0) is.read(&s[0], n);
    return s;
}

}  // namespace

void RunRecording::save(const std::string& path) const {
    std::ofstream os(path, std::ios::binary);
    if (!os) throw std::runtime_error("RunRecording: cannot write " + path);

    os.write(kMagic, 4);
    put(os, kVersion);
    put<int32_t>(os, profile);
    put<int32_t>(os, nVehicles);
    put<int32_t>(os, nCores);
    put<double>(os, baseStep);
    put<int64_t>(os, durationSteps);
    put<int32_t>(os, decimation);
    put<int64_t>(os, missedJobs);
    putStr(os, schedulerName);

    put<int32_t>(os, static_cast<int32_t>(startOffsets.size()));
    for (long off : startOffsets) put<int64_t>(os, static_cast<int64_t>(off));

    put<int32_t>(os, static_cast<int32_t>(summary.size()));
    for (const auto& s : summary) put(os, s);

    put<int32_t>(os, static_cast<int32_t>(frames.size()));
    for (const auto& vf : frames) {
        put<int64_t>(os, static_cast<int64_t>(vf.size()));
        if (!vf.empty())
            os.write(reinterpret_cast<const char*>(vf.data()),
                     static_cast<std::streamsize>(vf.size() * sizeof(Frame)));
    }
}

RunRecording RunRecording::load(const std::string& path) {
    std::ifstream is(path, std::ios::binary);
    if (!is) throw std::runtime_error("RunRecording: cannot read " + path);

    char magic[4];
    is.read(magic, 4);
    if (std::memcmp(magic, kMagic, 4) != 0)
        throw std::runtime_error("RunRecording: bad magic in " + path);
    if (get<int32_t>(is) != kVersion)
        throw std::runtime_error("RunRecording: unsupported version in " + path);

    RunRecording r;
    r.profile       = get<int32_t>(is);
    r.nVehicles     = get<int32_t>(is);
    r.nCores        = get<int32_t>(is);
    r.baseStep      = get<double>(is);
    r.durationSteps = static_cast<long>(get<int64_t>(is));
    r.decimation    = get<int32_t>(is);
    r.missedJobs    = static_cast<long>(get<int64_t>(is));
    r.schedulerName = getStr(is);

    const int32_t nOff = get<int32_t>(is);
    r.startOffsets.resize(nOff);
    for (int i = 0; i < nOff; ++i) r.startOffsets[i] = static_cast<long>(get<int64_t>(is));

    const int32_t nSum = get<int32_t>(is);
    r.summary.resize(nSum);
    for (int i = 0; i < nSum; ++i) r.summary[i] = get<VehicleSummary>(is);

    const int32_t nVeh = get<int32_t>(is);
    r.frames.resize(nVeh);
    for (int v = 0; v < nVeh; ++v) {
        const int64_t cnt = get<int64_t>(is);
        r.frames[v].resize(static_cast<size_t>(cnt));
        if (cnt > 0)
            is.read(reinterpret_cast<char*>(r.frames[v].data()),
                    static_cast<std::streamsize>(cnt * sizeof(Frame)));
    }
    return r;
}

}  // namespace cps
