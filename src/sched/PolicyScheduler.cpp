#include "sched/PolicyScheduler.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace cps {
namespace {

const char* taskAbbrev(TaskKind k) {
    switch (k) {
        case TaskKind::Estimator:   return "E";
        case TaskKind::Controller:  return "C";
        case TaskKind::Feedforward: return "F";
        case TaskKind::Merger:      return "M";
        case TaskKind::Sensor:      return "S";
        case TaskKind::Actuator:    return "A";
    }
    return "?";
}

int taskLane(TaskKind k) {
    switch (k) {
        case TaskKind::Estimator:   return 0;
        case TaskKind::Controller:  return 1;
        case TaskKind::Feedforward: return 2;
        case TaskKind::Merger:      return 3;
        case TaskKind::Sensor:
        case TaskKind::Actuator:    return 0;
    }
    return 0;
}

const char* vehicleColor(int vehicle) {
    static const char* colors[] = {
        "#2563eb", "#dc2626", "#059669", "#d97706",
        "#7c3aed", "#0891b2", "#db2777", "#4d7c0f",
        "#b45309", "#0f766e", "#be123c", "#4338ca"
    };
    return colors[vehicle % (sizeof(colors) / sizeof(colors[0]))];
}

double clampDouble(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

}  // namespace

PolicyScheduler::PolicyScheduler(std::unique_ptr<CorePolicy> policy, bool partitioned,
                                 ScheduleVizConfig scheduleViz)
    : policy_(std::move(policy)),
      partitioned_(partitioned),
      scheduleViz_(std::move(scheduleViz)) {
    name_ = std::string(partitioned_ ? "PartitionedPolicyScheduler[" : "PolicyScheduler[") +
            (policy_ ? policy_->name() : "null") + "]";
}

PolicyScheduler::~PolicyScheduler() {
    writeScheduleViz();
}

void PolicyScheduler::init(const SimConfig& cfg, const std::vector<TaskSet>& taskSets) {
    nCores_ = cfg.nCores;
    nVehicles_ = static_cast<int>(taskSets.size());
    baseStepSeconds_ = cfg.baseStep;
    models_.clear();
    models_.reserve(taskSets.size());
    for (int v = 0; v < static_cast<int>(taskSets.size()); ++v)
        models_.emplace_back(v, taskSets[v], cfg.baseStep);

    coreToReady_.assign(nCores_, -1);
    scheduleSamples_.clear();
    scheduleVizTicks_ = 0;
    if (scheduleViz_.enabled() && baseStepSeconds_ > 0.0) {
        scheduleVizTicks_ = std::max<long>(
            1, std::lround(scheduleViz_.windowMs / (baseStepSeconds_ * 1000.0)));
        scheduleSamples_.assign(nCores_, {});
        for (auto& coreSamples : scheduleSamples_)
            coreSamples.reserve(static_cast<size_t>(scheduleVizTicks_));
    }
}

void PolicyScheduler::onTick(double /*t*/, long step,
                             const std::vector<VehicleView>& views,
                             std::vector<VehicleTriggers>& out) {
    // 1. Release jobs and collect every vehicle's ready cloud jobs.
    ready_.clear();
    for (auto& m : models_) m.beginTick(step, ready_);

    // 2. Let the policy pick which ready jobs get the shared cores this tick.
    chosen_.clear();
    std::fill(coreToReady_.begin(), coreToReady_.end(), -1);
    if (!ready_.empty() && nCores_ > 0 && !partitioned_) {
        policy_->assign(ready_, nCores_, views, chosen_);
        const int n = std::min<int>(nCores_, static_cast<int>(chosen_.size()));
        for (int core = 0; core < n; ++core)
            coreToReady_[core] = chosen_[core];
    } else if (!ready_.empty() && nCores_ > 0) {
        for (int core = 0; core < nCores_; ++core) {
            partitionReady_.clear();
            partitionReadyToGlobal_.clear();
            for (int idx = 0; idx < static_cast<int>(ready_.size()); ++idx) {
                if (ready_[idx].vehicle % nCores_ != core) continue;
                partitionReady_.push_back(ready_[idx]);
                partitionReadyToGlobal_.push_back(idx);
            }
            if (partitionReady_.empty()) continue;

            partitionChosen_.clear();
            policy_->assign(partitionReady_, 1, views, partitionChosen_);
            for (int localIdx : partitionChosen_) {
                if (localIdx < 0 || localIdx >= static_cast<int>(partitionReadyToGlobal_.size()))
                    continue;
                const int globalIdx = partitionReadyToGlobal_[localIdx];
                coreToReady_[core] = globalIdx;
                chosen_.push_back(globalIdx);
                break;
            }
        }
    }
    recordScheduleTick(step);

    // 3. Grant the chosen cores.
    for (int idx : chosen_) {
        if (idx < 0 || idx >= static_cast<int>(ready_.size())) continue;
        const ReadyJob& j = ready_[idx];
        models_[j.vehicle].grantCore(j.kind);
    }

    // 4. Advance every model and emit this tick's triggers.
    for (size_t v = 0; v < models_.size(); ++v)
        models_[v].endTick(step, out[v]);
}

long PolicyScheduler::missedJobs() const {
    long total = 0;
    for (const auto& m : models_) total += m.missedJobs();
    return total;
}

long PolicyScheduler::maxDataAgeTicks(int vehicle) const {
    if (vehicle < 0 || vehicle >= static_cast<int>(models_.size())) return -1;
    return models_[vehicle].maxDataAgeTicks();
}

void PolicyScheduler::recordScheduleTick(long step) {
    if (!scheduleViz_.enabled() || step < 0 || step >= scheduleVizTicks_) return;
    if (scheduleSamples_.size() != static_cast<size_t>(nCores_)) return;

    for (int core = 0; core < nCores_; ++core) {
        CoreSample sample;
        const int readyIdx = core < static_cast<int>(coreToReady_.size()) ? coreToReady_[core] : -1;
        if (readyIdx >= 0 && readyIdx < static_cast<int>(ready_.size())) {
            sample.vehicle = ready_[readyIdx].vehicle;
            sample.kind = ready_[readyIdx].kind;
        }
        scheduleSamples_[core].push_back(sample);
    }
}

void PolicyScheduler::writeScheduleViz() {
    if (scheduleVizWritten_ || !scheduleViz_.enabled() || scheduleSamples_.empty())
        return;
    scheduleVizWritten_ = true;

    size_t ticks = 0;
    for (const auto& coreSamples : scheduleSamples_)
        ticks = std::max(ticks, coreSamples.size());
    if (ticks == 0 || baseStepSeconds_ <= 0.0) return;

    const double dtMs = baseStepSeconds_ * 1000.0;
    const double endMs = static_cast<double>(ticks) * dtMs;
    const int left = 106;
    const int top = 68;
    const int laneH = 10;
    const int rowH = laneH * 4;
    const int rowGap = 14;
    const int plotW = static_cast<int>(clampDouble(endMs * 10.0, 900.0, 2200.0));
    const int right = 34;
    const int legendItemW = 92;
    const int legendCols = std::max(1, plotW / legendItemW);
    const int legendRows = std::max(1, (nVehicles_ + legendCols - 1) / legendCols);
    const int plotH = nCores_ * (rowH + rowGap) - rowGap;
    const int legendTop = top + plotH + 54;
    const int width = left + plotW + right;
    const int height = legendTop + 44 + legendRows * 22;

    std::ofstream svg(scheduleViz_.path);
    if (!svg) {
        std::fprintf(stderr, "Warning: could not write schedule visualization to %s\n",
                     scheduleViz_.path.c_str());
        return;
    }

    const double scale = static_cast<double>(plotW) / static_cast<double>(ticks);
    const double majorMs = endMs <= 80.0 ? 5.0 : (endMs <= 200.0 ? 10.0 :
                           (endMs <= 500.0 ? 25.0 : 50.0));
    auto sameSample = [](const CoreSample& a, const CoreSample& b) {
        return a.vehicle == b.vehicle && a.kind == b.kind;
    };

    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " " << height
        << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\"/>\n";
    svg << "<text x=\"20\" y=\"28\" font-family=\"Arial, sans-serif\" font-size=\"18\" "
        << "font-weight=\"700\" fill=\"#111827\">Schedule core timeline</text>\n";
    svg << "<text x=\"20\" y=\"48\" font-family=\"Arial, sans-serif\" font-size=\"12\" "
        << "fill=\"#475569\">" << name_ << " first " << std::fixed << std::setprecision(1)
        << endMs << " ms; task lanes top-to-bottom: E, C, F, M</text>\n";

    for (double ms = 0.0; ms <= endMs + 1e-9; ms += majorMs) {
        const double x = left + (ms / endMs) * plotW;
        svg << "<line x1=\"" << x << "\" y1=\"" << (top - 16) << "\" x2=\"" << x
            << "\" y2=\"" << (top + plotH) << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
        svg << "<text x=\"" << x << "\" y=\"" << (top - 24)
            << "\" text-anchor=\"middle\" font-family=\"Arial, sans-serif\" font-size=\"10\" "
            << "fill=\"#475569\">" << std::setprecision(0) << ms << "ms</text>\n";
    }

    for (int core = 0; core < nCores_; ++core) {
        const int y = top + core * (rowH + rowGap);
        svg << "<text x=\"18\" y=\"" << (y + 19)
            << "\" font-family=\"Arial, sans-serif\" font-size=\"12\" font-weight=\"700\" "
            << "fill=\"#334155\">core " << core << "</text>\n";
        svg << "<rect x=\"" << left << "\" y=\"" << y << "\" width=\"" << plotW
            << "\" height=\"" << rowH << "\" fill=\"#e2e8f0\" stroke=\"#cbd5e1\"/>\n";
        const TaskKind laneKinds[] = {
            TaskKind::Estimator, TaskKind::Controller,
            TaskKind::Feedforward, TaskKind::Merger
        };
        for (int lane = 0; lane < 4; ++lane) {
            const int laneY = y + lane * laneH;
            if (lane % 2 == 1) {
                svg << "<rect x=\"" << left << "\" y=\"" << laneY << "\" width=\""
                    << plotW << "\" height=\"" << laneH << "\" fill=\"#dbe3ee\"/>\n";
            }
            if (lane > 0) {
                svg << "<line x1=\"" << left << "\" y1=\"" << laneY << "\" x2=\""
                    << (left + plotW) << "\" y2=\"" << laneY
                    << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
            }
            svg << "<text x=\"" << (left - 14) << "\" y=\"" << (laneY + 8)
                << "\" text-anchor=\"middle\" font-family=\"Arial, sans-serif\" "
                << "font-size=\"9\" font-weight=\"700\" fill=\"#64748b\">"
                << taskAbbrev(laneKinds[lane]) << "</text>\n";
        }

        const auto& samples = scheduleSamples_[core];
        size_t start = 0;
        while (start < samples.size()) {
            size_t end = start + 1;
            while (end < samples.size() && sameSample(samples[start], samples[end])) ++end;

            const CoreSample& s = samples[start];
            if (s.vehicle >= 0) {
                const double x = left + start * scale;
                const double w = std::max(1.0, (end - start) * scale);
                const int lane = taskLane(s.kind);
                const int laneY = y + lane * laneH;
                svg << "<rect x=\"" << x << "\" y=\"" << (laneY + 1) << "\" width=\"" << w
                    << "\" height=\"" << (laneH - 2) << "\" rx=\"2\" fill=\""
                    << vehicleColor(s.vehicle) << "\"/>\n";
                if (w > 34.0) {
                    svg << "<text x=\"" << (x + w / 2.0) << "\" y=\"" << (laneY + 8)
                        << "\" text-anchor=\"middle\" font-family=\"Arial, sans-serif\" "
                        << "font-size=\"8\" font-weight=\"700\" fill=\"white\">v"
                        << s.vehicle << "</text>\n";
                }
            }
            start = end;
        }
    }

    svg << "<text x=\"20\" y=\"" << legendTop
        << "\" font-family=\"Arial, sans-serif\" font-size=\"12\" font-weight=\"700\" "
        << "fill=\"#334155\">vehicles</text>\n";
    for (int v = 0; v < nVehicles_; ++v) {
        const int col = v % legendCols;
        const int row = v / legendCols;
        const int x = left + col * legendItemW;
        const int y = legendTop - 12 + row * 22;
        svg << "<rect x=\"" << x << "\" y=\"" << y << "\" width=\"12\" height=\"12\" "
            << "fill=\"" << vehicleColor(v) << "\"/>\n";
        svg << "<text x=\"" << (x + 18) << "\" y=\"" << (y + 10)
            << "\" font-family=\"Arial, sans-serif\" font-size=\"11\" fill=\"#334155\">v"
            << v << "</text>\n";
    }
    svg << "</svg>\n";
}

}  // namespace cps
