#include "sched/PolicyScheduler.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace cps {

namespace {

int taskLane(TaskKind kind) {
    switch (kind) {
        case TaskKind::Estimator:   return 0;
        case TaskKind::Controller:  return 1;
        case TaskKind::Feedforward: return 2;
        case TaskKind::Merger:      return 3;
        default:                    return -1;
    }
}

const char* taskLaneLabel(int lane) {
    static const char* labels[] = {"E", "C", "F", "M"};
    return (lane >= 0 && lane < 4) ? labels[lane] : "";
}

const char* vehicleColor(int vehicle) {
    static const char* colors[] = {
        "#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#17becf",
        "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#005f73", "#ae2012"
    };
    return colors[vehicle % (sizeof(colors) / sizeof(colors[0]))];
}

}  // namespace

PolicyScheduler::PolicyScheduler(std::unique_ptr<CorePolicy> policy,
                                 bool partitioned,
                                 ScheduleVizConfig viz)
    : policy_(std::move(policy)), partitioned_(partitioned), viz_(std::move(viz)) {
    name_ = std::string(partitioned_ ? "PartitionedPolicyScheduler["
                                     : "PolicyScheduler[") +
            (policy_ ? policy_->name() : "null") + "]";
}

PolicyScheduler::~PolicyScheduler() {
    writeScheduleViz();
}

void PolicyScheduler::init(const SimConfig& cfg, const std::vector<TaskSet>& taskSets) {
    nCores_ = cfg.nCores;
    baseStepSeconds_ = cfg.baseStep;
    models_.clear();
    models_.reserve(taskSets.size());
    for (int v = 0; v < static_cast<int>(taskSets.size()); ++v)
        models_.emplace_back(v, taskSets[v], cfg.baseStep);
    vizSpans_.clear();
    vizLastSpanByLane_.assign(static_cast<size_t>(std::max(0, nCores_) * 4), -1);
    vizWritten_ = false;
}

void PolicyScheduler::onTick(double /*t*/, long step,
                             const std::vector<VehicleView>& views,
                             std::vector<VehicleTriggers>& out) {
    // 1. Release jobs and collect every vehicle's ready cloud jobs.
    ready_.clear();
    for (auto& m : models_) m.beginTick(step, ready_);

    // 2. Let the policy pick which ready jobs get the shared cores this tick.
    chosen_.clear();
    if (!ready_.empty() && nCores_ > 0 && policy_) {
        if (partitioned_) {
            for (int core = 0; core < nCores_; ++core) {
                partitionReady_.clear();
                partitionMap_.clear();
                for (int i = 0; i < static_cast<int>(ready_.size()); ++i) {
                    if (ready_[i].vehicle % nCores_ != core) continue;
                    partitionMap_.push_back(i);
                    partitionReady_.push_back(ready_[i]);
                }
                if (partitionReady_.empty()) continue;

                partitionChosen_.clear();
                policy_->assign(partitionReady_, 1, views, partitionChosen_);
                for (int localIdx : partitionChosen_) {
                    if (localIdx < 0 || localIdx >= static_cast<int>(partitionMap_.size()))
                        continue;
                    const int globalIdx = partitionMap_[localIdx];
                    chosen_.push_back(globalIdx);
                    recordVizGrant(step, ready_[globalIdx], core);
                    break;
                }
            }
        } else {
            policy_->assign(ready_, nCores_, views, chosen_);
            for (int i = 0; i < static_cast<int>(chosen_.size()); ++i) {
                const int idx = chosen_[i];
                if (idx < 0 || idx >= static_cast<int>(ready_.size())) continue;
                if (i >= nCores_) break;
                recordVizGrant(step, ready_[idx], i);
            }
        }
    }

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
    writeScheduleViz();
    long total = 0;
    for (const auto& m : models_) total += m.missedJobs();
    return total;
}

long PolicyScheduler::maxDataAgeTicks(int vehicle) const {
    if (vehicle < 0 || vehicle >= static_cast<int>(models_.size())) return -1;
    return models_[vehicle].maxDataAgeTicks();
}

long PolicyScheduler::maxDataAgeOldestTicks(int vehicle) const {
    if (vehicle < 0 || vehicle >= static_cast<int>(models_.size())) return -1;
    return models_[vehicle].maxDataAgeOldestTicks();
}

void PolicyScheduler::recordVizGrant(long step, const ReadyJob& job, int core) {
    if (!viz_.enabled() || core < 0 || core >= nCores_) return;
    const double startMs = static_cast<double>(step) * baseStepSeconds_ * 1000.0;
    if (startMs >= viz_.maxMs) return;
    const int lane = taskLane(job.kind);
    if (lane < 0) return;

    const int key = core * 4 + lane;
    if (key >= 0 && key < static_cast<int>(vizLastSpanByLane_.size())) {
        const int lastIdx = vizLastSpanByLane_[key];
        if (lastIdx >= 0 && lastIdx < static_cast<int>(vizSpans_.size())) {
            VizSpan& last = vizSpans_[lastIdx];
            if (last.core == core && last.vehicle == job.vehicle &&
                last.kind == job.kind && last.endStep == step) {
                last.endStep = step + 1;
                return;
            }
        }
    }

    VizSpan span;
    span.core = core;
    span.vehicle = job.vehicle;
    span.kind = job.kind;
    span.startStep = step;
    span.endStep = step + 1;
    vizSpans_.push_back(span);
    if (key >= 0 && key < static_cast<int>(vizLastSpanByLane_.size()))
        vizLastSpanByLane_[key] = static_cast<int>(vizSpans_.size()) - 1;
}

void PolicyScheduler::writeScheduleViz() const {
    if (vizWritten_ || !viz_.enabled()) return;
    vizWritten_ = true;

    const double maxMs = std::max(1.0, viz_.maxMs);
    const double pxPerMs = 8.0;
    const int left = 86;
    const int right = 24;
    const int top = 74;
    const int axisY = 42;
    const int laneH = 10;
    const int laneGap = 4;
    const int rowGap = 18;
    const int rowH = 4 * laneH + 3 * laneGap;
    const int plotW = static_cast<int>(maxMs * pxPerMs);
    const int width = left + plotW + right;
    const int height = top + nCores_ * rowH + std::max(0, nCores_ - 1) * rowGap + 70;

    std::ofstream os(viz_.path);
    if (!os) {
        std::fprintf(stderr, "Warning: cannot write schedule SVG %s\n", viz_.path.c_str());
        return;
    }

    os << std::fixed << std::setprecision(2);
    os << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
       << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " "
       << height << "\">\n";
    os << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    os << "<style>"
       << "text{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;"
       << "font-size:12px;fill:#263238}"
       << ".small{font-size:10px;fill:#607d8b}"
       << ".axis{stroke:#90a4ae;stroke-width:1}"
       << ".grid{stroke:#eceff1;stroke-width:1}"
       << ".lane{fill:#f8fafc;stroke:#d8e0e8;stroke-width:1}"
       << ".bar{stroke:#263238;stroke-width:.4}"
       << "</style>\n";
    os << "<text x=\"" << left << "\" y=\"24\" font-size=\"15\" font-weight=\"600\">"
       << name_ << " schedule, first " << maxMs << " ms</text>\n";

    const int legendY = 48;
    int legendX = left;
    for (int v = 0; v < static_cast<int>(models_.size()); ++v) {
        os << "<rect x=\"" << legendX << "\" y=\"" << legendY - 9
           << "\" width=\"10\" height=\"10\" fill=\"" << vehicleColor(v) << "\"/>\n";
        os << "<text class=\"small\" x=\"" << legendX + 14 << "\" y=\"" << legendY
           << "\">v" << v << "</text>\n";
        legendX += 42;
    }

    os << "<line class=\"axis\" x1=\"" << left << "\" y1=\"" << axisY
       << "\" x2=\"" << left + plotW << "\" y2=\"" << axisY << "\"/>\n";
    const int tickEvery = maxMs <= 80.0 ? 10 : 20;
    for (int ms = 0; ms <= static_cast<int>(maxMs + 0.5); ms += tickEvery) {
        const double x = left + ms * pxPerMs;
        os << "<line class=\"grid\" x1=\"" << x << "\" y1=\"" << axisY
           << "\" x2=\"" << x << "\" y2=\"" << height - 42 << "\"/>\n";
        os << "<line class=\"axis\" x1=\"" << x << "\" y1=\"" << axisY - 4
           << "\" x2=\"" << x << "\" y2=\"" << axisY + 4 << "\"/>\n";
        os << "<text class=\"small\" text-anchor=\"middle\" x=\"" << x
           << "\" y=\"" << axisY - 10 << "\">" << ms << "</text>\n";
    }
    os << "<text class=\"small\" x=\"" << left + plotW - 18 << "\" y=\"24\">ms</text>\n";

    for (int core = 0; core < nCores_; ++core) {
        const int y0 = top + core * (rowH + rowGap);
        os << "<text text-anchor=\"end\" x=\"" << left - 34 << "\" y=\"" << y0 + 26
           << "\">core " << core << "</text>\n";
        for (int lane = 0; lane < 4; ++lane) {
            const int y = y0 + lane * (laneH + laneGap);
            os << "<text class=\"small\" text-anchor=\"end\" x=\"" << left - 10
               << "\" y=\"" << y + laneH - 1 << "\">" << taskLaneLabel(lane) << "</text>\n";
            os << "<rect class=\"lane\" x=\"" << left << "\" y=\"" << y
               << "\" width=\"" << plotW << "\" height=\"" << laneH << "\"/>\n";
        }
    }

    for (const VizSpan& span : vizSpans_) {
        const int lane = taskLane(span.kind);
        if (lane < 0 || span.core < 0 || span.core >= nCores_) continue;
        const double x = left + span.startStep * baseStepSeconds_ * 1000.0 * pxPerMs;
        const double endMs = std::min(maxMs, span.endStep * baseStepSeconds_ * 1000.0);
        const double startMs = span.startStep * baseStepSeconds_ * 1000.0;
        const double w = std::max(0.8, (endMs - startMs) * pxPerMs);
        const int y0 = top + span.core * (rowH + rowGap);
        const int y = y0 + lane * (laneH + laneGap);
        os << "<rect class=\"bar\" x=\"" << x << "\" y=\"" << y + 1
           << "\" width=\"" << w << "\" height=\"" << laneH - 2
           << "\" fill=\"" << vehicleColor(span.vehicle) << "\">\n";
        os << "<title>core " << span.core << ", vehicle " << span.vehicle << ", "
           << taskKindName(span.kind) << ", "
           << startMs << "-" << endMs << " ms</title>\n";
        os << "</rect>\n";
    }

    const int footerY = height - 22;
    os << "<text class=\"small\" x=\"" << left << "\" y=\"" << footerY
       << "\">lanes: E estimator, C controller, F feedforward, M merger</text>\n";
    os << "</svg>\n";
}

}  // namespace cps
