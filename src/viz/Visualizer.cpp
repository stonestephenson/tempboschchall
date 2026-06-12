#include "viz/Visualizer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "raylib.h"
#include "fmu/FmuVariables.h"
#include "sim/Simulation.h"

namespace cps {
namespace {

// World (y-up) -> raylib screen space (y-down) before the Camera2D transform.
inline Vector2 rl(Vec2 p) { return Vector2{p.x, -p.y}; }

inline Color lerpC(Color a, Color b, float u) {
    u = u < 0 ? 0 : (u > 1 ? 1 : u);
    return Color{(unsigned char)(a.r + (b.r - a.r) * u),
                 (unsigned char)(a.g + (b.g - a.g) * u),
                 (unsigned char)(a.b + (b.b - a.b) * u),
                 (unsigned char)(a.a + (b.a - a.a) * u)};
}

// Green within comfort, ramping to red at the hard safety bound, dark beyond.
inline Color errorColor(float absEy) {
    if (absEy <= (float)vr::kSoftBound)
        return lerpC(Color{60, 200, 90, 255}, Color{235, 200, 60, 255},
                     absEy / (float)vr::kSoftBound);
    if (absEy <= (float)vr::kHardBound)
        return lerpC(Color{235, 200, 60, 255}, Color{230, 60, 50, 255},
                     (absEy - (float)vr::kSoftBound) /
                         (float)(vr::kHardBound - vr::kSoftBound));
    return Color{150, 30, 30, 255};
}

const Color kBg{18, 18, 24, 255};

}  // namespace

Visualizer::Visualizer(std::shared_ptr<Trajectory> traj, const VizConfig& cfg)
    : traj_(std::move(traj)), cfg_(cfg), selected_(std::max(0, cfg.initialSelected)),
      exag_(cfg.exaggeration) {
    playbackSpeed_ = std::max(0.1, cfg.initialSpeed);
}

void Visualizer::replay(const RunRecording& rec) {
    rec_ = &rec;
    sim_ = nullptr;
    cursor_ = 0.0;
    loop();
}

void Visualizer::live(Simulation& sim) {
    sim_ = &sim;
    rec_ = &sim.recording();
    cursor_ = 0.0;
    loop();
}

int Visualizer::availableFrames() const { return rec_ ? rec_->frameCount() : 0; }

Vec2 Visualizer::framePos(int v, int idx) const {
    return traj_->pointAt(rec_->frames[v][idx].refStep);
}

Vec2 Visualizer::frameActualPos(int v, int idx) const {
    const Frame& f = rec_->frames[v][idx];
    const Vec2 p = traj_->pointAt(f.refStep);
    const Vec2 n = traj_->normalAt(f.refStep);
    return p + n * (f.e_y_real * exag_);
}

// State shared across the per-frame draw helpers (set at the top of each frame).
namespace {
float gZoom = 1.0f;   // active camera zoom, for pixel-constant line widths
int   gFrame = 0;     // active frame index
}

void Visualizer::loop() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(cfg_.screenW, cfg_.screenH, "CPS Challenge Visualizer");
    SetTargetFPS(cfg_.targetFps);

    // Downsample the lap into a centerline + normal polyline for the track.
    const long n = traj_->lapSteps();
    const long step = std::max<long>(1, n / 4000);
    for (long i = 0; i < n; i += step) {
        trackPos_.push_back(traj_->pointAt(i));
        trackNrm_.push_back(traj_->normalAt(i));
    }
    trackPos_.push_back(traj_->pointAt(0));   // close the loop
    trackNrm_.push_back(traj_->normalAt(0));

    const Vec2 mn = traj_->minBound();
    const Vec2 mx = traj_->maxBound();
    trackCenter_ = {(mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f};

    long frameCount = 0;
    while (!WindowShouldClose()) {
        handleInput();
        updateCursor();

        BeginDrawing();
        ClearBackground(kBg);
        drawScene();
        drawErrorStrip();
        drawTimeline();
        drawHud();
        EndDrawing();

        if (!cfg_.screenshotPath.empty() && ++frameCount >= cfg_.screenshotAtFrame) {
            TakeScreenshot(cfg_.screenshotPath.c_str());
            break;
        }
    }
    CloseWindow();
}

void Visualizer::handleInput() {
    const float dt = GetFrameTime();
    const int nVeh = rec_->nVehicles;
    const bool scrubbable = !sim_ || sim_->finished();

    if (IsKeyPressed(KEY_SPACE)) playing_ = !playing_;
    if (IsKeyPressed(KEY_F)) follow_ = !follow_;
    if (IsKeyPressed(KEY_H)) showHelp_ = !showHelp_;
    if (nVeh > 0) {
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) selected_ = (selected_ + 1) % nVeh;
        if (IsKeyPressed(KEY_LEFT_BRACKET))  selected_ = (selected_ - 1 + nVeh) % nVeh;
    }
    if (IsKeyPressed(KEY_UP))   playbackSpeed_ = std::min(200.0, playbackSpeed_ * 1.5);
    if (IsKeyPressed(KEY_DOWN)) playbackSpeed_ = std::max(0.1, playbackSpeed_ / 1.5);
    if (IsKeyDown(KEY_PERIOD)) exag_ = std::min(200.0f, exag_ * (1.0f + 1.5f * dt));
    if (IsKeyDown(KEY_COMMA))  exag_ = std::max(1.0f, exag_ / (1.0f + 1.5f * dt));

    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) userZoom_ = std::clamp(userZoom_ * (1.0f + wheel * 0.12f), 0.2f, 40.0f);

    // Manual scrubbing (replay or finished live run).
    if (scrubbable) {
        const int avail = availableFrames();
        const double perSec = playbackSpeed_ / (rec_->decimation * rec_->baseStep);
        if (IsKeyDown(KEY_LEFT))  { cursor_ -= perSec * dt; playing_ = false; }
        if (IsKeyDown(KEY_RIGHT)) { cursor_ += perSec * dt; playing_ = false; }
        cursor_ = std::clamp(cursor_, 0.0, double(std::max(0, avail - 1)));
    }
}

void Visualizer::updateCursor() {
    const float dt = GetFrameTime();
    const double frameDt = rec_->decimation * rec_->baseStep;  // seconds per frame

    if (sim_ && !sim_->finished()) {
        if (playing_) {
            long ticks = (long)std::ceil(playbackSpeed_ * dt / rec_->baseStep);
            ticks = std::clamp<long>(ticks, 1, 500000);
            for (long i = 0; i < ticks && sim_->step(); ++i) {}
        }
        cursor_ = std::max(0, availableFrames() - 1);
    } else if (playing_) {
        cursor_ += playbackSpeed_ * dt / frameDt;
        const int avail = availableFrames();
        if (cursor_ >= avail - 1) { cursor_ = avail - 1; playing_ = false; }
    }
    gFrame = std::clamp((int)cursor_, 0, std::max(0, availableFrames() - 1));
}

void Visualizer::drawScene() {
    const int sw = GetScreenWidth(), sh = GetScreenHeight();
    const float availH = std::max(100, sh - 150);

    const Vec2 mn = traj_->minBound(), mx = traj_->maxBound();
    const float margin = exag_ * vr::kHardBound + 25.0f;
    const float spanX = (mx.x - mn.x) + 2 * margin;
    const float spanY = (mx.y - mn.y) + 2 * margin;
    fitZoom_ = 0.95f * std::min(sw / spanX, availH / spanY);

    const bool haveData = availableFrames() > 0;
    Camera2D cam{};
    cam.rotation = 0.0f;
    cam.zoom = fitZoom_ * userZoom_ * (follow_ ? 5.0f : 1.0f);
    cam.offset = Vector2{sw * 0.5f, availH * 0.5f};
    Vec2 target = trackCenter_;
    if (follow_ && haveData) target = frameActualPos(selected_, gFrame);
    cam.target = rl(target);
    gZoom = cam.zoom;

    BeginMode2D(cam);
    drawTrack();
    if (haveData) {
        drawVehiclePaths();
        drawPrediction();
        drawCars();
    }
    EndMode2D();
}

const Prediction* Visualizer::currentPrediction() {
    if (rec_->frames[selected_].empty()) return nullptr;
    if (sim_ && !sim_->finished())
        return &sim_->prediction(selected_);  // live: the 10 ms cache

    // Replay (or scrubbing a finished run): recompute from the frame's stored
    // physical state + held command. Needs format >= 4 frames.
    if (rec_->loadedVersion < 4) return nullptr;
    if (replayPredVeh_ != selected_ || replayPredFrame_ != gFrame) {
        const Frame& f = rec_->frames[selected_][gFrame];
        double x0[6];
        for (int i = 0; i < 6; ++i) x0[i] = f.phys[i];
        // f.refStep is already the wrapped trajectory index (offset included),
        // so it serves directly as the rollout's input index base.
        replayPred_ = predictHold(x0, f.act, f.refStep, *traj_, 0, PredictParams{});
        replayPredVeh_ = selected_;
        replayPredFrame_ = gFrame;
    }
    return &replayPred_;
}

void Visualizer::drawPrediction() {
    const Prediction* pred = currentPrediction();
    if (!pred || pred->e_y.size() < 2) return;
    const Frame& f = rec_->frames[selected_][gFrame];
    const long stride = std::max<long>(1, pred->strideTicks);

    auto worldAt = [&](size_t k) {
        const long ref = static_cast<long>(f.refStep) + static_cast<long>(k) * stride;
        return traj_->pointAt(ref) + traj_->normalAt(ref) * (pred->e_y[k] * exag_);
    };

    // Dotted line: groups of drawn segments separated by gaps.
    const float w = 2.0f / gZoom;
    constexpr size_t kDash = 3, kGap = 2;
    for (size_t k = 0; k + 1 < pred->e_y.size(); ++k) {
        if (k % (kDash + kGap) >= kDash) continue;
        const Color c = Fade(errorColor(std::fabs(pred->e_y[k])), 0.9f);
        DrawLineEx(rl(worldAt(k)), rl(worldAt(k + 1)), w, c);
    }

    // Marker at the predicted 0.8 m crossing (red ring), if within horizon.
    const long H = PredictParams{}.horizonTicks;
    if (pred->ttvTicks < H) {
        const size_t iv = std::min(pred->e_y.size() - 1,
                                   static_cast<size_t>(pred->ttvTicks / stride));
        const Vec2 p = worldAt(iv);
        DrawCircleLinesV(rl(p), 6.0f / gZoom, Color{255, 60, 50, 255});
        DrawCircleLinesV(rl(p), 7.5f / gZoom, Color{255, 60, 50, 180});
    }
    // Marker at the point of no return (orange diamond).
    if (pred->ttpnrTicks < H) {
        const size_t ip = std::min(pred->e_y.size() - 1,
                                   static_cast<size_t>(pred->ttpnrTicks / stride));
        DrawPoly(rl(worldAt(ip)), 4, 6.0f / gZoom, 45.0f, Color{255, 160, 40, 230});
    }

    // The escape route: the latest-possible successful rescue trajectory,
    // branching off at rescueFromTick (≈ the PNR diamond). Dashed cyan so it
    // reads as "the alternative future the guard is protecting".
    if (!pred->rescue_e_y.empty() && pred->rescueFromTick >= 0) {
        auto rescueWorldAt = [&](size_t k) {
            const long ref = static_cast<long>(f.refStep) + pred->rescueFromTick +
                             static_cast<long>(k) * stride;
            return traj_->pointAt(ref) +
                   traj_->normalAt(ref) * (pred->rescue_e_y[k] * exag_);
        };
        const Color cRescue{90, 220, 220, 220};
        for (size_t k = 0; k + 1 < pred->rescue_e_y.size(); ++k) {
            if (k % 5 >= 3) continue;  // 3-on / 2-off dash, distinct cadence
            DrawLineEx(rl(rescueWorldAt(k)), rl(rescueWorldAt(k + 1)),
                       1.6f / gZoom, cRescue);
        }
    }
}

void Visualizer::drawTrack() {
    const float soft = exag_ * (float)vr::kSoftBound;
    const float hard = exag_ * (float)vr::kHardBound;
    const float wHard = 2.0f / gZoom, wSoft = 1.5f / gZoom, wMid = 1.2f / gZoom;
    const Color cHard = Color{210, 70, 60, 150};
    const Color cSoft = Color{220, 195, 70, 120};
    const Color cMid  = Color{200, 200, 215, 110};

    for (size_t i = 0; i + 1 < trackPos_.size(); ++i) {
        const Vec2 p0 = trackPos_[i], p1 = trackPos_[i + 1];
        const Vec2 n0 = trackNrm_[i], n1 = trackNrm_[i + 1];
        DrawLineEx(rl(p0 + n0 * hard), rl(p1 + n1 * hard), wHard, cHard);
        DrawLineEx(rl(p0 - n0 * hard), rl(p1 - n1 * hard), wHard, cHard);
        DrawLineEx(rl(p0 + n0 * soft), rl(p1 + n1 * soft), wSoft, cSoft);
        DrawLineEx(rl(p0 - n0 * soft), rl(p1 - n1 * soft), wSoft, cSoft);
        DrawLineEx(rl(p0), rl(p1), wMid, cMid);  // expected (reference) path
    }
}

void Visualizer::drawVehiclePaths() {
    const int nVeh = rec_->nVehicles;
    for (int v = 0; v < nVeh; ++v) {
        if (rec_->frames[v].empty()) continue;
        const bool sel = (v == selected_);
        const int trail = sel ? 1500 : 400;             // frames (~15 s / 4 s at 10 ms)
        const int start = std::max(0, gFrame - trail);
        const float w = (sel ? 2.2f : 1.2f) / gZoom;
        for (int i = start; i < gFrame; ++i) {
            const float a = std::fabs(rec_->frames[v][i].e_y_real);
            Color c = errorColor(a);
            if (!sel) c = Fade(c, 0.35f);
            DrawLineEx(rl(frameActualPos(v, i)), rl(frameActualPos(v, i + 1)), w, c);
        }
        // Persistent hard-breach markers for the selected vehicle.
        if (sel) {
            for (int i = 0; i <= gFrame; ++i)
                if (rec_->frames[v][i].flags & Frame::kHard)
                    DrawCircleV(rl(frameActualPos(v, i)), 3.0f / gZoom, Color{255, 60, 50, 230});
        }
    }
}

void Visualizer::drawCars() {
    const int nVeh = rec_->nVehicles;
    for (int v = 0; v < nVeh; ++v) {
        if (rec_->frames[v].empty()) continue;
        const bool sel = (v == selected_);
        const Vec2 p = frameActualPos(v, gFrame);
        const Vec2 nrm = traj_->normalAt(rec_->frames[v][gFrame].refStep);
        const Vec2 fwd = normalized(Vec2{nrm.y, -nrm.x});  // tangent
        const Color col = sel ? Color{90, 200, 255, 255} : Color{120, 150, 230, 200};

        const float rPx = sel ? 7.0f : 5.0f;
        DrawCircleV(rl(p), rPx / gZoom, col);
        DrawLineEx(rl(p), rl(p + fwd * (rPx * 2.2f / gZoom)), 3.0f / gZoom, col);
        if (sel)
            DrawCircleLinesV(rl(p), (rPx + 4.0f) / gZoom, Color{255, 255, 255, 220});
    }
}

void Visualizer::drawHud() {
    const int sw = GetScreenWidth();
    const int avail = availableFrames();
    DrawRectangle(0, 0, 360, 190, Color{0, 0, 0, 150});

    char line[160];
    int y = 8;
    auto put = [&](const char* s, Color c = RAYWHITE) { DrawText(s, 10, y, 18, c); y += 22; };

    std::snprintf(line, sizeof line, "%s   cores:%d  vehicles:%d  %s",
                  rec_->schedulerName.c_str(), rec_->nCores, rec_->nVehicles,
                  profileName((Profile)rec_->profile));
    put(line, Color{180, 220, 255, 255});

    const double total = rec_->duration();
    const bool live = sim_ && !sim_->finished();
    if (avail > 0) {
        const Frame& f = rec_->frames[selected_][gFrame];
        std::snprintf(line, sizeof line, "t=%.2f/%.1fs  x%.1f  %s%s", f.t, total,
                      playbackSpeed_, playing_ ? "PLAY" : "PAUSE", live ? " LIVE" : "");
        put(line);
        std::snprintf(line, sizeof line, "vehicle %d   v=%.1f m/s", selected_, f.vel);
        put(line, Color{90, 200, 255, 255});
        const float a = std::fabs(f.e_y_real);
        std::snprintf(line, sizeof line, "e_y=%+.3f m (est %+.3f)", f.e_y_real, f.e_y_est);
        put(line, errorColor(a));
        std::snprintf(line, sizeof line, "rolling=%.4f  avg=%.4f", f.rolling_real, f.average_real);
        put(line);
        const char* state = (f.flags & Frame::kHard) ? "HARD BREACH (>0.8m)"
                          : (f.flags & Frame::kSoft) ? "over soft bound (>0.2m)"
                                                     : "within bounds";
        const Color sc = (f.flags & Frame::kHard) ? Color{255, 60, 50, 255}
                       : (f.flags & Frame::kSoft) ? Color{235, 200, 60, 255}
                                                  : Color{60, 200, 90, 255};
        std::snprintf(line, sizeof line, "%s%s", state, (f.flags & Frame::kCritical) ? "  [curve]" : "");
        put(line, sc);
        // Held-command predictions (>= horizon shown as ">=500"; -1 = no data).
        if (f.ttpnr_ms >= 0.0f) {
            const float H = 500.0f;
            char ttvBuf[16], pnrBuf[16];
            if (f.ttv_ms >= H) std::snprintf(ttvBuf, sizeof ttvBuf, ">=%.0f", H);
            else               std::snprintf(ttvBuf, sizeof ttvBuf, "%.0f", f.ttv_ms);
            if (f.ttpnr_ms >= H) std::snprintf(pnrBuf, sizeof pnrBuf, ">=%.0f", H);
            else                 std::snprintf(pnrBuf, sizeof pnrBuf, "%.0f", f.ttpnr_ms);
            if (f.ttpnr_ms <= 0.0f) {
                put("pred: PAST POINT OF NO RETURN", Color{255, 60, 50, 255});
            } else {
                std::snprintf(line, sizeof line, "pred: hits 0.8m in %s ms   PNR in %s ms",
                              ttvBuf, pnrBuf);
                const Color pc = f.ttpnr_ms < 100.0f ? Color{255, 160, 40, 255}
                                                     : Color{200, 200, 215, 255};
                put(line, pc);
            }
            // Rescue clearance (live or recomputed in replay).
            const Prediction* pr = currentPrediction();
            if (pr && pr->rescueClearanceM < 1e8) {
                std::snprintf(line, sizeof line, "rescue margin %.2f m%s",
                              pr->rescueClearanceM,
                              pr->rescueClearanceM < 0.0 ? "  (rescue fails)" : "");
                put(line, pr->rescueClearanceM < 0.1 ? Color{255, 160, 40, 255}
                                                     : Color{160, 220, 220, 255});
            }
        }
    } else {
        put("warming up...");
    }

    if (showHelp_) {
        const char* help =
            "Space play/pause   [ ] vehicle   F follow   wheel zoom   "
            "Up/Down speed   ,/. exaggerate   Left/Right scrub   H help";
        DrawRectangle(0, GetScreenHeight() - 24, sw, 24, Color{0, 0, 0, 150});
        DrawText(help, 10, GetScreenHeight() - 21, 16, Color{200, 200, 200, 255});
    }
    std::snprintf(line, sizeof line, "exag x%.0f", exag_);
    DrawText(line, sw - 110, 8, 18, Color{200, 200, 200, 255});
}

void Visualizer::drawErrorStrip() {
    const int avail = availableFrames();
    if (avail <= 0) return;
    const int sw = GetScreenWidth(), sh = GetScreenHeight();
    const Rectangle r{0, (float)sh - 26 - 96, (float)sw, 96};
    DrawRectangle((int)r.x, (int)r.y, (int)r.width, (int)r.height, Color{0, 0, 0, 140});

    const float maxE = 1.0f;  // m, vertical scale (clamped)
    const float midY = r.y + r.height * 0.5f;
    auto eToY = [&](float e) { return midY - std::clamp(e / maxE, -1.0f, 1.0f) * (r.height * 0.5f - 4); };

    // Threshold guide lines.
    for (float b : {(float)vr::kHardBound, (float)vr::kSoftBound}) {
        const Color c = (b == (float)vr::kHardBound) ? Color{210, 70, 60, 200}
                                                     : Color{220, 195, 70, 200};
        DrawLine(0, (int)eToY(b), sw, (int)eToY(b), c);
        DrawLine(0, (int)eToY(-b), sw, (int)eToY(-b), c);
    }
    DrawLine(0, (int)midY, sw, (int)midY, Color{120, 120, 130, 160});

    const auto& fr = rec_->frames[selected_];
    const int total = (int)fr.size();
    int prevx = -1;
    float prevy = midY;
    for (int x = 0; x < sw; ++x) {
        const int idx = total > 1 ? (int)((float)x / sw * (total - 1)) : 0;
        const float e = fr[idx].e_y_real;
        const float yy = eToY(e);
        if (prevx >= 0) DrawLineEx(Vector2{(float)prevx, prevy}, Vector2{(float)x, yy}, 1.5f,
                                   errorColor(std::fabs(e)));
        prevx = x;
        prevy = yy;
    }
    // Playback cursor.
    const float cx = total > 1 ? (float)gFrame / (total - 1) * sw : 0;
    DrawLine((int)cx, (int)r.y, (int)cx, (int)(r.y + r.height), RAYWHITE);
    DrawText("lateral error e_y", 10, (int)r.y + 4, 14, Color{200, 200, 200, 200});
}

void Visualizer::drawTimeline() {
    const int avail = availableFrames();
    if (avail <= 0) return;
    const int sw = GetScreenWidth(), sh = GetScreenHeight();
    const Rectangle r{0, (float)sh - 24, (float)sw, 24};
    DrawRectangle((int)r.x, (int)r.y, (int)r.width, (int)r.height, Color{0, 0, 0, 170});

    const auto& fr = rec_->frames[selected_];
    const int total = (int)fr.size();
    // Hard-breach ticks across the whole run.
    for (int i = 0; i < total; ++i) {
        if (fr[i].flags & Frame::kHard) {
            const int x = total > 1 ? (int)((float)i / (total - 1) * sw) : 0;
            DrawLine(x, (int)r.y, x, (int)(r.y + r.height), Color{255, 60, 50, 180});
        }
    }
    const float cx = total > 1 ? (float)gFrame / (total - 1) * sw : 0;
    DrawRectangle(0, (int)r.y, (int)cx, (int)r.height, Color{90, 200, 255, 50});
    DrawLine((int)cx, (int)r.y, (int)cx, (int)(r.y + r.height), RAYWHITE);

    // Click/drag to scrub when not advancing a live run.
    const bool scrubbable = !sim_ || sim_->finished();
    if (scrubbable && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const Vector2 m = GetMousePosition();
        if (m.y >= r.y) {
            cursor_ = std::clamp((double)m.x / sw, 0.0, 1.0) * std::max(0, avail - 1);
            playing_ = false;
        }
    }
}

}  // namespace cps
