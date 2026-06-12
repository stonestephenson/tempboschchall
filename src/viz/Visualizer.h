// Visualizer.h — raylib top-down visualizer for a run. Shows the track
// (reference centerline + soft/hard lateral bounds), each vehicle, the expected
// vs. actual driven path (offset by the lateral error, with a visual
// exaggeration), and where the car breaches the bounds. Works either as a
// replay of a recorded run or driving a Simulation live.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/Vec2.h"
#include "sim/Predictor.h"
#include "sim/Recording.h"
#include "trace/Trajectory.h"

namespace cps {

class Simulation;

struct VizConfig {
    int   screenW      = 1280;
    int   screenH      = 820;
    float exaggeration = 25.0f;  // multiplies e_y and the drawn bounds
    int   targetFps    = 60;
    // If set, capture a PNG after `screenshotAtFrame` frames and exit (for
    // verification / sharing stills). Empty disables.
    std::string screenshotPath;
    int         screenshotAtFrame = 180;
    int         initialSelected   = 0;   // vehicle selected at startup
    double      initialSpeed      = 4.0; // playback speed (driving-seconds per wall-second)
};

class Visualizer {
public:
    Visualizer(std::shared_ptr<Trajectory> traj, const VizConfig& cfg = {});

    // Replay a finished recording.
    void replay(const RunRecording& rec);
    // Drive `sim` (already constructed) and render as it runs.
    void live(Simulation& sim);

private:
    void loop();
    void handleInput();
    void updateCursor();
    void drawScene();
    void drawTrack();
    void drawVehiclePaths();
    // Dotted held-command prediction for the selected vehicle, with markers at
    // the predicted 0.8 m crossing and the point of no return (PREDICTOR.md).
    void drawPrediction();
    // The prediction to draw for the current (selected_, gFrame), or nullptr.
    // Live: the simulation's 10 ms cache. Replay / finished run: recomputed
    // from the frame's stored state (recording format >= 4), cached per frame.
    const Prediction* currentPrediction();
    void drawCars();
    void drawHud();
    void drawErrorStrip();
    void drawTimeline();

    int  availableFrames() const;
    // World position / normal for a recorded frame.
    Vec2 framePos(int vehicle, int frameIdx) const;
    Vec2 frameActualPos(int vehicle, int frameIdx) const;

    std::shared_ptr<Trajectory> traj_;
    VizConfig                   cfg_;

    const RunRecording* rec_ = nullptr;   // active recording (replay or sim's)
    Simulation*         sim_ = nullptr;   // non-null in live mode

    // Downsampled track geometry: centerline point + unit normal.
    std::vector<Vec2> trackPos_;
    std::vector<Vec2> trackNrm_;

    // View / playback state.
    double cursor_       = 0.0;    // fractional frame index
    bool   playing_      = true;
    double playbackSpeed_= 4.0;    // driving-seconds per wall-second (cfg.initialSpeed)
    int    selected_     = 0;
    bool   follow_       = false;
    float  exag_         = 25.0f;
    float  userZoom_     = 1.0f;   // mouse-wheel multiplier on the fit zoom
    bool   showHelp_     = true;

    float  fitZoom_      = 1.0f;
    Vec2   trackCenter_{};

    // Replay-mode prediction cache (recomputed when the cursor or selection moves).
    Prediction replayPred_;
    int        replayPredVeh_   = -1;
    int        replayPredFrame_ = -1;
};

}  // namespace cps
