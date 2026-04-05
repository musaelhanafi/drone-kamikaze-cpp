#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <tuple>

// ── Constants ─────────────────────────────────────────────────────────────────
inline constexpr float  PINK_H_LO        = 130.f;
inline constexpr float  PINK_H_HI        = 173.f;
inline constexpr float  PINK_S_LO        = 40.f;
inline constexpr float  PINK_V_LO        = 80.f;
inline constexpr float  PINK_SV_HI       = 233.f;

inline constexpr double MIN_BLOB_AREA    = 20.0;
inline constexpr double CENTER_THRESHOLD = 0.1;
inline constexpr double MIN_EXTENT       = 0.45;
inline constexpr double MIN_SOLIDITY     = 0.60;
inline constexpr int    MIN_DIM          = 4;
inline constexpr double MAX_ASPECT       = 6.0;

inline constexpr double GAUSS_SIGMA      = 3.0;

// Kalman tuning
inline constexpr double KF_Q_POS  = 2.0;
inline constexpr double KF_Q_VEL  = 80.0;
inline constexpr double KF_R      = 30.0;
inline constexpr int    KF_MISS_MAX = 5;

inline constexpr double EMA_ALPHA = 0.3;

inline constexpr const char* CAL_HISTOGRAM_FILE = "color_histogram.txt";

// ── Free helpers ─────────────────────────────────────────────────────────────

// Return bounding rect of best blob in mask, or empty if none found.
// prefer_pt: optional point to bias selection toward (by proximity).
std::optional<cv::Rect> nearestBlobRect(
    const cv::Mat& mask,
    bool box_filter = true,
    cv::Point2f prefer_pt = {-1.f, -1.f},
    bool use_prefer = false);

// Fit a circular-mean Gaussian to a 180-bin hue histogram.
// Returns {mean_hue, std_hue}.
std::pair<double, double> fitGaussian(const cv::Mat& hist);

// Build a confidence histogram: zero bins outside mean ± GAUSS_SIGMA*std.
cv::Mat confidenceHist(const cv::Mat& hist, double mean, double std);

// 1-D constant-velocity Kalman update step.
// State: [pos, vel]; observation: pos only.
// Returns updated {x0, x1, P00, P01, P10, P11}.
struct KF1D { double x0, x1, P00, P01, P10, P11; };
KF1D kf1dUpdate(KF1D s, double meas, double dt);
KF1D kf1dPredict(KF1D s, double dt);

// ── Seeker ────────────────────────────────────────────────────────────────────

class Seeker {
public:
    explicit Seeker(
        int         source_index   = 0,
        std::string source_file    = "",     // if non-empty, open file instead of index
        std::string window_name    = "Seeker",
        int         capture_width  = 0,      // 0 = don't request
        int         capture_height = 0,
        cv::Rect    crop           = {},     // empty = no crop
        bool        use_crop       = false,
        std::string histogram_file = CAL_HISTOGRAM_FILE,
        bool        show_histogram = false,
        bool        show_mask      = false,
        std::string mask_algo      = "all",  // "gaussian"|"adaptive"|"inrange"|"all"
        bool        use_camshift   = true,
        bool        box_filter     = true
    );

    void open();
    void close();

    // Read latest frame from background thread (thread-safe).
    // Returns false if no frame yet or end-of-stream.
    bool readFrame(cv::Mat& out);

    // Run one tracking step on *frame*.
    // Writes annotated frame to *frame* in-place.
    // Returns {cx, cy} or {-1, -1} if not locked.
    std::pair<int,int> track(cv::Mat& frame);

    // Compute normalised error [-1,1] from pixel centroid.
    // Returns {0,0} when cx/cy are -1.
    std::pair<double,double> errorXY(int cx, int cy, int frame_w, int frame_h) const;

    // Reset tracker state (lose lock)
    void resetTracker();

    const std::string& windowName() const { return _window_name; }

public:
    // Band is public so the free function applyBand() in Seeker.cpp can use it
    struct Band {
        cv::Scalar lo_a, hi_a;
        cv::Scalar lo_b, hi_b;
        enum Mode { SINGLE, WRAP_LO, WRAP_HI } mode = SINGLE;
    };

private:
    // Detection sub-masks (calibrated path)
    cv::Mat _maskGaussian(const cv::Mat& hsv, const cv::Mat& h_blur);
    cv::Mat _maskAdaptive(const cv::Mat& hsv, const cv::Mat& h_blur);
    cv::Mat _maskInrange(const cv::Mat& hsv);

    // Combined detection mask.  locked=true → fast inRange only path.
    cv::Mat _detectionMask(const cv::Mat& hsv, bool locked);

    // Pink mask fallback (no calibration histogram)
    static cv::Mat _pinkMask(const cv::Mat& hsv);

    // Precompute inRange bounds from gaussian parameters
    void _precomputeBands();

    // Optionally show/update the calibration histogram window
    void _updateHistogramWindow();

    // Draw center-bracket crosshair
    static void _drawCenterCross(cv::Mat& frame, int w, int h);

    // Background capture thread
    void _captureLoop();

    // ── Parameters ───────────────────────────────────────────────────────────
    int         _source_index;
    std::string _source_file;
    std::string _window_name;
    int         _capture_width;
    int         _capture_height;
    cv::Rect    _crop;
    bool        _use_crop;
    bool        _show_histogram;
    bool        _show_mask;
    std::string _mask_algo;
    bool        _use_camshift;
    bool        _box_filter;

    // ── Camera ───────────────────────────────────────────────────────────────
    cv::VideoCapture _cap;
    std::thread      _cap_thread;
    std::mutex       _cap_mtx;
    std::atomic<bool>_cap_stop{false};
    cv::Mat          _cap_frame;
    bool             _cap_ok    = false;
    bool             _res_logged= false;

    // ── Calibration histogram ─────────────────────────────────────────────────
    cv::Mat  _cal_hist;             // 180×1 float32, or empty if not loaded
    cv::Mat  _conf_hist;            // windowed confidence histogram
    double   _gauss_mean = -1.0;
    double   _gauss_std  = -1.0;
    cv::Mat  _hue_gate_lut;         // uint8 LUT: 255 inside σ band, 0 outside
    bool     _has_cal = false;

    // Precomputed inRange band bounds (for _maskInrange / _maskGaussian)
    Band _band_core;
    Band _band_outer;

    // Outer half-weight LUT: 255→128, 0→0
    cv::Mat _outer_lut;

    // Morphological kernels
    cv::Mat _kern3;

    // H-channel copy buffer for Gaussian back-projection
    cv::Mat _h_buf;

    // Persistent HSV buffer
    cv::Mat _hsv_buf;

    // ── Tracking state ────────────────────────────────────────────────────────
    cv::Rect     _track_win;          // CamShift window
    int          _detect_count = 0;
    double       _win_w_ema    = 0.0;
    double       _win_h_ema    = 0.0;
    cv::TermCriteria _term_crit;

    // Kalman filter state (independent x/y)
    KF1D         _kf_x{};
    KF1D         _kf_y{};
    bool         _kf_initialized = false;
    double       _kf_last_t      = 0.0;
    int          _miss_count     = 0;

    // Display windows
    std::string  _hist_window;
    std::string  _mask_window;

    // Render hue histogram as BGR image
    static cv::Mat _renderHistogram(const cv::Mat& hist, int width=360, int height=200);
};
