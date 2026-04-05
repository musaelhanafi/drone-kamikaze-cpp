#include "Seeker.hpp"

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <chrono>

using namespace std::chrono;

// ── Monotonic clock helper ────────────────────────────────────────────────────
static double mono_s()
{
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ── Free helpers ──────────────────────────────────────────────────────────────

std::optional<cv::Rect> nearestBlobRect(
    const cv::Mat& mask,
    bool box_filter,
    cv::Point2f prefer_pt,
    bool use_prefer)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return std::nullopt;

    double   best_score = -1.0;
    cv::Rect best_rect;

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < MIN_BLOB_AREA) continue;

        cv::Rect r = cv::boundingRect(c);
        if (r.width == 0 || r.height == 0) continue;

        double score;
        if (box_filter) {
            if (r.width < MIN_DIM || r.height < MIN_DIM) continue;
            double aspect = (double)std::max(r.width, r.height) /
                            (double)std::min(r.width, r.height);
            if (aspect > MAX_ASPECT) continue;
            double extent = area / (r.width * r.height);
            if (extent < MIN_EXTENT) continue;
            std::vector<cv::Point> hull;
            cv::convexHull(c, hull);
            double hull_area = cv::contourArea(hull);
            double solidity  = hull_area > 0.0 ? area / hull_area : 0.0;
            if (solidity < MIN_SOLIDITY) continue;
            score = solidity * extent * area;
        } else {
            score = area;
        }

        if (use_prefer) {
            double bcx  = r.x + r.width  * 0.5;
            double bcy  = r.y + r.height * 0.5;
            double dist = std::hypot(bcx - prefer_pt.x, bcy - prefer_pt.y);
            score /= (1.0 + dist / 100.0);
        }

        if (score > best_score) {
            best_score = score;
            best_rect  = r;
        }
    }
    if (best_score < 0.0) return std::nullopt;
    return best_rect;
}

std::pair<double,double> fitGaussian(const cv::Mat& hist)
{
    const float* h = hist.ptr<float>(0);
    double total = 0.0;
    for (int i = 0; i < 180; i++) total += h[i];
    if (total == 0.0) return {90.0, 30.0};

    const double two_pi = 2.0 * M_PI;
    double cx_sum = 0.0, cy_sum = 0.0;
    for (int i = 0; i < 180; i++) {
        double prob  = h[i] / total;
        double angle = i * two_pi / 180.0;
        cx_sum += std::cos(angle) * prob;
        cy_sum += std::sin(angle) * prob;
    }
    double mean_rad = std::atan2(cy_sum, cx_sum);
    if (mean_rad < 0.0) mean_rad += two_pi;
    double mean_hue = mean_rad * 180.0 / two_pi;

    // Circular std: wrap to [-90, 90]
    double var = 0.0;
    for (int i = 0; i < 180; i++) {
        double prob = h[i] / total;
        double diff = i - mean_hue;
        // wrap to [-90, 90]
        diff = std::fmod(diff + 90.0, 180.0) - 90.0;
        var += diff * diff * prob;
    }
    return {mean_hue, std::sqrt(std::max(var, 1.0))};
}

cv::Mat confidenceHist(const cv::Mat& hist, double mean, double std)
{
    cv::Mat conf = hist.clone();
    float* p = conf.ptr<float>(0);
    for (int i = 0; i < 180; i++) {
        double diff = std::abs(i - mean);
        diff = std::min(diff, 180.0 - diff);
        if (diff >= GAUSS_SIGMA * std) p[i] = 0.0f;
    }
    return conf;
}

KF1D kf1dUpdate(KF1D s, double meas, double dt)
{
    // Predict
    double px0  = s.x0 + s.x1 * dt;
    double px1  = s.x1;
    double pp00 = s.P00 + dt * (s.P10 + s.P01) + dt * dt * s.P11 + KF_Q_POS;
    double pp01 = s.P01 + dt * s.P11;
    double pp10 = s.P10 + dt * s.P11;
    double pp11 = s.P11 + KF_Q_VEL;
    // Update
    double S_inv = 1.0 / (pp00 + KF_R);
    double K0    = pp00 * S_inv;
    double K1    = pp10 * S_inv;
    double innov = meas - px0;
    KF1D out;
    out.x0  = px0 + K0 * innov;
    out.x1  = px1 + K1 * innov;
    out.P00 = (1.0 - K0) * pp00;
    out.P01 = (1.0 - K0) * pp01;
    out.P10 = pp10 - K1 * pp00;
    out.P11 = pp11 - K1 * pp01;
    return out;
}

KF1D kf1dPredict(KF1D s, double dt)
{
    KF1D out;
    out.x0  = s.x0 + s.x1 * dt;
    out.x1  = s.x1;
    out.P00 = s.P00 + dt * (s.P10 + s.P01) + dt * dt * s.P11 + KF_Q_POS;
    out.P01 = s.P01 + dt * s.P11;
    out.P10 = s.P10 + dt * s.P11;
    out.P11 = s.P11 + KF_Q_VEL;
    return out;
}

// ── Seeker ctor ───────────────────────────────────────────────────────────────

Seeker::Seeker(
    int         source_index,
    std::string source_file,
    std::string window_name,
    int         capture_width,
    int         capture_height,
    cv::Rect    crop,
    bool        use_crop,
    std::string histogram_file,
    bool        show_histogram,
    bool        show_mask,
    std::string mask_algo,
    bool        use_camshift,
    bool        box_filter)
    : _source_index   (source_index)
    , _source_file    (source_file)
    , _window_name    (window_name)
    , _capture_width  (capture_width)
    , _capture_height (capture_height)
    , _crop           (crop)
    , _use_crop       (use_crop)
    , _show_histogram (show_histogram)
    , _show_mask      (show_mask)
    , _mask_algo      (mask_algo)
    , _use_camshift   (use_camshift)
    , _box_filter     (box_filter)
    , _term_crit      (cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.5)
{
    // Load calibration histogram
    std::ifstream ifs(histogram_file);
    if (ifs.is_open()) {
        std::vector<float> vals;
        float v;
        while (ifs >> v) vals.push_back(v);
        if (vals.size() == 180) {
            _cal_hist = cv::Mat(180, 1, CV_32F, vals.data()).clone();
            auto [mean, std] = fitGaussian(_cal_hist);
            _gauss_mean = mean;
            _gauss_std  = std;
            _conf_hist  = confidenceHist(_cal_hist, mean, std);
            int kept = cv::countNonZero(_conf_hist);
            printf("[Seeker] Cal hist loaded: mean=%.1f std=%.1f bins=%d/180\n",
                   mean, std, kept);
            _has_cal = true;
        } else {
            printf("[Seeker] Bad histogram file (expected 180 values)\n");
        }
    } else {
        printf("[Seeker] No histogram file '%s' — using HSV ranges\n",
               histogram_file.c_str());
    }

    // Morphological kernels
    _kern3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    // Hue gate LUT and inRange precomputed bands
    if (_has_cal) {
        // Boolean hue gate LUT: hue → 0 or 255
        _hue_gate_lut = cv::Mat(1, 256, CV_8U, cv::Scalar(0));
        double sigma_thresh = GAUSS_SIGMA * _gauss_std;
        for (int i = 0; i < 180; i++) {
            double d = std::abs(i - _gauss_mean);
            d = std::min(d, 180.0 - d);
            _hue_gate_lut.at<uchar>(0, i) = (d < sigma_thresh) ? 255 : 0;
        }
        _precomputeBands();
    }

    // Outer half-weight LUT: 255 → 128, 0 → 0
    _outer_lut = cv::Mat(1, 256, CV_8U, cv::Scalar(0));
    _outer_lut.at<uchar>(0, 255) = 128;

#ifdef DRONE_USE_CUDA
    {
        int n = cv::cuda::getCudaEnabledDeviceCount();
        _use_gpu = (n > 0);
        if (_use_gpu) {
            printf("[Seeker] CUDA enabled (%d device(s)) — GPU acceleration active\n", n);
            _gpu_outer_lut_d.upload(_outer_lut);
            _gpu_morph_close = cv::cuda::createMorphologyFilter(
                cv::MORPH_CLOSE, CV_8UC1, _kern3);
            _gpu_gauss_bp = cv::cuda::createGaussianFilter(
                CV_8UC1, CV_8UC1, cv::Size(3, 3), 0);
        }
    }
#endif
}

// ── open / close ──────────────────────────────────────────────────────────────

void Seeker::open()
{
    if (!_source_file.empty())
        _cap.open(_source_file);
    else
        _cap.open(_source_index);

    if (!_cap.isOpened())
        throw std::runtime_error("[Seeker] Cannot open source");

    if (_capture_width  > 0) _cap.set(cv::CAP_PROP_FRAME_WIDTH,  _capture_width);
    if (_capture_height > 0) _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _capture_height);

    int aw = (int)_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int ah = (int)_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("[Seeker] Opened source  capture=%dx%d\n", aw, ah);

    _cap_stop.store(false);
    _cap_thread = std::thread(&Seeker::_captureLoop, this);

    cv::namedWindow(_window_name, cv::WINDOW_NORMAL);

    if (_show_histogram && _has_cal) {
        _hist_window = _window_name + " - Histogram";
        cv::namedWindow(_hist_window, cv::WINDOW_NORMAL);
        cv::resizeWindow(_hist_window, 360, 230);
    }
    if (_show_mask) {
        _mask_window = _window_name + " - Mask";
        cv::namedWindow(_mask_window, cv::WINDOW_NORMAL);
    }
}

void Seeker::close()
{
    _cap_stop.store(true);
    if (_cap_thread.joinable()) _cap_thread.join();
    _cap.release();
    cv::destroyWindow(_window_name);
    if (!_hist_window.empty()) cv::destroyWindow(_hist_window);
    if (!_mask_window.empty()) cv::destroyWindow(_mask_window);
}

// ── Background capture thread ─────────────────────────────────────────────────

void Seeker::_captureLoop()
{
    while (!_cap_stop.load()) {
        cv::Mat frame;
        bool ok = _cap.read(frame);
        std::lock_guard<std::mutex> lk(_cap_mtx);
        _cap_ok    = ok;
        _cap_frame = frame;
    }
}

bool Seeker::readFrame(cv::Mat& out)
{
    std::lock_guard<std::mutex> lk(_cap_mtx);
    if (_cap_frame.empty()) return false;
    if (!_use_crop) {
        out = _cap_frame.clone();
    } else {
        int fx = _cap_frame.cols, fy = _cap_frame.rows;
        cv::Rect roi = _crop;
        if (roi.width  <= 0) roi.width  = fx - roi.x;
        if (roi.height <= 0) roi.height = fy - roi.y;
        out = _cap_frame(roi).clone();
    }
    if (_cap_ok && !_res_logged) {
        printf("[Seeker] Resolution: %dx%d\n", out.cols, out.rows);
        _res_logged = true;
    }
    return _cap_ok;
}

// ── Precompute inRange bands ──────────────────────────────────────────────────

void Seeker::_precomputeBands()
{
    auto buildBand = [](double mean, double hw, Band& b) {
        double lo = mean - hw;
        double hi = mean + hw;
        if (lo < 0) {
            b.mode = Band::WRAP_LO;
            b.lo_a = cv::Scalar(std::max(0.0, lo + 180.0), 40, 40);
            b.hi_a = cv::Scalar(179, 255, 255);
            b.lo_b = cv::Scalar(0, 40, 40);
            b.hi_b = cv::Scalar(std::min(179.0, hi), 255, 255);
        } else if (hi > 179) {
            b.mode = Band::WRAP_HI;
            b.lo_a = cv::Scalar(std::max(0.0, lo), 40, 40);
            b.hi_a = cv::Scalar(179, 255, 255);
            b.lo_b = cv::Scalar(0, 40, 40);
            b.hi_b = cv::Scalar(std::min(179.0, hi - 180.0), 255, 255);
        } else {
            b.mode = Band::SINGLE;
            b.lo_a = cv::Scalar(std::max(0.0, lo), 40, 40);
            b.hi_a = cv::Scalar(std::min(179.0, hi), 255, 255);
        }
    };
    buildBand(_gauss_mean, _gauss_std * 1.0,    _band_core);   // ±1σ
    buildBand(_gauss_mean, _gauss_std * GAUSS_SIGMA, _band_outer);  // ±3σ
}

static cv::Mat applyBand(const cv::Mat& hsv, const Seeker::Band& b)
{
    cv::Mat m_a, m_b;
    if (b.mode == Seeker::Band::SINGLE) {
        cv::inRange(hsv, b.lo_a, b.hi_a, m_a);
        return m_a;
    }
    cv::inRange(hsv, b.lo_a, b.hi_a, m_a);
    cv::inRange(hsv, b.lo_b, b.hi_b, m_b);
    cv::bitwise_or(m_a, m_b, m_a);
    return m_a;
}

// ── Detection sub-masks ───────────────────────────────────────────────────────

cv::Mat Seeker::_maskGaussian(const cv::Mat& hsv, const cv::Mat& h_blur)
{
    // Swap H channel with blurred H temporarily, back-project, restore
    if (_h_buf.empty() || _h_buf.size() != hsv.size())
        _h_buf = hsv.rowRange(0, hsv.rows).col(0).clone();  // unused placeholder

    cv::Mat planes[3];
    cv::split(hsv, planes);
    cv::Mat h_orig = planes[0].clone();
    h_blur.copyTo(planes[0]);
    cv::Mat hsv_blur;
    cv::merge(planes, 3, hsv_blur);

    cv::Mat bp;
    {
        int   ch[]  = {0};
        float rng[] = {0, 180};
        const float* ranges[] = {rng};
        cv::calcBackProject(&hsv_blur, 1, ch, _conf_hist, bp, ranges);
    }

    // Restore original H
    // (planes[0] is already h_orig from the split, hsv_blur used blurred copy)

    cv::Mat thresh;
    cv::threshold(bp, thresh, 0, 255, cv::THRESH_BINARY);

    cv::Mat sv_ok = applyBand(hsv, _band_outer);   // reuse outer band as S/V gate
    cv::Mat result;
    cv::bitwise_and(thresh, sv_ok, result);
    return result;
}

cv::Mat Seeker::_maskAdaptive(const cv::Mat& /*hsv*/, const cv::Mat& h_blur)
{
    cv::Mat adapt;
    cv::adaptiveThreshold(h_blur, adapt, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY,
                          11, 3);

    // Apply hue gate LUT per-pixel
    cv::Mat hue_gate;
    cv::LUT(h_blur, _hue_gate_lut, hue_gate);

    cv::Mat result;
    cv::bitwise_and(adapt, hue_gate, result);
    return result;
}

cv::Mat Seeker::_maskInrange(const cv::Mat& hsv)
{
    cv::Mat core  = applyBand(hsv, _band_core);
    cv::Mat outer = applyBand(hsv, _band_outer);
    // outer-only pixels get half weight (128), core pixels keep 255
    cv::subtract(outer, core, outer);
    cv::Mat half;
    cv::LUT(outer, _outer_lut, half);
    cv::Mat result;
    cv::bitwise_or(core, half, result);
    return result;
}

cv::Mat Seeker::_pinkMask(const cv::Mat& hsv)
{
    cv::Mat mask;
    cv::inRange(hsv,
                cv::Scalar(PINK_H_LO, PINK_S_LO, PINK_V_LO),
                cv::Scalar(PINK_H_HI, PINK_SV_HI, PINK_SV_HI),
                mask);
    cv::Mat kern = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,   kern);
    cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kern);
    return mask;
}

#ifdef DRONE_USE_CUDA
// GPU version of _maskInrange + morphologyEx CLOSE.
// Reads _gpu_hsv_d (must be uploaded before calling).
cv::Mat Seeker::_maskInrangeGpu()
{
    // Apply an inRange band on the GPU, writing to `out`.
    auto gpu_band = [&](const Band& b, cv::cuda::GpuMat& out) {
        if (b.mode == Band::SINGLE) {
            cv::cuda::inRange(_gpu_hsv_d, b.lo_a, b.hi_a, out);
        } else {
            cv::cuda::inRange(_gpu_hsv_d, b.lo_a, b.hi_a, _gpu_tmp_a);
            cv::cuda::inRange(_gpu_hsv_d, b.lo_b, b.hi_b, _gpu_tmp_b);
            cv::cuda::bitwise_or(_gpu_tmp_a, _gpu_tmp_b, out);
        }
    };

    cv::cuda::GpuMat gpu_core, gpu_outer;
    gpu_band(_band_core,  gpu_core);
    gpu_band(_band_outer, gpu_outer);

    // outer-only pixels → scale to 128 via LUT, then OR with core (255)
    cv::cuda::subtract(gpu_outer, gpu_core, _gpu_tmp_a);
    cv::cuda::LUT(_gpu_tmp_a, _gpu_outer_lut_d, _gpu_tmp_b);
    cv::cuda::bitwise_or(gpu_core, _gpu_tmp_b, _gpu_tmp_a);

    _gpu_morph_close->apply(_gpu_tmp_a, _gpu_tmp_a);

    cv::Mat result;
    _gpu_tmp_a.download(result);
    return result;
}
#endif

cv::Mat Seeker::_detectionMask(const cv::Mat& hsv, bool locked)
{
    if (!_has_cal) return _pinkMask(hsv);

    if (locked) {
#ifdef DRONE_USE_CUDA
        if (_use_gpu) return _maskInrangeGpu();
#endif
        cv::Mat m = _maskInrange(hsv);
        cv::morphologyEx(m, m, cv::MORPH_CLOSE, _kern3);
        return m;
    }

    cv::Mat mask;
    if (_mask_algo == "inrange") {
        mask = _maskInrange(hsv);
    } else {
        cv::Mat h_blur;
        cv::GaussianBlur(hsv, h_blur, cv::Size(5,5), 0);
        // Extract just the H channel for adaptive/gaussian methods
        std::vector<cv::Mat> planes;
        cv::split(h_blur, planes);
        cv::Mat h_ch = planes[0];

        if (_mask_algo == "gaussian") {
            mask = _maskGaussian(hsv, h_ch);
        } else if (_mask_algo == "adaptive") {
            mask = _maskAdaptive(hsv, h_ch);
        } else {  // "all" — 2-of-3 majority vote
            cv::Mat m1 = _maskGaussian(hsv, h_ch);
            cv::Mat m2 = _maskAdaptive(hsv, h_ch);
            cv::Mat m3 = _maskInrange(hsv);
            // Vote: accumulate in uint8, then threshold at > 1
            cv::Mat votes = cv::Mat::zeros(hsv.rows, hsv.cols, CV_8U);
            cv::Mat tmp;
            cv::threshold(m1, tmp, 0, 1, cv::THRESH_BINARY); cv::add(votes, tmp, votes);
            cv::threshold(m2, tmp, 0, 1, cv::THRESH_BINARY); cv::add(votes, tmp, votes);
            cv::threshold(m3, tmp, 0, 1, cv::THRESH_BINARY); cv::add(votes, tmp, votes);
            cv::threshold(votes, mask, 1, 255, cv::THRESH_BINARY);
        }
    }
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, _kern3);
    return mask;
}

// ── HUD helpers ───────────────────────────────────────────────────────────────

void Seeker::_drawCenterCross(cv::Mat& frame, int w, int h)
{
    int cx = w / 2, cy = h / 2;
    int box = 80, arm = 16;
    cv::Scalar color(0, 0, 233);
    int thick = 3;
    int corners[4][4] = {
        {cx-box, cy-box, +1, +1},
        {cx+box, cy-box, -1, +1},
        {cx+box, cy+box, -1, -1},
        {cx-box, cy+box, +1, -1},
    };
    for (auto& c : corners) {
        cv::line(frame, {c[0], c[1]}, {c[0]+arm*c[2], c[1]}, color, thick);
        cv::line(frame, {c[0], c[1]}, {c[0], c[1]+arm*c[3]}, color, thick);
    }
    int cs = 24;
    cv::line(frame, {cx-cs, cy}, {cx+cs, cy}, color, thick);
    cv::line(frame, {cx, cy-cs}, {cx, cy+cs}, color, thick);
}

cv::Mat Seeker::_renderHistogram(const cv::Mat& hist, int width, int height)
{
    cv::Mat canvas(height + 30, width, CV_8UC3, cv::Scalar(0, 0, 0));
    const float* h = hist.ptr<float>(0);
    float max_v = *std::max_element(h, h + 180);
    if (max_v == 0.f) max_v = 1.f;
    double bar_w = (double)width / 180.0;

    for (int i = 0; i < 180; i++) {
        int bar_h = (int)(h[i] / max_v * height);
        uint8_t hsv_arr[3] = {(uint8_t)i, 220, 200};
        cv::Mat hsv_px(1, 1, CV_8UC3, hsv_arr);
        cv::Mat bgr_px;
        cv::cvtColor(hsv_px, bgr_px, cv::COLOR_HSV2BGR);
        cv::Scalar colour(bgr_px.at<cv::Vec3b>(0, 0)[0],
                          bgr_px.at<cv::Vec3b>(0, 0)[1],
                          bgr_px.at<cv::Vec3b>(0, 0)[2]);
        int x0 = (int)(i * bar_w);
        int x1 = std::max(x0 + 1, (int)((i + 1) * bar_w));
        cv::rectangle(canvas, {x0, height - bar_h}, {x1, height}, colour, -1);
    }
    for (int hue = 0; hue <= 180; hue += 30) {
        int x = (int)(hue * bar_w);
        cv::line(canvas, {x, height}, {x, height+3}, {200,200,200}, 1);
        cv::putText(canvas, std::to_string(hue), {std::max(0, x-8), height+20},
                    cv::FONT_HERSHEY_SIMPLEX, 0.33, {200,200,200}, 1);
    }
    return canvas;
}

void Seeker::_updateHistogramWindow()
{
    if (_hist_window.empty() || !_has_cal) return;
    cv::imshow(_hist_window, _renderHistogram(_cal_hist));
}

// ── resetTracker ─────────────────────────────────────────────────────────────

void Seeker::resetTracker()
{
    _track_win     = {};
    _detect_count  = 0;
    _win_w_ema     = 0.0;
    _win_h_ema     = 0.0;
    _kf_initialized= false;
    _miss_count    = 0;
}

// ── error_xy ─────────────────────────────────────────────────────────────────

std::pair<double,double> Seeker::errorXY(int cx, int cy, int fw, int fh) const
{
    if (cx < 0) return {0.0, 0.0};
    double ex =  (cx - fw / 2.0) / (fw / 2.0);
    double ey = -(cy - fh / 2.0) / (fh / 2.0);
    return {ex, ey};
}

// ── track ─────────────────────────────────────────────────────────────────────

std::pair<int,int> Seeker::track(cv::Mat& frame)
{
    int w_frame = frame.cols;
    int h_frame = frame.rows;

    if (_hsv_buf.empty() || _hsv_buf.size() != frame.size())
        _hsv_buf = cv::Mat(frame.size(), CV_8UC3);

#ifdef DRONE_USE_CUDA
    if (_use_gpu) {
        _gpu_src.upload(frame);
        cv::cuda::cvtColor(_gpu_src, _gpu_hsv_d, cv::COLOR_BGR2HSV);
        _gpu_hsv_d.download(_hsv_buf);
    } else
#endif
    cv::cvtColor(frame, _hsv_buf, cv::COLOR_BGR2HSV);
    const cv::Mat& hsv = _hsv_buf;

    // ── Detection-only path (no CamShift) ─────────────────────────────────────
    if (!_use_camshift) {
        cv::Mat mask = _detectionMask(hsv, false);
        if (!_mask_window.empty()) cv::imshow(_mask_window, mask);

        auto rect = nearestBlobRect(mask, _box_filter);
        _drawCenterCross(frame, w_frame, h_frame);
        _updateHistogramWindow();
        if (!rect) return {-1, -1};

        int cx = rect->x + rect->width  / 2;
        int cy = rect->y + rect->height / 2;
        auto [ex, ey] = errorXY(cx, cy, w_frame, h_frame);
        bool centred  = std::abs(ex) < CENTER_THRESHOLD && std::abs(ey) < CENTER_THRESHOLD;
        cv::Scalar box_col = centred ? cv::Scalar(0,233,0) : cv::Scalar(203,192,233);
        cv::rectangle(frame, *rect, box_col, 2);
        cv::line(frame, {0, cy}, {w_frame, cy}, {0,233,233}, 1);
        cv::line(frame, {cx, 0}, {cx, h_frame}, {0,233,233}, 1);
        cv::circle(frame, {cx, cy}, 3, {0,233,233}, -1);
        return {cx, cy};
    }

    bool locked = (_has_cal &&
                   _track_win.area() > 0 &&
                   _detect_count >= 3);

    // ── Detection / re-acquisition ────────────────────────────────────────────
    std::optional<cv::Rect> blob;
    cv::Mat mask_disp;

    if (!locked) {
        if (_track_win.area() > 0 && _detect_count > 0) {
            // Restricted search around last known window
            int pad = std::max({_track_win.width, _track_win.height, 40});
            cv::Rect search(
                std::max(0, _track_win.x - pad),
                std::max(0, _track_win.y - pad),
                0, 0);
            search.width  = std::min(w_frame, _track_win.x + _track_win.width  + pad) - search.x;
            search.height = std::min(h_frame, _track_win.y + _track_win.height + pad) - search.y;

            cv::Mat roi_mask = _detectionMask(hsv(search), false);
            auto b_crop = nearestBlobRect(roi_mask, _box_filter);
            if (b_crop) {
                blob = cv::Rect(b_crop->x + search.x, b_crop->y + search.y,
                                b_crop->width, b_crop->height);
            }
            if (!_mask_window.empty()) {
                mask_disp = cv::Mat::zeros(h_frame, w_frame, CV_8U);
                roi_mask.copyTo(mask_disp(search));
            }
        } else {
            cv::Mat full_mask = _detectionMask(hsv, false);
            blob = nearestBlobRect(full_mask, _box_filter);
            if (!_mask_window.empty()) mask_disp = full_mask;
        }

        if (blob) {
            int pad = std::max(8, (int)(std::max(blob->width, blob->height) * 0.3));
            _track_win = cv::Rect(
                std::max(0, blob->x - pad),
                std::max(0, blob->y - pad),
                std::min(w_frame - std::max(0, blob->x - pad), blob->width  + 2 * pad),
                std::min(h_frame - std::max(0, blob->y - pad), blob->height + 2 * pad));
            _detect_count = std::min(_detect_count + 1, 3);
            if (_detect_count == 1) {
                _win_w_ema = _track_win.width;
                _win_h_ema = _track_win.height;
            }
        } else {
            _detect_count = 0;
            _track_win    = {};
        }
    }

    if (!_mask_window.empty() && !mask_disp.empty())
        cv::imshow(_mask_window, mask_disp);

    if (!_has_cal || _track_win.area() == 0 || _detect_count < 3) {
        _drawCenterCross(frame, w_frame, h_frame);
        _updateHistogramWindow();
        return {-1, -1};
    }

    // ── CamShift step ─────────────────────────────────────────────────────────
    double now_t  = mono_s();
    double kf_dt  = _kf_initialized ? std::min(now_t - _kf_last_t, 0.5) : 0.0;
    _kf_last_t = now_t;

    // Back-project on H channel using confidence histogram
    cv::Mat back_proj;
    {
        int   ch[]  = {0};
        float rng[] = {0, 180};
        const float* ranges[] = {rng};
        cv::calcBackProject(&hsv, 1, ch, _conf_hist, back_proj, ranges);
    }

    // Pre-translate search window by predicted velocity
    if (_kf_initialized && kf_dt > 0.0) {
        int dx = (int)std::round(_kf_x.x1 * kf_dt);
        int dy = (int)std::round(_kf_y.x1 * kf_dt);
        _track_win.x = std::max(0, std::min(_track_win.x + dx, w_frame - _track_win.width));
        _track_win.y = std::max(0, std::min(_track_win.y + dy, h_frame - _track_win.height));
    }

    // Gate back-projection to padded search window
    int pad_x = std::max(_track_win.width  / 2, 20);
    int pad_y = std::max(_track_win.height / 2, 20);
    int bx1 = std::max(0, _track_win.x - pad_x);
    int by1 = std::max(0, _track_win.y - pad_y);
    int bx2 = std::min(w_frame, _track_win.x + _track_win.width  + pad_x);
    int by2 = std::min(h_frame, _track_win.y + _track_win.height + pad_y);

    if (by1 > 0)         back_proj.rowRange(0, by1).setTo(0);
    if (by2 < h_frame)   back_proj.rowRange(by2, h_frame).setTo(0);
    if (bx1 > 0)         back_proj.colRange(0, bx1).setTo(0);
    if (bx2 < w_frame)   back_proj.colRange(bx2, w_frame).setTo(0);

#ifdef DRONE_USE_CUDA
    if (_use_gpu) {
        _gpu_bp_d.upload(back_proj);
        _gpu_gauss_bp->apply(_gpu_bp_d, _gpu_bp_d);
        _gpu_bp_d.download(back_proj);
    } else
#endif
    cv::GaussianBlur(back_proj, back_proj, cv::Size(3, 3), 0);

    cv::RotatedRect ret;
    ret = cv::CamShift(back_proj, _track_win, _term_crit);

    // Clamp window
    _track_win.x      = std::max(0, std::min(_track_win.x, w_frame - 1));
    _track_win.y      = std::max(0, std::min(_track_win.y, h_frame - 1));
    _track_win.width  = std::max(1, std::min(_track_win.width,  w_frame - _track_win.x));
    _track_win.height = std::max(1, std::min(_track_win.height, h_frame - _track_win.y));

    // Validate
    bool camshift_bad = (_track_win.width  < 4  || _track_win.height < 4 ||
                         _track_win.width  > w_frame * 0.9 ||
                         _track_win.height > h_frame * 0.9);

    if (!camshift_bad) {
        cv::Mat roi_bp = back_proj(_track_win);
        double density = cv::mean(roi_bp)[0] / 255.0;
        if (density < 0.05) camshift_bad = true;
    }

    if (camshift_bad) {
        if (_kf_initialized && _miss_count < KF_MISS_MAX) {
            _miss_count++;
            double pred_dt = std::max(kf_dt, 1.0 / 30.0);
            _kf_x = kf1dPredict(_kf_x, pred_dt);
            _kf_y = kf1dPredict(_kf_y, pred_dt);
            int pcx = std::max(0, std::min((int)std::round(_kf_x.x0), w_frame - 1));
            int pcy = std::max(0, std::min((int)std::round(_kf_y.x0), h_frame - 1));
            int tw = std::max((int)_win_w_ema, 40);
            int th = std::max((int)_win_h_ema, 40);
            _track_win = {std::max(0, pcx - tw/2), std::max(0, pcy - th/2),
                          std::min(tw, w_frame), std::min(th, h_frame)};
            _drawCenterCross(frame, w_frame, h_frame);
            cv::circle(frame, {pcx, pcy}, 5, {0, 165, 255}, 2);
            cv::line(frame,   {0, pcy},   {w_frame, pcy},  {0, 165, 255}, 1);
            cv::line(frame,   {pcx, 0},   {pcx, h_frame},  {0, 165, 255}, 1);
            _updateHistogramWindow();
            return {pcx, pcy};
        } else {
            resetTracker();
            _drawCenterCross(frame, w_frame, h_frame);
            _updateHistogramWindow();
            return {-1, -1};
        }
    }

    // EMA of window size
    _win_w_ema = EMA_ALPHA * _track_win.width  + (1 - EMA_ALPHA) * _win_w_ema;
    _win_h_ema = EMA_ALPHA * _track_win.height + (1 - EMA_ALPHA) * _win_h_ema;

    // Snap: if blob disagrees with CamShift centre, snap to blob
    if (blob) {
        int cs_cx = (int)ret.center.x;
        int cs_cy = (int)ret.center.y;
        int b_cx  = blob->x + blob->width  / 2;
        int b_cy  = blob->y + blob->height / 2;
        double dist = std::hypot(cs_cx - b_cx, cs_cy - b_cy);
        if (dist > std::max(blob->width, blob->height) * 0.5) {
            int pad = std::max(8, (int)(std::max(blob->width, blob->height) * 0.3));
            _track_win = {std::max(0, blob->x - pad),
                          std::max(0, blob->y - pad),
                          std::min(w_frame - std::max(0, blob->x - pad), blob->width  + 2*pad),
                          std::min(h_frame - std::max(0, blob->y - pad), blob->height + 2*pad)};
            _detect_count = std::max(_detect_count - 1, 0);
        }
    }

    double raw_cx = ret.center.x;
    double raw_cy = ret.center.y;
    _miss_count = 0;

    // Kalman update
    int cx, cy;
    if (!_kf_initialized) {
        _kf_x = {raw_cx, 0.0, 1.0, 0.0, 0.0, 1.0};
        _kf_y = {raw_cy, 0.0, 1.0, 0.0, 0.0, 1.0};
        _kf_initialized = true;
        cx = (int)std::round(raw_cx);
        cy = (int)std::round(raw_cy);
    } else {
        _kf_x = kf1dUpdate(_kf_x, raw_cx, kf_dt);
        _kf_y = kf1dUpdate(_kf_y, raw_cy, kf_dt);
        cx = (int)std::round(_kf_x.x0);
        cy = (int)std::round(_kf_y.x0);
    }
    cx = std::max(0, std::min(cx, w_frame - 1));
    cy = std::max(0, std::min(cy, h_frame - 1));

    // Draw
    auto [ex, ey] = errorXY(cx, cy, w_frame, h_frame);
    bool centred = std::abs(ex) < CENTER_THRESHOLD && std::abs(ey) < CENTER_THRESHOLD;
    cv::Scalar box_col = centred ? cv::Scalar(0,233,0) : cv::Scalar(203,192,233);

    cv::Point2f pts[4];
    ret.points(pts);
    std::vector<cv::Point> poly;
    for (auto& p : pts) poly.push_back({(int)p.x, (int)p.y});
    cv::polylines(frame, poly, true, box_col, 2);

    cv::line(frame, {0, cy}, {w_frame, cy}, {0,233,233}, 1);
    cv::line(frame, {cx, 0}, {cx, h_frame}, {0,233,233}, 1);
    cv::circle(frame, {cx, cy}, 3, {0,233,233}, -1);

    _drawCenterCross(frame, w_frame, h_frame);
    _updateHistogramWindow();
    return {cx, cy};
}
