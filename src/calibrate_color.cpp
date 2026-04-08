// calibrate_color.cpp — Interactive HSV calibration tool.
//
// Drag a rectangle over the target to sample its hue distribution.
// Press 's' to save color_histogram.txt, 'r' to reset, 'q' to quit.
//
// Build: included in the calibrate_color CMake target.

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

static const char* HIST_FILE   = "color_histogram.txt";
static const char* MASK_WIN    = "Mask";
static constexpr double GAUSS_SIGMA = 2.0;   // must match Seeker.hpp

// ── Mouse state ───────────────────────────────────────────────────────────────
struct MouseState {
    bool      dragging = false;
    bool      has_roi  = false;
    cv::Point start_pt, end_pt;
};

static void onMouse(int event, int x, int y, int /*flags*/, void* ud)
{
    auto* ms = static_cast<MouseState*>(ud);
    if (event == cv::EVENT_LBUTTONDOWN) {
        ms->dragging = true; ms->has_roi = false;
        ms->start_pt = ms->end_pt = {x, y};
    } else if (event == cv::EVENT_MOUSEMOVE && ms->dragging) {
        ms->end_pt = {x, y};
    } else if (event == cv::EVENT_LBUTTONUP) {
        ms->dragging = false; ms->end_pt = {x, y};
        ms->has_roi = (ms->start_pt != ms->end_pt);
    }
}

// ── Circular mean/std ─────────────────────────────────────────────────────────
static std::pair<double,double> circStats(const std::vector<float>& hist)
{
    double total = 0;
    for (float v : hist) total += v;
    if (total == 0) return {90.0, 30.0};

    const double two_pi = 2.0 * M_PI;
    double cx = 0, cy = 0;
    for (int i = 0; i < 180; i++) {
        double p = hist[i] / total;
        double a = i * two_pi / 180.0;
        cx += std::cos(a) * p; cy += std::sin(a) * p;
    }
    double mean_rad = std::atan2(cy, cx);
    if (mean_rad < 0) mean_rad += two_pi;
    double mean = mean_rad * 180.0 / two_pi;

    double var = 0;
    for (int i = 0; i < 180; i++) {
        double p    = hist[i] / total;
        double diff = std::fmod(i - mean + 90.0, 180.0) - 90.0;
        var += diff * diff * p;
    }
    return {mean, std::sqrt(std::max(var, 1.0))};
}

// ── inRange band (wrapping-aware) ─────────────────────────────────────────────
struct Band {
    cv::Scalar lo_a, hi_a, lo_b, hi_b;
    enum Mode { SINGLE, WRAP } mode = SINGLE;
};

static Band makeBand(double mean, double hw)
{
    Band b;
    double lo = mean - hw, hi = mean + hw;
    if (lo < 0) {
        b.mode = Band::WRAP;
        b.lo_a = {std::max(0.0, lo + 180.0), 40, 40}; b.hi_a = {179, 255, 255};
        b.lo_b = {0, 40, 40};                          b.hi_b = {std::min(179.0, hi), 255, 255};
    } else if (hi > 179) {
        b.mode = Band::WRAP;
        b.lo_a = {std::max(0.0, lo), 40, 40};          b.hi_a = {179, 255, 255};
        b.lo_b = {0, 40, 40};                          b.hi_b = {std::min(179.0, hi - 180.0), 255, 255};
    } else {
        b.lo_a = {std::max(0.0, lo), 40, 40};          b.hi_a = {std::min(179.0, hi), 255, 255};
    }
    return b;
}

static cv::Mat applyBand(const cv::Mat& hsv, const Band& b)
{
    cv::Mat a;
    cv::inRange(hsv, b.lo_a, b.hi_a, a);
    if (b.mode == Band::WRAP) {
        cv::Mat bm; cv::inRange(hsv, b.lo_b, b.hi_b, bm);
        cv::bitwise_or(a, bm, a);
    }
    return a;
}

// ── Detection mask algorithms (mirror of Seeker.cpp) ─────────────────────────
static cv::Mat maskInrange(const cv::Mat& hsv,
                           const Band& core, const Band& outer,
                           const cv::Mat& outer_lut)
{
    cv::Mat c = applyBand(hsv, core);
    cv::Mat o = applyBand(hsv, outer);
    cv::subtract(o, c, o);
    cv::Mat half; cv::LUT(o, outer_lut, half);
    cv::Mat result; cv::bitwise_or(c, half, result);
    return result;
}

static cv::Mat maskGaussian(const cv::Mat& hsv, const cv::Mat& h_blur,
                            const cv::Mat& conf_hist, const Band& outer)
{
    cv::Mat planes[3]; cv::split(hsv, planes);
    h_blur.copyTo(planes[0]);
    cv::Mat hsv_blur; cv::merge(planes, 3, hsv_blur);

    cv::Mat bp;
    int ch[] = {0}; float rng[] = {0, 180}; const float* ranges[] = {rng};
    cv::calcBackProject(&hsv_blur, 1, ch, conf_hist, bp, ranges);

    cv::Mat thresh; cv::threshold(bp, thresh, 0, 255, cv::THRESH_BINARY);
    cv::Mat sv_ok = applyBand(hsv, outer);
    cv::Mat result; cv::bitwise_and(thresh, sv_ok, result);
    return result;
}

static cv::Mat maskAdaptive(const cv::Mat& h_blur, const cv::Mat& hue_gate_lut)
{
    cv::Mat adapt;
    cv::adaptiveThreshold(h_blur, adapt, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 3);
    cv::Mat gate; cv::LUT(h_blur, hue_gate_lut, gate);
    cv::Mat result; cv::bitwise_and(adapt, gate, result);
    return result;
}

static cv::Mat detectionMask(const cv::Mat& hsv,
                             const std::string& algo,
                             const cv::Mat& conf_hist,
                             const Band& core, const Band& outer,
                             const cv::Mat& outer_lut, const cv::Mat& hue_gate_lut,
                             const cv::Mat& kern3)
{
    cv::Mat mask;
    if (conf_hist.empty()) {
        // Fallback: hardcoded pink range (no histogram loaded)
        cv::inRange(hsv,
                    cv::Scalar(130, 40, 80), cv::Scalar(173, 233, 233),
                    mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,   kern3);
        cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kern3);
        return mask;
    }

    cv::Mat h_blur;
    if (algo != "inrange")
        cv::GaussianBlur(hsv, h_blur, cv::Size(5, 5), 0);

    if (algo == "inrange") {
        mask = maskInrange(hsv, core, outer, outer_lut);
    } else if (algo == "gaussian") {
        cv::Mat planes[3]; cv::split(h_blur, planes);
        mask = maskGaussian(hsv, planes[0], conf_hist, outer);
    } else if (algo == "adaptive") {
        cv::Mat planes[3]; cv::split(h_blur, planes);
        mask = maskAdaptive(planes[0], hue_gate_lut);
    } else {  // "all" — 2-of-3 majority vote
        cv::Mat planes[3]; cv::split(h_blur, planes);
        cv::Mat m1 = maskGaussian(hsv, planes[0], conf_hist, outer);
        cv::Mat m2 = maskAdaptive(planes[0], hue_gate_lut);
        cv::Mat m3 = maskInrange(hsv, core, outer, outer_lut);
        cv::Mat votes = cv::Mat::zeros(hsv.rows, hsv.cols, CV_8U);
        cv::Mat tmp;
        cv::threshold(m1, tmp, 0, 1, cv::THRESH_BINARY); cv::add(votes, tmp, votes);
        cv::threshold(m2, tmp, 0, 1, cv::THRESH_BINARY); cv::add(votes, tmp, votes);
        cv::threshold(m3, tmp, 0, 1, cv::THRESH_BINARY); cv::add(votes, tmp, votes);
        cv::threshold(votes, mask, 1, 255, cv::THRESH_BINARY);
    }
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kern3);
    return mask;
}

// ── Build detection state from accumulated histogram ──────────────────────────
struct DetState {
    cv::Mat conf_hist;
    Band    core, outer;
    cv::Mat outer_lut;
    cv::Mat hue_gate_lut;
    double  mean = -1, std = -1;

    void build(const std::vector<float>& accum) {
        cv::Mat hist_mat(180, 1, CV_32F, const_cast<float*>(accum.data()));
        conf_hist = hist_mat.clone();

        auto [m, s] = circStats(accum);
        mean = m; std = s;

        // Confidence hist: zero bins outside mean ± GAUSS_SIGMA*std
        float* p = conf_hist.ptr<float>(0);
        for (int i = 0; i < 180; i++) {
            double d = std::abs(i - mean);
            d = std::min(d, 180.0 - d);
            if (d >= GAUSS_SIGMA * std) p[i] = 0.f;
        }

        core  = makeBand(mean, std * 1.0);
        outer = makeBand(mean, std * GAUSS_SIGMA);

        outer_lut = cv::Mat(1, 256, CV_8U, cv::Scalar(0));
        outer_lut.at<uchar>(0, 255) = 128;

        hue_gate_lut = cv::Mat(1, 256, CV_8U, cv::Scalar(0));
        double sigma_thresh = GAUSS_SIGMA * std;
        for (int i = 0; i < 180; i++) {
            double d = std::abs(i - mean);
            d = std::min(d, 180.0 - d);
            hue_gate_lut.at<uchar>(0, i) = (d < sigma_thresh) ? 255 : 0;
        }
    }
};

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    // ── Argument parsing ──────────────────────────────────────────────────────
    int         source_idx  = 0;
    std::string source_file;
    std::string mask_algo;   // empty = no mask window
    std::string out_file    = HIST_FILE;

    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];
        if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            printf(
                "Usage: %s [options]\n"
                "  --source STR     Camera index or video file (default: 0)\n"
                "  --output FILE    Histogram output file (default: %s)\n"
                "  --mask ALGO      Show detection mask window.\n"
                "                   ALGO: gaussian | adaptive | inrange | all\n"
                "\nControls:\n"
                "  Drag     Sample hue from rectangle (S/V-masked)\n"
                "  s        Save histogram\n"
                "  r        Reset samples\n"
                "  q        Quit\n", argv[0], HIST_FILE);
            return 0;
        } else if (std::strcmp(a, "--source") == 0 && i+1 < argc) {
            const char* src = argv[++i];
            char* end; long idx = std::strtol(src, &end, 10);
            if (*end == '\0') source_idx = (int)idx;
            else              source_file = src;
        } else if (std::strcmp(a, "--output") == 0 && i+1 < argc) {
            out_file = argv[++i];
        } else if (std::strcmp(a, "--mask") == 0 && i+1 < argc) {
            mask_algo = argv[++i];
            if (mask_algo != "gaussian" && mask_algo != "adaptive" &&
                mask_algo != "inrange"  && mask_algo != "all") {
                fprintf(stderr, "Unknown --mask algo '%s'. "
                        "Choose: gaussian adaptive inrange all\n", mask_algo.c_str());
                return 1;
            }
        } else {
            // Legacy positional: first non-flag arg is source
            char* end; long idx = std::strtol(a, &end, 10);
            if (*end == '\0') source_idx = (int)idx;
            else              source_file = a;
        }
    }

    // ── Open camera ───────────────────────────────────────────────────────────
    cv::VideoCapture cap;
    if (!source_file.empty()) cap.open(source_file);
    else                       cap.open(source_idx);
    if (!cap.isOpened()) { fprintf(stderr, "Cannot open source\n"); return 1; }

    // ── Windows ───────────────────────────────────────────────────────────────
    const std::string WIN = "Calibrate";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    MouseState ms;
    cv::setMouseCallback(WIN, onMouse, &ms);
    if (!mask_algo.empty())
        cv::namedWindow(MASK_WIN, cv::WINDOW_NORMAL);

    // ── Load existing histogram ───────────────────────────────────────────────
    std::vector<float> accum(180, 0.f);
    bool has_hist = false;
    DetState det;
    cv::Mat kern3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    {
        std::ifstream ifs(out_file);
        if (ifs.is_open()) {
            float v; int n = 0;
            while (ifs >> v && n < 180) accum[n++] = v;
            if (n == 180) {
                has_hist = true;
                det.build(accum);
                printf("[Cal] Loaded %s  mean=%.1f  std=%.1f\n",
                       out_file.c_str(), det.mean, det.std);
            }
        }
    }

    if (mask_algo.empty())
        printf("Drag or press 'd' to select a rectangle over the target to sample hue.\n");
    else
        printf("Drag or press 'd' to select a rectangle. Mask window shows '%s' detection.\n",
               mask_algo.c_str());
    printf("  'd' — draw ROI\n  's' — save %s\n  'r' — reset\n  'q' — quit\n", out_file.c_str());

    cv::Mat hsv_buf;

    while (true) {
        cv::Mat frame, display;
        if (!cap.read(frame)) { printf("End of stream.\n"); break; }
        frame.copyTo(display);

        // ── Compute mask & detection ──────────────────────────────────────────
        cv::cvtColor(frame, hsv_buf, cv::COLOR_BGR2HSV);
        {
            std::string algo = mask_algo.empty() ? "inrange" : mask_algo;
            cv::Mat mask = detectionMask(hsv_buf, algo,
                                         det.conf_hist,
                                         det.core, det.outer,
                                         det.outer_lut, det.hue_gate_lut,
                                         kern3);
            if (!mask_algo.empty())
                cv::imshow(MASK_WIN, mask);

            // Find best blob and draw bounding rect on display
            if (has_hist) {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                double best_area = 0;
                cv::Rect  best_rect;
                for (const auto& c : contours) {
                    double area = cv::contourArea(c);
                    if (area < 20.0) continue;
                    if (area > best_area) {
                        best_area = area;
                        best_rect = cv::boundingRect(c);
                    }
                }
                if (best_area > 0) {
                    cv::rectangle(display, best_rect, cv::Scalar(0, 255, 0), 2);
                    int cx = best_rect.x + best_rect.width  / 2;
                    int cy = best_rect.y + best_rect.height / 2;
                    cv::circle(display, {cx, cy}, 4, {0, 255, 0}, -1);
                    char lbl[32];
                    std::snprintf(lbl, sizeof(lbl), "%dx%d", best_rect.width, best_rect.height);
                    cv::putText(display, lbl, {best_rect.x, best_rect.y - 6},
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 0}, 1);
                }
            }
        }

        // ── Draw drag ROI ─────────────────────────────────────────────────────
        cv::Point p1(std::min(ms.start_pt.x, ms.end_pt.x),
                     std::min(ms.start_pt.y, ms.end_pt.y));
        cv::Point p2(std::max(ms.start_pt.x, ms.end_pt.x),
                     std::max(ms.start_pt.y, ms.end_pt.y));
        cv::Scalar rect_col = ms.has_roi ? cv::Scalar(0,255,0) : cv::Scalar(0,200,255);
        cv::rectangle(display, p1, p2, rect_col, 2);

        // ── Sample on completed drag ──────────────────────────────────────────
        if (ms.has_roi && p1 != p2) {
            cv::Rect roi(p1, p2);
            roi &= cv::Rect(0, 0, frame.cols, frame.rows);
            if (roi.area() > 0) {
                cv::Mat crop_hsv = hsv_buf(roi);

                // Only sample sufficiently-saturated and bright pixels
                cv::Mat sv_mask;
                cv::inRange(crop_hsv,
                            cv::Scalar(0, 40, 40), cv::Scalar(179, 255, 255),
                            sv_mask);

                cv::Mat h_hist;
                int ch[] = {0}; int histSize[] = {180};
                float ranges_arr[] = {0, 180}; const float* ranges[] = {ranges_arr};
                cv::calcHist(&crop_hsv, 1, ch, sv_mask, h_hist, 1, histSize, ranges);

                for (int i = 0; i < 180; i++)
                    accum[i] += h_hist.at<float>(i);
                has_hist = true;
                ms.has_roi = false;

                det.build(accum);
                printf("[Cal] mean=%.1f  std=%.1f\n", det.mean, det.std);
            }
        }

        // ── Histogram bar at bottom ───────────────────────────────────────────
        if (has_hist) {
            float max_v = *std::max_element(accum.begin(), accum.end());
            if (max_v > 0) {
                int bar_h = 40, w = display.cols;
                cv::Mat bar = cv::Mat::zeros(bar_h, w, CV_8UC3);
                double bw = (double)w / 180.0;
                for (int i = 0; i < 180; i++) {
                    int bh2 = (int)(accum[i] / max_v * bar_h);
                    uint8_t hsv_px[3] = {(uint8_t)i, 220, 200};
                    cv::Mat hsv_m(1, 1, CV_8UC3, hsv_px), bgr_m;
                    cv::cvtColor(hsv_m, bgr_m, cv::COLOR_HSV2BGR);
                    cv::Scalar col(bgr_m.at<cv::Vec3b>(0,0)[0],
                                   bgr_m.at<cv::Vec3b>(0,0)[1],
                                   bgr_m.at<cv::Vec3b>(0,0)[2]);
                    int x0 = (int)(i * bw), x1 = std::max(x0+1, (int)((i+1)*bw));
                    cv::rectangle(bar, {x0, bar_h-bh2}, {x1, bar_h}, col, -1);
                }
                cv::Mat combined; cv::vconcat(display, bar, combined);
                display = combined;
            }
        }

        // ── Status label ──────────────────────────────────────────────────────
        char status[128];
        if (has_hist)
            std::snprintf(status, sizeof(status),
                "mean=%.1f std=%.1f — 'd'=draw 's'=save 'r'=reset", det.mean, det.std);
        else
            std::snprintf(status, sizeof(status), "Drag or press 'd' to select ROI");
        cv::putText(display, status,
                    {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);

        cv::imshow(WIN, display);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 'q') break;
        if (key == 'd') {
            cv::Rect2d sel = cv::selectROI(WIN, display, false, false);
            if (sel.width > 0 && sel.height > 0) {
                ms.start_pt = {(int)sel.x, (int)sel.y};
                ms.end_pt   = {(int)(sel.x + sel.width), (int)(sel.y + sel.height)};
                ms.has_roi  = true;
                ms.dragging = false;
            }
        }
        if (key == 'r') {
            std::fill(accum.begin(), accum.end(), 0.f);
            has_hist = false;
            det = DetState{};
            printf("[Cal] Reset.\n");
        }
        if (key == 's' && has_hist) {
            std::ofstream ofs(out_file);
            if (!ofs.is_open()) {
                fprintf(stderr, "Cannot write %s\n", out_file.c_str());
            } else {
                for (int i = 0; i < 180; i++) ofs << accum[i] << "\n";
                ofs.close();
                printf("[Cal] Saved %s  (mean=%.1f std=%.1f)\n",
                       out_file.c_str(), det.mean, det.std);
            }
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
