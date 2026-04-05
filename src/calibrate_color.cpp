// calibrate_color.cpp — Interactive HSV calibration tool.
//
// Drag a rectangle over the target to sample its hue distribution.
// Press 's' to save color_histogram.txt, 'r' to reset, 'q' to quit.
//
// Build: included in the calibrate_color CMake target.

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

static const char* HIST_FILE = "color_histogram.txt";

// ── Mouse state ───────────────────────────────────────────────────────────────
struct MouseState {
    bool   dragging  = false;
    bool   has_roi   = false;
    cv::Point start_pt;
    cv::Point end_pt;
};

static void onMouse(int event, int x, int y, int /*flags*/, void* ud)
{
    auto* ms = static_cast<MouseState*>(ud);
    if (event == cv::EVENT_LBUTTONDOWN) {
        ms->dragging = true;
        ms->has_roi  = false;
        ms->start_pt = {x, y};
        ms->end_pt   = {x, y};
    } else if (event == cv::EVENT_MOUSEMOVE && ms->dragging) {
        ms->end_pt = {x, y};
    } else if (event == cv::EVENT_LBUTTONUP) {
        ms->dragging = false;
        ms->end_pt   = {x, y};
        ms->has_roi  = (ms->start_pt != ms->end_pt);
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
        cx += std::cos(a) * p;
        cy += std::sin(a) * p;
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

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    if (argc > 1 && (std::strcmp(argv[1], "--help") == 0 ||
                     std::strcmp(argv[1], "-h")     == 0)) {
        printf("Usage: %s [source]\n"
               "  source   Camera index (default: 0) or video file path\n"
               "\nControls:\n"
               "  Drag     Sample hue from rectangle\n"
               "  s        Save color_histogram.txt\n"
               "  r        Reset samples\n"
               "  q        Quit\n", argv[0]);
        return 0;
    }

    int source_idx = 0;
    std::string source_file;
    if (argc > 1) {
        char* end;
        long idx = std::strtol(argv[1], &end, 10);
        if (*end == '\0') source_idx = (int)idx;
        else source_file = argv[1];
    }

    cv::VideoCapture cap;
    if (!source_file.empty()) cap.open(source_file);
    else                       cap.open(source_idx);
    if (!cap.isOpened()) {
        fprintf(stderr, "Cannot open source\n");
        return 1;
    }

    const std::string WIN = "Calibrate";
    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    MouseState ms;
    cv::setMouseCallback(WIN, onMouse, &ms);

    std::vector<float> accum(180, 0.f);
    bool has_hist = false;

    printf("Drag a rectangle over the target to sample hue.\n"
           "  's' — save %s\n"
           "  'r' — reset\n"
           "  'q' — quit\n", HIST_FILE);

    while (true) {
        cv::Mat frame, display;
        if (!cap.read(frame)) { printf("End of stream.\n"); break; }
        frame.copyTo(display);

        // Draw ROI rectangle while dragging
        cv::Point p1(std::min(ms.start_pt.x, ms.end_pt.x),
                     std::min(ms.start_pt.y, ms.end_pt.y));
        cv::Point p2(std::max(ms.start_pt.x, ms.end_pt.x),
                     std::max(ms.start_pt.y, ms.end_pt.y));
        cv::Scalar rect_col = ms.has_roi ? cv::Scalar(0,255,0) : cv::Scalar(0,200,255);
        cv::rectangle(display, p1, p2, rect_col, 2);

        // Sample on newly completed drag
        if (ms.has_roi && p1 != p2) {
            cv::Rect roi(p1, p2);
            roi &= cv::Rect(0, 0, frame.cols, frame.rows);
            if (roi.area() > 0) {
                cv::Mat crop = frame(roi);
                cv::Mat hsv;
                cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

                // Compute hue histogram (180 bins, H channel)
                cv::Mat h_hist;
                int ch[] = {0};
                int histSize[] = {180};
                float ranges_arr[] = {0, 180};
                const float* ranges[] = {ranges_arr};
                cv::calcHist(&hsv, 1, ch, cv::Mat(), h_hist, 1, histSize, ranges);

                // Accumulate (simple sum)
                for (int i = 0; i < 180; i++)
                    accum[i] += h_hist.at<float>(i);
                has_hist = true;
                ms.has_roi = false;   // consume

                auto [mean, std] = circStats(accum);
                printf("[Cal] mean=%.1f  std=%.1f\n", mean, std);
            }
        }

        // Show histogram bar at bottom
        if (has_hist) {
            float max_v = *std::max_element(accum.begin(), accum.end());
            if (max_v > 0) {
                int bar_h = 40;
                int w = display.cols;
                cv::Mat bar = cv::Mat::zeros(bar_h, w, CV_8UC3);
                double bw = (double)w / 180.0;
                for (int i = 0; i < 180; i++) {
                    int bh2 = (int)(accum[i] / max_v * bar_h);
                    uint8_t hsv_px[3] = {(uint8_t)i, 220, 200};
                    cv::Mat hsv_m(1, 1, CV_8UC3, hsv_px);
                    cv::Mat bgr_m;
                    cv::cvtColor(hsv_m, bgr_m, cv::COLOR_HSV2BGR);
                    cv::Scalar col(bgr_m.at<cv::Vec3b>(0,0)[0],
                                   bgr_m.at<cv::Vec3b>(0,0)[1],
                                   bgr_m.at<cv::Vec3b>(0,0)[2]);
                    int x0 = (int)(i * bw);
                    int x1 = std::max(x0+1, (int)((i+1)*bw));
                    cv::rectangle(bar, {x0, bar_h-bh2}, {x1, bar_h}, col, -1);
                }
                cv::Mat combined;
                cv::vconcat(display, bar, combined);
                display = combined;
            }
        }

        cv::putText(display, has_hist ? "Hist: OK — press 's' to save" : "Drag ROI to sample",
                    {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);

        cv::imshow(WIN, display);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 'q') break;
        if (key == 'r') {
            std::fill(accum.begin(), accum.end(), 0.f);
            has_hist = false;
            printf("[Cal] Reset.\n");
        }
        if (key == 's' && has_hist) {
            std::ofstream ofs(HIST_FILE);
            if (!ofs.is_open()) {
                fprintf(stderr, "Cannot write %s\n", HIST_FILE);
            } else {
                for (int i = 0; i < 180; i++)
                    ofs << accum[i] << "\n";
                ofs.close();
                auto [mean, std] = circStats(accum);
                printf("[Cal] Saved %s  (mean=%.1f std=%.1f)\n", HIST_FILE, mean, std);
            }
        }
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
