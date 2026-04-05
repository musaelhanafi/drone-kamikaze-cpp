#include "HudDisplay.hpp"
#include <cmath>
#include <cstdio>

HudDisplay::HudDisplay(int offsety, int size, bool show_pitch, bool show_yaw)
    : _offsety(offsety), _size(size),
      _show_pitch(show_pitch), _show_yaw(show_yaw)
{}

cv::Point2i HudDisplay::transform(double x, double y, double theta)
{
    double ct = std::cos(theta);
    double st = std::sin(theta);
    return {(int)(x * ct - y * st), (int)(x * st + y * ct)};
}

void HudDisplay::drawHud(bool is_enabled, cv::Mat& frame,
                         double lat, double lon,
                         double yaw, double pitch, double roll)
{
    if (is_enabled) {
        cv::Mat zero = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);

        double roll_neg = -roll;
        double roll_rad = roll_neg * M_PI / 180.0;
        double cr = std::cos(roll_rad);
        double sr = std::sin(roll_rad);
        int rows = frame.rows, cols = frame.cols;

        auto [xx1, yy1] = transform(3.0 * cols / 4.0, rows / 2.0 - _offsety, roll_rad);

        cv::Mat M = (cv::Mat_<float>(2, 3) <<
            (float)cr, (float)(-sr), (float)(-(xx1 - 3.0f*cols/4.0f)),
            (float)sr, (float)  cr,  (float)(-(yy1 - rows/2.0f + _offsety)));

        drawCenter(zero, roll_neg);
        if (_show_pitch) drawPitch(zero, pitch);

        cv::Point center((int)(3.0 * frame.cols / 4), (int)(frame.rows / 2.0 - _offsety));
        cv::Size  axes((int)(1.1 * _size), (int)(1.1 * _size));
        cv::Scalar color(0, 0, 255);

        cv::warpAffine(zero, zero, M, {cols, rows});

        int x = (int)(3.0 * frame.cols / 4.0);
        int y = (int)(frame.rows / 2.0 - _offsety);

        if (_show_pitch) {
            cv::line(zero,
                     {(int)(2.0 * _size / 3.0) + x, y},
                     {(int)(-2.0 * _size / 3.0) + x, y},
                     {0, 255, 255}, 2, cv::LINE_8);
            cv::ellipse(zero, center, axes, 270, -60, 60, color, 4);
        }
        cv::addWeighted(frame, 0.6, zero, 0.4, 0.0, frame);
    }

    if (_show_yaw) drawYaw(frame, lat, lon, yaw);
}

void HudDisplay::drawYaw(cv::Mat& frame, double lat, double lon, double yaw)
{
    int x = frame.cols / 2;
    int y = (int)(3.0 * frame.rows / 4.0);

    if (yaw > 180.0) yaw -= 360.0;
    int pp    = (int)(yaw / 5.0) * 5;
    int delta = (int)(yaw - pp);

    char latlon[64];
    std::snprintf(latlon, sizeof(latlon), "Location: %7.5f, %8.5f", lat, lon);
    cv::putText(frame, latlon, {x - 120, y + 60},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);

    for (int idx = 0; idx < 15; idx++) {
        int yy = idx - 7;
        int dd = yy * 5 + pp;
        if (dd < 0) dd += 360;

        char deg[8];
        std::snprintf(deg, sizeof(deg), "%3d", dd);
        const char* label = deg;
        if (dd == 0)   label = "  N ";
        if (dd == 90)  label = "  E ";
        if (dd == 180) label = "  S ";
        if (dd == 270) label = "  W ";

        int sx  = ((yy * 5 + pp) % 10 != 0) ? 10 : 15;
        int xx1 = 15 * yy - delta * 3;

        cv::Scalar color = (idx == 7) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        int size_line    = (idx == 7) ? 4 : 2;

        cv::line(frame, {xx1 + x, y - sx}, {xx1 + x, y + sx}, color, size_line, cv::LINE_8);
        if ((yy * 5 + pp) % 20 == 0 || label[2] == 'E' || label[2] == 'W') {
            cv::putText(frame, label, {xx1 + x - 20, y + sx + 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
        }
    }
}

void HudDisplay::drawPitch(cv::Mat& frame, double pitch)
{
    int x = (int)(3.0 * frame.cols / 4.0);
    int y = (int)(frame.rows / 2.0 - _offsety);

    int pp    = (int)(pitch / 5.0) * 5;
    int rem   = ((int)(pitch / 5.0) % 2 == 0) ? 1 : 0;
    int delta = (int)(pitch - pp);

    for (int idx = 0; idx < 7; idx++) {
        int yy = idx - 3;
        int dd = yy * 5 + pp;
        char deg[8];
        std::snprintf(deg, sizeof(deg), "%3d", dd);

        int sx    = (dd == 0) ? 50 : ((dd % 10 == 0) ? 15 : 10);
        cv::Scalar color = (dd == 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        int thick = (dd == 0) ? 4 : 2;

        int oy = yy * 15 - delta * 3;
        cv::line(frame, {x - sx, y + oy}, {x + sx, y + oy}, color, thick);
        if (idx % 2 == rem) {
            cv::putText(frame, deg, {x + 20, y + oy + 5},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
        }
    }
}

void HudDisplay::drawCenter(cv::Mat& frame, double roll)
{
    int x = (int)(3.0 * frame.cols / 4.0);
    int y = (int)(frame.rows / 2.0 - _offsety);

    cv::line(frame, {x, y - (int)(1.1 * _size)}, {x, y},
             {0, 255, 255}, 2, cv::LINE_8);

    char deg[8];
    std::snprintf(deg, sizeof(deg), "%3d", (int)(-roll));
    cv::putText(frame, deg, {x - 15, y - (int)(1.2 * _size)},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);
}
