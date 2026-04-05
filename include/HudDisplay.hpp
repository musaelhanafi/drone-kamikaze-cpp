#pragma once
#include <opencv2/opencv.hpp>

class HudDisplay {
public:
    explicit HudDisplay(int offsety   = 0,
                        int size      = 120,
                        bool show_pitch = true,
                        bool show_yaw   = true);

    void drawHud(bool is_enabled, cv::Mat& frame,
                 double lat, double lon,
                 double yaw, double pitch, double roll);

private:
    static cv::Point2i transform(double x, double y, double theta);
    void drawYaw  (cv::Mat& frame, double lat, double lon, double yaw);
    void drawPitch(cv::Mat& frame, double pitch);
    void drawCenter(cv::Mat& frame, double roll);

    int  _offsety;
    int  _size;
    bool _show_pitch;
    bool _show_yaw;
};
