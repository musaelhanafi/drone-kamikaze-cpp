// test_record_camera.cpp — Open camera and record to a timestamped MP4 file.
//
// Usage: ./test_record_camera [camera_index]   (default: 0)
// Press 'q' or Ctrl-C to stop recording.
//
// Build: included in the test_record_camera CMake target.

#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>

static std::string timestamped_filename()
{
    std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "record_%Y%m%d_%H%M%S.mp4", std::localtime(&t));
    return buf;
}

int main(int argc, char* argv[])
{
    int camera_index = 0;
    int req_width = 0, req_height = 0;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--res" && i + 2 < argc) {
            req_width  = std::stoi(argv[++i]);
            req_height = std::stoi(argv[++i]);
        } else {
            camera_index = std::stoi(argv[i]);
        }
    }

    cv::VideoCapture cap(camera_index);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "Error: cannot open camera %d\n", camera_index);
        return 1;
    }

    if (req_width > 0 && req_height > 0) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  req_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, req_height);
    }

    int width  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    using Clock = std::chrono::steady_clock;
    using Sec   = std::chrono::duration<double>;

    // Measure actual capture FPS over a warmup period so the VideoWriter
    // uses the real rate — avoids slow-motion playback when the camera
    // delivers fewer frames than CAP_PROP_FPS claims.
    static constexpr int    WARMUP_FRAMES = 30;
    static constexpr double WARMUP_MIN_S  = 1.0;

    std::printf("Measuring actual camera FPS (%d frames)...\n", WARMUP_FRAMES);
    cv::Mat warmup_frame;
    auto t_warmup_start = Clock::now();
    for (int i = 0; i < WARMUP_FRAMES; ++i) {
        cap >> warmup_frame;
        if (warmup_frame.empty()) {
            std::fprintf(stderr, "Error: empty frame during warmup\n");
            return 1;
        }
    }
    double warmup_elapsed = Sec(Clock::now() - t_warmup_start).count();
    if (warmup_elapsed < WARMUP_MIN_S) warmup_elapsed = WARMUP_MIN_S;
    double fps = WARMUP_FRAMES / warmup_elapsed;
    std::printf("Measured FPS: %.2f\n", fps);

    std::string outfile = timestamped_filename();
    cv::VideoWriter writer(outfile,
                           cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                           fps,
                           cv::Size(width, height));
    if (!writer.isOpened()) {
        std::fprintf(stderr, "Error: cannot open output file %s\n", outfile.c_str());
        return 1;
    }

    std::printf("Recording %dx%d @ %.2f fps → %s\n", width, height, fps, outfile.c_str());
    std::printf("Press 'q' to stop.\n");

    cv::Mat frame;
    long long frame_count = 0;
    double    measured_fps = fps;
    auto      t_start   = Clock::now();
    auto      t_last    = t_start;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::fprintf(stderr, "Warning: empty frame, stopping.\n");
            break;
        }

        ++frame_count;
        auto t_now = Clock::now();

        // Update FPS every second
        double elapsed_since_last = Sec(t_now - t_last).count();
        if (elapsed_since_last >= 1.0) {
            double total_elapsed = Sec(t_now - t_start).count();
            measured_fps = frame_count / total_elapsed;
            t_last = t_now;
            std::printf("\rFPS: %5.1f  frames: %lld", measured_fps, frame_count);
            std::fflush(stdout);
        }

        // Overlay FPS on frame
        char fps_label[32];
        std::snprintf(fps_label, sizeof(fps_label), "FPS: %.1f", measured_fps);
        cv::putText(frame, fps_label, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

        writer.write(frame);
        cv::imshow("Recording", frame);

        if ((cv::waitKey(1) & 0xFF) == 'q')
            break;
    }

    double total_elapsed = Sec(Clock::now() - t_start).count();
    double final_fps = (total_elapsed > 0.0) ? frame_count / total_elapsed : 0.0;
    std::printf("\nFinal: %lld frames in %.2f s = %.2f fps\n",
                frame_count, total_elapsed, final_fps);
    std::printf("Saved: %s\n", outfile.c_str());
    return 0;
}
