# drone-seeker-cpp

C++ rewrite of the drone-seeker pink-object visual tracking pipeline.

## Dependencies

| Dependency | Min version | Notes |
|---|---|---|
| CMake | 3.16 | Build system |
| C++ compiler | C++17 | GCC 8+, Clang 7+, MSVC 2017+ |
| OpenCV | 4.x | `opencv-dev` / `libopencv-dev` |
| MAVLink C headers | v2 | `c_library_v2` — fetched below |

### Install OpenCV

**Ubuntu / Debian**
```bash
sudo apt install libopencv-dev
```

**macOS (Homebrew)**
```bash
brew install opencv
```

**Windows (vcpkg)**
```bash
vcpkg install opencv4
```

### Fetch MAVLink headers

The project expects the `mavlink/c_library_v2` header-only library inside `mavlink/`.

```bash
git clone --depth=1 https://github.com/mavlink/c_library_v2.git mavlink
```

> **Dialect:** the code includes `<ardupilotmega/mavlink.h>`, which is present in `c_library_v2/ardupilotmega/`.

---

## Build

```bash
# From the drone-seeker-cpp directory
cmake -B build
cmake --build build -j$(nproc)   # Linux/macOS
cmake --build build              # Windows
```

If CMake cannot find OpenCV or the MAVLink headers automatically, pass the paths explicitly:

```bash
cmake -B build \
  -DMAVLINK_INCLUDE_DIR=/path/to/c_library_v2 \
  -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4
```

Build produces two binaries inside `build/`:

| Binary | Description |
|---|---|
| `drone_seeker` | Main tracker + MAVLink controller |
| `calibrate_color` | Interactive HSV calibration tool |

---

## Calibrate color (first time)

Run `calibrate_color` with your camera pointed at the target:

```bash
./build/calibrate_color 0          # camera index 0
./build/calibrate_color video.mp4  # or a video file
```

1. Drag a rectangle over the pink target to sample its hue.
2. Repeat on several frames to build up a robust histogram.
3. Press **`s`** to save `color_histogram.txt` in the current directory.
4. Press **`r`** to reset the sample if needed, **`q`** to quit.

`color_histogram.txt` must be present in the working directory when `drone_seeker` runs.

---

## Run

```bash
./build/drone_seeker [options]
```

### Options

| Option | Default | Description |
|---|---|---|
| `--connection STR` | `udpin:0.0.0.0:14560` | MAVLink connection string |
| `--baud N` | `57600` | Baud rate (serial connections) |
| `--source STR` | `0` | Camera index or video file path |
| `--res W H` | — | Request capture resolution (e.g. `--res 1280 720`) |
| `--crop X Y W H` | — | Crop ROI from each frame (use `0` for W/H = rest of dimension) |
| `--mask-algo ALGO` | `all` | `gaussian` / `adaptive` / `inrange` / `all` (2-of-3 vote) |
| `--histogram` | off | Show calibration histogram window |
| `--mask` | off | Show detection mask window |
| `--no-camshift` | — | Use blob centroid directly (disable CamShift tracking) |
| `--no-box-filter` | — | Accept any blob shape (disable extent/solidity gates) |
| `--no-prediction` | — | Disable latency + PN lead input prediction |
| `--no-hud-pitch` | — | Disable pitch ladder in HUD |
| `--no-hud-yaw` | — | Disable yaw compass in HUD |
| `--debug` | off | Log telemetry to `tracking.csv` during TRACKING mode |
| `--record` | off | Record annotated video to `tracking_<timestamp>.avi` |
| `--auto` | off | Auto mode: enter TRACKING when within 700 m of target on final WP |

### Connection strings

| String | Description |
|---|---|
| `udpin:0.0.0.0:14560` | Bind UDP on port 14560; reply to first sender |
| `udp:127.0.0.1:14550` | Connect UDP to SITL / GCS at host:port |
| `udpout:192.168.1.1:14550` | Alias for `udp:` |
| `/dev/ttyUSB0` | Serial at `--baud` (default 57600) |
| `serial:/dev/ttyUSB0:115200` | Serial with explicit baud |

### Examples

**SITL / simulator (UDP)**
```bash
./build/drone_seeker --connection udp:127.0.0.1:14550 --source 0
```

**Companion computer over serial**
```bash
./build/drone_seeker --connection /dev/ttyUSB0 --baud 115200 --source 0
```

**Custom resolution with crop and debug log**
```bash
./build/drone_seeker \
  --connection udpin:0.0.0.0:14560 \
  --source 0 \
  --res 1280 720 \
  --crop 320 180 640 360 \
  --debug
```

**Auto mission mode with video recording**
```bash
./build/drone_seeker \
  --connection /dev/ttyUSB0 --baud 57600 \
  --source /dev/video1 \
  --auto --record
```

---

## Runtime controls

| Key | Action |
|---|---|
| **`q`** | Quit |
| **`r`** | Reset tracker (lose lock, re-acquire) |

---

## ArduPlane parameters

The following parameters must match on the flight controller.  
`drone_seeker` fetches the `TRK_*` values from ArduPlane on connect and prints them; compile-time defaults are used as fallbacks.

| Parameter | Default | Description |
|---|---|---|
| `TRK_TGT_LAT` | `-6.897367724` | Target latitude (decimal degrees) |
| `TRK_TGT_LON` | `107.566559898` | Target longitude (decimal degrees) |
| `TRK_TGT_ALT` | `744.0` | Target altitude MSL (m) |
| `TRK_TERM_ALT` | `0.0` | Terminal phase altitude threshold (0 = disabled) |
| `TRK_MAX_DEG` | `30.0` | Full-scale error angle (degrees) |
| `TRK_PITCH_OFFSET` | `3.0` | Cruise nose-up pitch bias (degrees) |

---

## Project structure

```
drone-seeker-cpp/
├── CMakeLists.txt
├── mavlink/               ← c_library_v2 (clone here)
├── color_histogram.txt    ← generated by calibrate_color
├── include/
│   ├── MavlinkConn.hpp    MAVLink UDP/serial link + message cache
│   ├── Seeker.hpp         Visual tracker (CamShift + Kalman + histogram detection)
│   ├── HudDisplay.hpp     Pitch ladder / yaw tape overlay
│   └── SeekerCtrl.hpp     Main controller (mode FSM + MAVLink + main loop)
└── src/
    ├── MavlinkConn.cpp
    ├── Seeker.cpp
    ├── HudDisplay.cpp
    ├── SeekerCtrl.cpp
    ├── main.cpp
    └── calibrate_color.cpp
```
