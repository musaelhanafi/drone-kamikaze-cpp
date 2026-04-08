# drone-seeker-cpp

C++ rewrite of the drone-seeker pink-object visual tracking pipeline.

## Dependencies

| Dependency | Min version | Notes |
|---|---|---|
| CMake | 3.16 | Build system |
| C++ compiler | C++17 | GCC 8+, Clang 7+, MSVC 2017+ |
| OpenCV | 4.x | `opencv-dev` / `libopencv-dev` |
| MAVLink C headers | v2 | `c_library_v2` — fetched below |
| SDL2 | 2.x | **Optional** — required for `--joystick` |

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

### Install SDL2 (required for `--joystick`)

**Ubuntu / Debian**
```bash
sudo apt install libsdl2-dev
```

**macOS (Homebrew)**
```bash
brew install sdl2
```

**Windows (vcpkg)**
```bash
vcpkg install sdl2
```

SDL2 is auto-detected by CMake. If not found, joystick support is silently disabled and `--joystick` will print an error at runtime. Force-disable with `-DWITH_JOYSTICK=OFF`.

---

### GPU acceleration (optional)

The tracker can offload the hot per-frame operations (colour-space conversion,
inRange masking, morphology, back-projection blur) to a CUDA GPU via OpenCV's
CUDA modules.  It is completely optional — the build falls back to CPU if CUDA is
not available, and the feature is detected automatically at both compile time and
run time.

**Requirements**

| Requirement | Notes |
|---|---|
| NVIDIA GPU | Kepler (GTX 600) or newer |
| CUDA Toolkit | 11.x or 12.x — must match the OpenCV CUDA build |
| OpenCV built with CUDA | The default `apt`/`brew` packages are CPU-only; see below |

**Build OpenCV with CUDA (Ubuntu)**

```bash
# Install CUDA Toolkit from https://developer.nvidia.com/cuda-downloads first
sudo apt install -y cmake g++ libgtk-3-dev libavcodec-dev libavformat-dev \
    libswscale-dev libv4l-dev

git clone --depth=1 https://github.com/opencv/opencv.git
git clone --depth=1 https://github.com/opencv/opencv_contrib.git

# Adjust CUDA_ARCH_BIN to your GPU's compute capability (see note below)
cmake -B opencv/build opencv \
    -DOPENCV_EXTRA_MODULES_PATH=opencv_contrib/modules \
    -DWITH_CUDA=ON \
    -DCUDA_ARCH_BIN="6.1;7.5;8.6;8.9" \
    -DBUILD_opencv_cudaimgproc=ON \
    -DBUILD_opencv_cudafilters=ON \
    -DBUILD_opencv_cudaarithm=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

cmake --build opencv/build -j$(nproc)
sudo cmake --install opencv/build
```

> Find your GPU's compute capability at <https://developer.nvidia.com/cuda-gpus>.
> Common values: GTX 10xx → `6.1`, RTX 20xx → `7.5`, RTX 30xx → `8.6`,
> RTX 40xx → `8.9`, Jetson Orin → `8.7`.

**Build drone_seeker with GPU**

CMake detects the CUDA build automatically via `OpenCV_CUDA_VERSION`:

```bash
cmake -B build -DOpenCV_DIR=/usr/local/lib/cmake/opencv4
cmake --build build -j$(nproc)
# Build output will show:
#   [drone_seeker] OpenCV CUDA 12.x found — GPU path ON
```

To force CPU-only even when CUDA OpenCV is present:

```bash
cmake -B build -DOpenCV_DIR=... -DWITH_GPU=OFF
cmake --build build -j$(nproc)
```

**Verify GPU is active at run time**

When a CUDA device is found, the tracker prints on startup:

```
[Seeker] CUDA enabled (1 device(s)) — GPU acceleration active
```

If no CUDA device is found (or the binary was built with `-DWITH_GPU=OFF`), the
CPU path is used silently.

---

### Fetch MAVLink headers

The project expects the `mavlink/c_library_v2` header-only library inside `mavlink/`.

```bash
git clone --depth=1 https://github.com/mavlink/c_library_v2.git mavlink
git apply mavlink_patch/tracking_message.patch --directory=mavlink
```

The patch adds `TRACKING_MESSAGE` (ID 11045) — the custom MAVLink message used to send normalised tracking errors to ArduPlane. It modifies one file (`ardupilotmega/ardupilotmega.h`) and adds one new header (`ardupilotmega/mavlink_msg_tracking_message.h`).

> **Dialect:** the code includes `<ardupilotmega/mavlink.h>`, which is present in `c_library_v2/ardupilotmega/`.

---

## Build

```bash
# From the drone-seeker-cpp directory
cmake -B build -DOpenCV_DIR=/usr/local/opt/opencv/lib/cmake/opencv4
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
./build/calibrate_color --source 0                          # camera index 0
./build/calibrate_color --source video.mp4                  # video file
./build/calibrate_color --source 0 --mask all               # show detection mask
./build/calibrate_color --source 0 --mask inrange           # specific algorithm
```

### Options

| Option | Default | Description |
|---|---|---|
| `--source STR` | `0` | Camera index or video file path |
| `--output FILE` | `color_histogram.txt` | Histogram output file |
| `--mask ALGO` | off | Show detection mask window. ALGO: `gaussian` \| `adaptive` \| `inrange` \| `all` |

### Workflow

1. Point the camera at the target.  If a histogram file already exists it is loaded automatically and the mask window shows live detection immediately.
2. Drag a rectangle over the pink target **or press `d`** to open the selection tool — only saturated/bright pixels are counted, background is ignored.
3. Repeat on several frames to build a robust histogram.
4. Watch the mask window (if `--mask` is given) to confirm the detection looks correct.
5. Press **`s`** to save `color_histogram.txt` in the current directory.
6. Press **`r`** to reset all samples if needed, **`q`** to quit.

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
| `--joystick [N]` | off | Enable joystick RC override; N = SDL2 device index (default 0) |

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

**Joystick RC override (device 0)**
```bash
./build/drone_seeker --connection udp:127.0.0.1:14550 --source 0 --joystick
```

**Joystick with explicit device index**
```bash
./build/drone_seeker --connection /dev/ttyUSB0 --baud 115200 --source 0 --joystick 1
```

> The joystick sends `RC_CHANNELS_OVERRIDE` to ArduPlane every frame.
> CH6 (trigger / button 4-5) acts as the tracking arm switch, identical to a
> hardware RC transmitter.  Channel mapping:
>
> | SDL axis / button | RC channel | Function |
> |---|---|---|
> | Axis 0 | CH1 | Aileron |
> | Axis 1 (inverted) | CH2 | Elevator |
> | Axis 2 | CH3 | Throttle |
> | Axis 3 | CH4 | Rudder |
> | Button 6 or 7 | CH5 | Flight mode switch |
> | Button 4/5 or trigger axis 4/5 | CH6 | Arm / tracking enable |

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
