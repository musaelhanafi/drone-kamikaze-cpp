cmake_minimum_required(VERSION 3.16)
project(drone_seeker_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Optional joystick support via SDL2 (auto-detected; disable with -DWITH_JOYSTICK=OFF)
option(WITH_JOYSTICK "Enable joystick RC override via SDL2" ON)
if(WITH_JOYSTICK)
    find_package(SDL2 QUIET)
    if(SDL2_FOUND)
        message(STATUS "[drone_seeker] SDL2 ${SDL2_VERSION} found — joystick support ON")
    else()
        message(STATUS "[drone_seeker] SDL2 not found — joystick support OFF (install libsdl2-dev)")
        set(WITH_JOYSTICK OFF)
    endif()
endif()

# MAVLink C headers (ardupilotmega dialect)
# Fetch with: git clone --depth=1 https://github.com/mavlink/c_library_v2.git mavlink
if(NOT DEFINED MAVLINK_INCLUDE_DIR)
    set(MAVLINK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/mavlink" CACHE PATH
        "Path to mavlink/c_library_v2 (contains ardupilotmega/)")
endif()

add_executable(drone_seeker
    src/main.cpp
    src/Seeker.cpp
    src/SeekerCtrl.cpp
    src/HudDisplay.cpp
    src/MavlinkConn.cpp
    src/JoystickHandler.cpp
)

target_include_directories(drone_seeker PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${MAVLINK_INCLUDE_DIR}
)

target_link_libraries(drone_seeker PRIVATE
    ${OpenCV_LIBS}
    Threads::Threads
)

# Optional GPU acceleration via OpenCV CUDA (auto-detected; disable with -DWITH_GPU=OFF)
option(WITH_GPU "Enable GPU acceleration via OpenCV CUDA" ON)
if(WITH_JOYSTICK AND SDL2_FOUND)
    target_compile_definitions(drone_seeker PRIVATE DRONE_USE_JOYSTICK)
    target_include_directories(drone_seeker PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(drone_seeker PRIVATE ${SDL2_LIBRARIES})
endif()

if(WITH_GPU AND OpenCV_CUDA_VERSION)
    message(STATUS "[drone_seeker] OpenCV CUDA ${OpenCV_CUDA_VERSION} found — GPU path ON")
    target_compile_definitions(drone_seeker PRIVATE DRONE_USE_CUDA)
else()
    message(STATUS "[drone_seeker] OpenCV CUDA not available — CPU-only build")
endif()

# Optional appearance trackers via opencv_tracking (disable with -DWITH_TRACKING=OFF)
option(WITH_TRACKING "Enable appearance trackers: mil, csrt, dasiamrpn, nano, vit (requires opencv_tracking)" ON)
if(WITH_TRACKING)
    find_package(OpenCV COMPONENTS tracking QUIET)
    if(OpenCV_tracking_FOUND)
        message(STATUS "[drone_seeker] opencv_tracking found — appearance trackers ON")
        target_compile_definitions(drone_seeker PRIVATE DRONE_USE_TRACKING)
        target_link_libraries(drone_seeker PRIVATE opencv_tracking)
    else()
        message(STATUS "[drone_seeker] opencv_tracking not found — appearance trackers OFF (install opencv_contrib)")
    endif()
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(drone_seeker PRIVATE rt)
endif()

# Calibration tool (standalone)
add_executable(calibrate_color
    src/calibrate_color.cpp
)
target_include_directories(calibrate_color PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(calibrate_color PRIVATE
    ${OpenCV_LIBS}
)

# Camera recording test (standalone)
add_executable(test_record_camera
    src/test_record_camera.cpp
)
target_include_directories(test_record_camera PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_record_camera PRIVATE
    ${OpenCV_LIBS}
)

# Joystick ch6 toggle test (requires SDL2)
if(WITH_JOYSTICK AND SDL2_FOUND)
    add_executable(test_joystick
        src/test_joystick.cpp
        src/MavlinkConn.cpp
    )
    target_include_directories(test_joystick PRIVATE
        include
        ${MAVLINK_INCLUDE_DIR}
        ${SDL2_INCLUDE_DIRS}
    )
    target_link_libraries(test_joystick PRIVATE
        ${SDL2_LIBRARIES}
        Threads::Threads
    )
    if(UNIX AND NOT APPLE)
        target_link_libraries(test_joystick PRIVATE rt)
    endif()
endif()
