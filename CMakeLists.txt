cmake_minimum_required(VERSION 3.16)
project(drone_seeker_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

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
