cmake_minimum_required(VERSION 3.28)

# Note: Vcpkg should be off on Linux

option(USE_VCPKG "Use Vcpkg" ON)
if (USE_VCPKG)
    set(VCPKG_ROOT "D:/AppData/vcpkg")
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "Vcpkg toolchain file")
    message("Vcpkg loaded from: ${CMAKE_TOOLCHAIN_FILE}")
    message("If you want to disable vcpkg, use `-DUSE_VCPKG=OFF`")
else ()
    message("Vcpkg is set to disabled.")
endif ()

# Load
message("Loading project with cpp standard 17")
project(ArduCV)

set(CMAKE_CXX_STANDARD 17)

message("Finding OpenCV")
find_package(OpenCV 4.8.0 CONFIG REQUIRED)
if (OpenCV_FOUND)
    message("OpenCV founded")
    message("OpenCV libs: ${OpenCV_LIBS}")
else ()
    message("OpenCV not found, make sure your version newer than 4.8.0")
endif ()

message("Finding Boost")
find_package(Boost 1.84.0 COMPONENTS json)
if (Boost_FOUND)
    message("Boost founded")
    message("Boost libs: ${Boost_LIBRARIES}")
else ()
    message("Boost not found, make sure your version is 1.84.0")
endif ()

message("Loading mavlink")
include_directories(3rdParty/mavlink_header)

add_executable(${PROJECT_NAME} main.cpp
        calibration/Calibration.cpp
        calibration/Calibration.h
        types/Types.h
        tools/Tools.h
        tools/Tools.cpp
        param/CameraParam.cpp
        param/CameraParam.h
        param/Settings.cpp
        param/Settings.h
        reconstruction/Reconstruction.cpp
        reconstruction/Reconstruction.h
        Reconstruction/DistanceDetection.cpp
        Reconstruction/DistanceDetection.h
        Reconstruction/StereoRectify.cpp
        Reconstruction/StereoRectify.h
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES})
