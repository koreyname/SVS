# Helper to locate OpenCV (2.4.3+) with OpenCV 4 defaults.
if(OpenCV_FOUND)
  return()
endif()

set(_ORB_SLAM2_OPENCV_HINTS
    /usr/lib/x86_64-linux-gnu/cmake/opencv4
    /usr/local/lib/cmake/opencv4)

# Respect user-provided OpenCV_DIR; otherwise try common OpenCV 4 locations.
if(NOT DEFINED OpenCV_DIR)
  foreach(_ocv_dir ${_ORB_SLAM2_OPENCV_HINTS})
    if(EXISTS "${_ocv_dir}/OpenCVConfig.cmake")
      set(OpenCV_DIR "${_ocv_dir}")
      break()
    endif()
  endforeach()
endif()

find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND)
  find_package(OpenCV QUIET HINTS ${_ORB_SLAM2_OPENCV_HINTS})
endif()

if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV >= 2.4.3 not found. Please install OpenCV (3.x/4.x) and/or set OpenCV_DIR to the path containing OpenCVConfig.cmake, e.g. /usr/lib/x86_64-linux-gnu/cmake/opencv4.")
endif()

if(OpenCV_VERSION VERSION_LESS "2.4.3")
  message(FATAL_ERROR "OpenCV >= 2.4.3 required, but found ${OpenCV_VERSION}.")
endif()

message(STATUS "Found OpenCV ${OpenCV_VERSION}")
