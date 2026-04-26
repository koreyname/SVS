# Install script for directory: /home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake"
         "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/CMakeFiles/Export/share/sophus/cmake/SophusTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/sophus/cmake/SophusTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sophus/cmake" TYPE FILE FILES "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/CMakeFiles/Export/share/sophus/cmake/SophusTargets.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sophus/cmake" TYPE FILE FILES
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/SophusConfig.cmake"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/SophusConfigVersion.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/sophus" TYPE FILE FILES
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/average.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/common.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/geometry.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/interpolate.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/interpolate_details.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/num_diff.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/rotation_matrix.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/rxso2.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/rxso3.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/se2.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/se3.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/sim2.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/sim3.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/sim_details.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/so2.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/so3.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/types.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/velocities.hpp"
    "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/sophus/formatstring.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/test/cmake_install.cmake")
  include("/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/korey/Desktop/my_work/src/ORBSLAM3/Thirdparty/Sophus/build_orbs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
