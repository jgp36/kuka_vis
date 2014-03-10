cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )

find_package(Orocos-RTT REQUIRED ${RTT_HINTS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
if(RTNET_INCLUDE_DIR AND ${OROCOS_TARGET} STREQUAL "xenomai" )
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  include_directories(/usr/include/xenomai
		      /usr/include/xenomai/rtdm)
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=rtdm --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=rtdm --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  orocos_component(rt_udp_client src/rt_udp_client_component.cpp) # ...you may add multiple source files
  orocos_add_compile_flags(rt_udp_client ${XENO_CFLAGS})
  orocos_add_link_flags(rt_udp_client ${XENO_LDFLAGS})
  find_library(XENO_native native /usr/lib/xenomai)
  find_library(XENO_rtdm rtdm /usr/lib/xenomai)
  find_library(XENO_xenomai xenomai /usr/lib/xenomai)
  target_link_libraries(rt_udp_client ${XENO_rtdm} ${XENO_native} ${XENO_xenomai})
endif()


rosbuild_add_executable(kuka_vis src/kuka_vis.cpp)
rosbuild_add_executable(proxy_vis src/proxy_vis.cpp)
rosbuild_add_executable(udp_server src/udp_server.cpp)
