cmake_minimum_required(VERSION 3.16)
project(make_multi LANGUAGES C CXX)

set(BUILD_SCRIPT "${CMAKE_SOURCE_DIR}/build_mjpeg_rknn.sh")
if(NOT EXISTS "${BUILD_SCRIPT}")
  message(FATAL_ERROR "Build script not found: ${BUILD_SCRIPT}")
endif()

set(BUILD_STAMP "${CMAKE_BINARY_DIR}/mjpeg_rknn_http.stamp")

add_custom_command(
  OUTPUT "${BUILD_STAMP}"
  COMMAND "${CMAKE_COMMAND}" -E env BUILD_DIR="${CMAKE_BINARY_DIR}" "${BUILD_SCRIPT}"
  COMMAND "${CMAKE_COMMAND}" -E touch "${BUILD_STAMP}"
  DEPENDS "${BUILD_SCRIPT}"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Building mjpeg_rknn_http via build_mjpeg_rknn.sh"
  VERBATIM
)

add_custom_target(mjpeg_rknn_http ALL DEPENDS "${BUILD_STAMP}")
