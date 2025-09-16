cmake_minimum_required(VERSION 3.10)
project(rknn_yolov8_demo_src_2)


add_compile_options(-Wno-psabi)

set(PROJ ${CMAKE_CURRENT_SOURCE_DIR})

# ---------- RGA (headers + lib) ----------
set(RGA_INC_DIR  ${PROJ}/include/3rdparty/librga/include)
# Выбираем подходящую папку с готовой библиотекой
if(EXISTS "${PROJ}/include/3rdparty/librga/Linux/aarch64/librga.a")
  set(RGA_LIB_DIR ${PROJ}/include/3rdparty/librga/Linux/aarch64)
elseif(EXISTS "${PROJ}/include/3rdparty/librga/Linux/armhf/librga.a")
  set(RGA_LIB_DIR ${PROJ}/include/3rdparty/librga/Linux/armhf)
elseif(EXISTS "${PROJ}/include/3rdparty/librga/Linux/aarch64/librga.so")
  set(RGA_LIB_DIR ${PROJ}/include/3rdparty/librga/Linux/aarch64)
  set(RGA_SHARED 1)
elseif(EXISTS "${PROJ}/include/3rdparty/librga/Linux/armhf/librga.so")
  set(RGA_LIB_DIR ${PROJ}/include/3rdparty/librga/Linux/armhf)
  set(RGA_SHARED 1)
else()
  message(FATAL_ERROR "librga not found under include/3rdparty/librga/Linux/*")
endif()

# импортируем librga
if(RGA_SHARED)
  add_library(rga SHARED IMPORTED)
  set_target_properties(rga PROPERTIES IMPORTED_LOCATION ${RGA_LIB_DIR}/librga.so)
else()
  add_library(rga STATIC IMPORTED)
  set_target_properties(rga PROPERTIES IMPORTED_LOCATION ${RGA_LIB_DIR}/librga.a)
endif()
target_include_directories(rga INTERFACE ${RGA_INC_DIR})

# ---------- Глобальные include'ы ----------
include_directories(
  ${PROJ}
  ${PROJ}/include
  ${PROJ}/rknpu1
  ${PROJ}/rknpu2
  ${PROJ}/include/3rdparty
  ${PROJ}/include/utils
  ${RGA_INC_DIR}                  # <- ВАЖНО для im2d.h
)

# ---------- rknnrt ----------
set(LIBRKNNRT_PATH "${PROJ}/lib/librknnrt.so")
set(LIBRKNNRT ${LIBRKNNRT_PATH})
set(LIBRKNNRT_INCLUDES "${PROJ}/include")

# ---------- Threads / OpenCV ----------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d video)

# ---------- TurboJPEG (system or bundled) ----------
set(TJPEG_BUNDLED_ROOT ${PROJ}/include/3rdparty/jpeg_turbo)

# Определяем архитектуру для бандла
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(_TJ_ARCH aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|armv7")
  set(_TJ_ARCH armhf)
else()
  set(_TJ_ARCH x64)
endif()

# Пробуем системную библиотеку (если стоит libturbojpeg0-dev)
find_library(TURBOJPEG_LIB NAMES turbojpeg PATHS /usr/lib /usr/local/lib)

# Если не нашли — берём статическую из include/3rdparty
if(NOT TURBOJPEG_LIB)
  set(TURBOJPEG_LIB ${TJPEG_BUNDLED_ROOT}/Linux/${_TJ_ARCH}/libturbojpeg.a)
endif()

if(NOT EXISTS ${TURBOJPEG_LIB})
  message(FATAL_ERROR "TurboJPEG not found. Установи libturbojpeg0-dev ИЛИ положи ${TURBOJPEG_LIB}")
endif()

# ---------- nlohmann/json ----------
find_package(PkgConfig)
if(PkgConfig_FOUND)
    pkg_check_modules(NLOHMANN_JSON nlohmann_json)
endif()

if(NOT NLOHMANN_JSON_FOUND)
    # Пытаемся найти через find_package
    find_package(nlohmann_json QUIET)
    if(nlohmann_json_FOUND)
        set(NLOHMANN_JSON_LIBRARIES nlohmann_json::nlohmann_json)
    else()
        # Используем header-only версию если есть
        find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
            PATHS /usr/include /usr/local/include ${PROJ}/include/3rdparty)
        if(NLOHMANN_JSON_INCLUDE_DIR)
            set(NLOHMANN_JSON_LIBRARIES "")
            include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
        else()
            message(WARNING "nlohmann/json not found. Please install nlohmann-json3-dev or place headers in include/3rdparty/")
        endif()
    endif()
endif()

# ---------- FFmpeg (timestamps for calibration) ----------
if(NOT PkgConfig_FOUND)
  find_package(PkgConfig REQUIRED)
endif()
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)

set(FFMPEG_INCLUDE_DIRS ${AVFORMAT_INCLUDE_DIRS} ${AVCODEC_INCLUDE_DIRS} ${AVUTIL_INCLUDE_DIRS})
list(REMOVE_DUPLICATES FFMPEG_INCLUDE_DIRS)
set(FFMPEG_LIBS ${AVFORMAT_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVUTIL_LIBRARIES})


# ---------- utils (перенесены в include/utils) ----------
add_library(imagedrawing STATIC
  ${PROJ}/include/utils/image_drawing.c
)
add_library(imageutils STATIC
  ${PROJ}/include/utils/image_utils.c
)
add_library(fileutils STATIC
  ${PROJ}/include/utils/file_utils.c
)

target_include_directories(imagedrawing PUBLIC ${PROJ}/include)
# image_utils.c тянет im2d.h -> добавим RGA_INC_DIR
target_include_directories(imageutils   PUBLIC ${PROJ}/include ${RGA_INC_DIR})
target_include_directories(fileutils    PUBLIC ${PROJ}/include)

# ---------- Калибровка ----------
add_library(calibration_watcher STATIC
  ${PROJ}/src/calibration_watcher.cpp
)
target_include_directories(calibration_watcher PUBLIC 
  ${PROJ}/include
  ${PROJ}/src
  ${OpenCV_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
)
target_link_libraries(calibration_watcher PUBLIC ${OpenCV_LIBS} ${FFMPEG_LIBS})

# ---------- выбор исходника для rknpu2 ----------
if(NOT DEFINED rknpu_yolov8_file)
  if(TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)
    set(rknpu_yolov8_file rknpu2/yolov8_rv1106_1103.cc)
  elseif(TARGET_SOC STREQUAL "zero_copy")
    set(rknpu_yolov8_file rknpu2/yolov8_zero_copy.cc)
  else()
    set(rknpu_yolov8_file rknpu2/yolov8.cc)
  endif()
endif()

# ---------- основная демка ----------
add_executable(${PROJECT_NAME}
  main.cc
  postprocess.cc
  ${rknpu_yolov8_file}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${PROJ}
  ${PROJ}/include
  ${PROJ}/include/utils
  ${PROJ}/rknpu2
  ${LIBRKNNRT_INCLUDES}
  ${RGA_INC_DIR}
)

target_link_libraries(${PROJECT_NAME}
  imageutils
  fileutils
  imagedrawing
  rga
  ${LIBRKNNRT}
  ${TURBOJPEG_LIB}
  Threads::Threads
  dl
)

# ---------- веб-сервер ----------
option(BUILD_WEB_SERVER "Build HTTP web server for YOLOv8" ON)

if(BUILD_WEB_SERVER)
    add_executable(yolov8_web_server
      yolov8_web_server.cc
      camera_manager.cpp
      calibration/session.cpp
      src/calibration_watcher.cpp
      src/camera_roles.cpp
      src/camera_scheme.cpp
      src/global_tracker.cpp
      postprocess.cc
      ${rknpu_yolov8_file}
    )

    add_executable(yolov8_web_server_calibration
      yolov8_web_server_calibration.cc
      camera_manager.cpp
      calibration/session.cpp
      src/calibration_watcher.cpp
      src/camera_roles.cpp
      src/camera_scheme.cpp
      src/global_tracker.cpp
      postprocess.cc
      ${rknpu_yolov8_file}
    )

    target_include_directories(yolov8_web_server PRIVATE
      ${PROJ}
      ${PROJ}/include
      ${PROJ}/src
      ${PROJ}/rknpu2
      ${LIBRKNNRT_INCLUDES}
      ${RGA_INC_DIR}
      ${OpenCV_INCLUDE_DIRS}
      ${FFMPEG_INCLUDE_DIRS}
    )

    target_include_directories(yolov8_web_server_calibration PRIVATE
      ${PROJ}
      ${PROJ}/include
      ${PROJ}/src
      ${PROJ}/rknpu2
      ${LIBRKNNRT_INCLUDES}
      ${RGA_INC_DIR}
      ${OpenCV_INCLUDE_DIRS}
      ${FFMPEG_INCLUDE_DIRS}
    )

    target_link_libraries(yolov8_web_server
      imageutils
      fileutils
      imagedrawing
      calibration_watcher
      rga
      ${LIBRKNNRT}
      ${TURBOJPEG_LIB}
      ${NLOHMANN_JSON_LIBRARIES}
      Threads::Threads
      dl
      ${OpenCV_LIBS}
    )

    target_link_libraries(yolov8_web_server_calibration
      imageutils
      fileutils
      imagedrawing
      calibration_watcher
      rga
      ${LIBRKNNRT}
      ${TURBOJPEG_LIB}
      ${NLOHMANN_JSON_LIBRARIES}
      Threads::Threads
      dl
      ${OpenCV_LIBS}
    )

    set_target_properties(yolov8_web_server_calibration PROPERTIES
      INSTALL_RPATH "$ORIGIN/../lib"
    )

    install(TARGETS yolov8_web_server yolov8_web_server_calibration DESTINATION .)
    install(DIRECTORY ${PROJ}/web DESTINATION .)
endif()

# ---------- install / rpath ----------
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS ${PROJECT_NAME} DESTINATION .)

# ---------- camera manager demo ----------
add_executable(camera_manager_demo
  camera_manager_main.cc
  camera_manager.cpp
  calibration/session.cpp
  src/camera_roles.cpp
  src/camera_scheme.cpp
  src/global_tracker.cpp
)

target_include_directories(camera_manager_demo PRIVATE
  ${PROJ}
  ${PROJ}/include
  ${PROJ}/src
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_manager_demo
  calibration_watcher
  ${NLOHMANN_JSON_LIBRARIES}
  Threads::Threads
  ${OpenCV_LIBS}
)

install(TARGETS camera_manager_demo DESTINATION .)

# ---------- calibration CLI ----------
add_executable(calibration_cli
  src/tools/calibration_cli.cpp
  camera_manager.cpp
  calibration/session.cpp
  src/calibration_watcher.cpp
  src/camera_roles.cpp
)

target_include_directories(calibration_cli PRIVATE
  ${PROJ}
  ${PROJ}/src
  ${OpenCV_INCLUDE_DIRS}
  ${FFMPEG_INCLUDE_DIRS}
)

target_link_libraries(calibration_cli
  calibration_watcher
  Threads::Threads
  ${OpenCV_LIBS}
  ${NLOHMANN_JSON_LIBRARIES}
  pthread
  calibration_watcher
)

install(TARGETS calibration_cli DESTINATION .)