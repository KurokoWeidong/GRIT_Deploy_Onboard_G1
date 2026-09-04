cmake_minimum_required(VERSION 3.16)
project(g1_sim2real VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build bundled unitree_sdk2 examples" FORCE)
add_subdirectory(third_party/unitree_sdk2)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(ZLIB_BUILD_SHARED OFF CACHE BOOL "Do not build shared zlib" FORCE)
set(ZLIB_BUILD_STATIC ON CACHE BOOL "Build static zlib" FORCE)
set(ZLIB_BUILD_TESTING OFF CACHE BOOL "Do not build zlib tests" FORCE)
set(ZLIB_INSTALL OFF CACHE BOOL "Do not install vendored zlib" FORCE)
add_subdirectory(third_party/zlib EXCLUDE_FROM_ALL)

set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "Build yaml-cpp as a static library" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "Do not build yaml-cpp contrib sources" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Do not build yaml-cpp tests" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Do not build yaml-cpp tools" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "Do not install vendored yaml-cpp" FORCE)
add_subdirectory(third_party/yaml-cpp EXCLUDE_FROM_ALL)

find_package(Threads REQUIRED)
find_path(ZMQ_INCLUDE_DIR NAMES zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)

add_executable(g1_udp_bridge
  src/g1_udp_bridge.cpp
)
set_target_properties(g1_udp_bridge PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

target_compile_features(g1_udp_bridge PRIVATE cxx_std_17)
target_compile_options(g1_udp_bridge PRIVATE -Wall -Wextra -Wpedantic -O3)
target_link_libraries(g1_udp_bridge
  PRIVATE
    unitree_sdk2
    yaml-cpp::yaml-cpp
    ZLIB::ZLIBSTATIC
    Threads::Threads
)

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime C/C++ installation prefix")
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT} $ENV{ONNXRUNTIME_ROOT} /opt/onnxruntime /usr/local
  PATH_SUFFIXES include include/onnxruntime/core/session
)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS ${ONNXRUNTIME_ROOT} $ENV{ONNXRUNTIME_ROOT} /opt/onnxruntime /usr/local
  PATH_SUFFIXES lib lib64
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  add_executable(g1_npz_policy
    src/g1_npz_policy.cpp
    src/npz_reader.cpp
    src/vr_motion_client.cpp
  )
  target_include_directories(g1_npz_policy PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_compile_features(g1_npz_policy PRIVATE cxx_std_17)
  target_compile_options(g1_npz_policy PRIVATE -Wall -Wextra -Wpedantic -O3)
  target_link_libraries(g1_npz_policy
    PRIVATE
      yaml-cpp::yaml-cpp
      ZLIB::ZLIBSTATIC
      Threads::Threads
      ${ONNXRUNTIME_LIBRARY}
  )
  if(ZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
    target_include_directories(g1_npz_policy PRIVATE ${ZMQ_INCLUDE_DIR})
    target_link_libraries(g1_npz_policy PRIVATE ${ZMQ_LIBRARY})
    target_compile_definitions(g1_npz_policy PRIVATE GRIT_HAVE_ZMQ=1)
  else()
    message(WARNING "ZeroMQ was not found; NPZ mode remains available but VR mode will be disabled.")
  endif()
  set_target_properties(g1_npz_policy PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib;$ENV{ONNXRUNTIME_ROOT}/lib"
  )
else()
  message(STATUS
    "ONNX Runtime C++ was not found; g1_npz_policy is disabled. "
    "Set ONNXRUNTIME_ROOT to enable the onboard policy target."
  )
endif()

install(TARGETS g1_udp_bridge DESTINATION bin)
if(TARGET g1_npz_policy)
  install(TARGETS g1_npz_policy DESTINATION bin)
endif()
install(DIRECTORY config DESTINATION .)
install(DIRECTORY scripts DESTINATION . USE_SOURCE_PERMISSIONS)
