cmake_minimum_required(VERSION 3.16)

project(rm_aim_auto_aim_bridge LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "auto_aim_bridge requires Linux local IPC under /dev/shm")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build configuration" FORCE)
endif()

option(
  AUTOAIM_USE_REDUCED_DOF_PNP
  "Refine the selected IPPE pose with constrained translation and yaw"
  OFF
)

set(
  AUTOAIM_ROOT
  ""
  CACHE PATH
  "Absolute path to the auto-aim-new-0 source checkout"
)

if(NOT AUTOAIM_ROOT OR NOT IS_ABSOLUTE "${AUTOAIM_ROOT}")
  message(FATAL_ERROR "Pass -DAUTOAIM_ROOT=/absolute/path/to/auto-aim-new-0")
endif()

get_filename_component(AUTOAIM_SOURCE_ROOT "${AUTOAIM_ROOT}" REALPATH)

set(AUTOAIM_ALGORITHM_SOURCES
  tasks/auto_aim/armor.cpp
  tasks/auto_aim/solver.cpp
  tasks/auto_aim/pnp_reprojection.cpp
  tasks/auto_aim/reduced_dof_pnp_solver.cpp
  tasks/auto_aim/ba_solver.cpp
  tasks/auto_aim/target.cpp
  tasks/auto_aim/tracker.cpp
  tasks/auto_aim/planner/planner.cpp
  tools/extended_kalman_filter.cpp
  tools/math_tools.cpp
  tools/trajectory.cpp
  tools/logger.cpp
)

set(AUTOAIM_DETECTOR_SOURCES
  tasks/auto_aim/classifier.cpp
  tasks/auto_aim/detector.cpp
  tasks/auto_aim/yolo.cpp
  tasks/auto_aim/yolos/yolov5.cpp
  tasks/auto_aim/yolos/yolov8.cpp
  tasks/auto_aim/yolos/yolo11.cpp
  tools/img_tools.cpp
)

set(AUTOAIM_TINYMPC_SOURCES
  tasks/auto_aim/planner/tinympc/admm.cpp
  tasks/auto_aim/planner/tinympc/tiny_api.cpp
  tasks/auto_aim/planner/tinympc/codegen.cpp
  tasks/auto_aim/planner/tinympc/rho_benchmark.cpp
)

foreach(
  SOURCE_PATH
  IN LISTS
    AUTOAIM_ALGORITHM_SOURCES
    AUTOAIM_DETECTOR_SOURCES
    AUTOAIM_TINYMPC_SOURCES
)
  if(NOT EXISTS "${AUTOAIM_SOURCE_ROOT}/${SOURCE_PATH}")
    message(FATAL_ERROR "AUTOAIM_ROOT is missing ${SOURCE_PATH}")
  endif()
endforeach()

list(TRANSFORM AUTOAIM_ALGORITHM_SOURCES PREPEND "${AUTOAIM_SOURCE_ROOT}/")
list(TRANSFORM AUTOAIM_DETECTOR_SOURCES PREPEND "${AUTOAIM_SOURCE_ROOT}/")
list(TRANSFORM AUTOAIM_TINYMPC_SOURCES PREPEND "${AUTOAIM_SOURCE_ROOT}/")

if(NOT OpenVINO_DIR AND
   EXISTS "/opt/intel/openvino_2024.6.0/runtime/cmake/OpenVINOConfig.cmake")
  set(
    OpenVINO_DIR
    "/opt/intel/openvino_2024.6.0/runtime/cmake"
    CACHE PATH
    "Directory containing the OpenVINO CMake package"
  )
endif()

find_package(OpenCV 4 REQUIRED COMPONENTS
  core
  calib3d
  dnn
  highgui
  imgcodecs
  imgproc
)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${AUTOAIM_SOURCE_ROOT}/cmake")
find_package(g2o REQUIRED)

# Discover the header-only JSON dependency through pkg-config as well. A Conda
# nlohmann_json CMake package exports its entire include prefix, which can put a
# different fmt version ahead of the system headers selected for spdlog.
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOHMANN_JSON REQUIRED IMPORTED_TARGET nlohmann_json>=3.2)
pkg_check_modules(FMT REQUIRED IMPORTED_TARGET fmt)
pkg_check_modules(SPDLOG REQUIRED IMPORTED_TARGET spdlog)

add_library(autoaim_tinympc STATIC ${AUTOAIM_TINYMPC_SOURCES})

target_include_directories(autoaim_tinympc PUBLIC
  "${AUTOAIM_SOURCE_ROOT}/tasks/auto_aim/planner"
  "${AUTOAIM_SOURCE_ROOT}/tasks/auto_aim/planner/tinympc"
)

target_link_libraries(autoaim_tinympc PUBLIC Eigen3::Eigen)

# Compile the synchronous image detector and the PnP, tracker and planner
# dependencies from the sibling tree. Camera drivers, serial/CAN and hardware
# entry points remain outside this target.
add_library(autoaim_algorithms STATIC
  ${AUTOAIM_ALGORITHM_SOURCES}
  ${AUTOAIM_DETECTOR_SOURCES}
)

target_compile_definitions(autoaim_algorithms PUBLIC
  AUTOAIM_USE_REDUCED_DOF_PNP=$<BOOL:${AUTOAIM_USE_REDUCED_DOF_PNP}>
  AUTOAIM_ENABLE_UV_EKF=0
)

target_include_directories(autoaim_algorithms PUBLIC "${AUTOAIM_SOURCE_ROOT}")
target_include_directories(autoaim_algorithms SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(autoaim_algorithms PUBLIC
  autoaim_tinympc
  Eigen3::Eigen
  PkgConfig::FMT
  PkgConfig::SPDLOG
  yaml-cpp
  openvino::runtime
  g2o::core
  g2o::types_slam3d
  g2o::solver_dense
  ${OpenCV_LIBS}
)

# Keep this transitive g2o dependency in the executable's direct runtime search
# path, including with the linker's default --as-needed behavior.
target_link_libraries(autoaim_algorithms INTERFACE
  "-Wl,--push-state,--no-as-needed"
  g2o::opengl_helper
  "-Wl,--pop-state"
)

add_library(bridge_core STATIC
  src/sim_adapter.cpp
  src/sim_client.cpp
)

target_include_directories(bridge_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_definitions(bridge_core PUBLIC
  BRIDGE_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/default.yaml"
  BRIDGE_AUTOAIM_ROOT="${AUTOAIM_SOURCE_ROOT}"
)

target_compile_options(bridge_core PRIVATE -Wall -Wextra -Wpedantic)

find_package(Git QUIET)

function(record_source_revision REPOSITORY_NAME REPOSITORY_PATH)
  set(SOURCE_COMMIT "unknown")
  set(SOURCE_DIRTY 1)

  if(GIT_FOUND)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${REPOSITORY_PATH}" rev-parse HEAD
      OUTPUT_VARIABLE COMMIT_OUTPUT
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE COMMIT_STATUS
    )

    if(COMMIT_STATUS EQUAL 0)
      set(SOURCE_COMMIT "${COMMIT_OUTPUT}")
    endif()

    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${REPOSITORY_PATH}" status --porcelain
      OUTPUT_VARIABLE WORKTREE_STATUS
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE STATUS_RESULT
    )

    if(STATUS_RESULT EQUAL 0 AND NOT WORKTREE_STATUS)
      set(SOURCE_DIRTY 0)
    endif()
  endif()

  target_compile_definitions(bridge_core PUBLIC
    BRIDGE_${REPOSITORY_NAME}_COMMIT="${SOURCE_COMMIT}"
    BRIDGE_${REPOSITORY_NAME}_DIRTY=${SOURCE_DIRTY}
  )
endfunction()

get_filename_component(SIM_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." REALPATH)
record_source_revision(SIM "${SIM_SOURCE_ROOT}")
record_source_revision(AUTOAIM "${AUTOAIM_SOURCE_ROOT}")

target_link_libraries(bridge_core PUBLIC
  autoaim_algorithms
  PkgConfig::NLOHMANN_JSON
  Threads::Threads
)

add_executable(auto_aim_bridge src/main.cpp)
target_link_libraries(auto_aim_bridge PRIVATE bridge_core)
target_compile_options(auto_aim_bridge PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)

if(BUILD_TESTING)
  add_executable(bridge_test tests/bridge_test.cpp)
  target_link_libraries(bridge_test PRIVATE bridge_core)
  target_compile_options(bridge_test PRIVATE -Wall -Wextra -Wpedantic)

  add_test(NAME bridge_test COMMAND bridge_test)
  set_tests_properties(bridge_test PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 30
  )
endif()

message(STATUS "Auto-aim sources: ${AUTOAIM_SOURCE_ROOT}")
message(STATUS "Reduced-DoF PnP: ${AUTOAIM_USE_REDUCED_DOF_PNP}")
