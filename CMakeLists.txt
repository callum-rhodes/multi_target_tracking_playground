cmake_minimum_required(VERSION 3.16)
project(sfsim CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Add "Profile" to the list of known configurations for IDEs
set(CMAKE_CONFIGURATION_TYPES "Debug;Release;MinSizeRel;RelWithDebInfo;Profile" CACHE STRING "" FORCE)

include(FetchContent)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)
  set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(eigen)
endif()

# --- yaml-cpp: scenario files ---
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
    GIT_SHALLOW TRUE)
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(yaml-cpp)
endif()
# The imported target's name differs between yaml-cpp versions.
if(TARGET yaml-cpp::yaml-cpp)
  set(SFSIM_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(SFSIM_YAML_TARGET yaml-cpp)
endif()

# Layers A-C: no rendering dependencies, so it stays testable and headless-capable
add_library(sfsim_core
  src/core/Target.cpp
  src/sensors/Sensor.cpp
  src/sensors/MeasurementModel.cpp
  src/tracking/ExtendedKalmanFilter.cpp
  src/tracking/CentralisedTracker.cpp
  src/tracking/Decentralised.cpp
  src/sim/Simulation.cpp
  src/sim/CommNetwork.cpp
  src/sim/Scenario.cpp
  src/sim/ScenarioLoader.cpp)
target_include_directories(sfsim_core PUBLIC src)
target_compile_definitions(sfsim_core PUBLIC SFSIM_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/scenarios")
target_link_libraries(sfsim_core PUBLIC Eigen3::Eigen PRIVATE ${SFSIM_YAML_TARGET})

add_executable(sfsim_headless app/main_headless.cpp)
target_link_libraries(sfsim_headless PRIVATE sfsim_core)

# Layer D: SDL2 + OpenGL3 + Dear ImGui front end
option(SFSIM_GUI "Build the SDL2 + Dear ImGui front end" ON)
if(SFSIM_GUI)
  find_package(SDL2 REQUIRED)
  find_package(OpenGL REQUIRED)

  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG docking
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(imgui)   # imgui has no CMakeLists; this just downloads it

  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
  target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
  if(TARGET SDL2::SDL2)
    target_link_libraries(imgui PUBLIC SDL2::SDL2)
  else()
    target_include_directories(imgui PUBLIC ${SDL2_INCLUDE_DIRS})
    target_link_libraries(imgui PUBLIC ${SDL2_LIBRARIES})
  endif()
  target_link_libraries(imgui PUBLIC OpenGL::GL ${CMAKE_DL_LIBS})

  add_executable(sfsim_gui app/main_gui.cpp)
  target_link_libraries(sfsim_gui PRIVATE sfsim_core imgui)
  if(TARGET SDL2::SDL2main)
    target_link_libraries(sfsim_gui PRIVATE SDL2::SDL2main)
  endif()
endif()

# ================= Profiling support =====================
# Start with the core headless targets that always exist
set(ALL_TARGETS sfsim_core sfsim_headless)

# Only add the GUI target to the list if the option is enabled
if(SFSIM_GUI)
    list(APPEND ALL_TARGETS sfsim_gui)
endif()

foreach(pkg_target sfsim_core sfsim_headless)
    target_compile_options(${pkg_target} PRIVATE $<$<CONFIG:Profile>:-O2 -g -pg>)
    target_link_options(${pkg_target} PRIVATE $<$<CONFIG:Profile>:-pg>)
endforeach()

if(SFSIM_GUI)
    target_compile_options(sfsim_gui PRIVATE $<$<CONFIG:Profile>:-O2 -g -pg>)
    target_link_options(sfsim_gui PRIVATE $<$<CONFIG:Profile>:-pg>)
endif()
