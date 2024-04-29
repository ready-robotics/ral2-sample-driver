cmake_minimum_required(VERSION 3.8.2)
project(ral2-sample-driver)

# Documentation
if(CMAKE_BUILD_DOCS)
  add_subdirectory(docs)
  return()
endif()

# Forge/READY Robotics CMake library dependencies
set(RR_CATKIN_DEPS
  device-abstraction-layer
  device-tools
  package-manager-utils
  robot-abstraction-layer
  robot-abstraction-layer-utils
  forge-core-logger-cpp
  forge-core-storage
  forge-core-common
  shared-library-cpp
  ral-messages
  ral2-driver
  robot-configuration
)

# Find all CMake packages
find_package(catkin REQUIRED
  COMPONENTS
  ${RR_CATKIN_DEPS}
  device-tools
  package-manager-utils
  robot-configuration
)

forge_setup_cmake()
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Network REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Build 
include_directories(src)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(ral2-sample-driver SHARED)

forge_target_compile_options(
  ral2-sample-driver
  PRIVATE -Weffc++
  PUBLIC -fPIC
)

target_compile_definitions(
  ral2-sample-driver
  PRIVATE
    PACKAGE_NAME="ready-ral2-sample-driver"
)

target_sources(
  ral2-sample-driver
  PRIVATE
  src/SampleDriver.cpp
  src/SampleDriver.hpp
  src/DriverLib.cpp
)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

target_link_libraries(
  ral2-sample-driver
  ${catkin_LIBRARIES}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Network
)

set(URDF_DIRECTORIES
  "READY-5.0"
)
list(TRANSFORM URDF_DIRECTORIES APPEND ".json" OUTPUT_VARIABLE ROBOT_MODEL_FILES)
validate_robot_configurations(ROBOT_MODELS ${ROBOT_MODEL_FILES})

SET(CONTROLLER_FILES
  "RCU/Program Mode.json"
)
validate_controller_configurations(CONTROLLER_MODELS ${CONTROLLER_FILES})

device_translations(
  ready-ral2-sample-driver
  translation-files
  QM_FILE_DIR
  DEVICE_FILES
    configurations/Sample.json
)

# -------------------------------------------------------------------------------------
# Uncomment the following lines to install and test the driver implementation in Forge
# -------------------------------------------------------------------------------------
#
# # Install the package
# catkin_package(
#   LIBRARIES
#     ral2-sample-driver
# )
#
# # Install Robot models
# install_robot_models(URDF_DIRS ${URDF_DIRECTORIES})
#
# # Install the driver implementation
# install(
#   TARGETS ral2-sample-driver
#   LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
# )
#
# # Install the controller models ahead of the install package
# install(
#   DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/configurations/controller-models
#   DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/configurations
# )
#
# install_package(
#   ready-ral2-sample-driver
#   manifest.json
#   DIRS
#     ${CMAKE_CURRENT_BINARY_DIR}/configurations
#   VENDOR_ICON
#     READY.svg
#   VENDOR_FILES
#     configurations/Sample.json
#   LOCALE_DIR
#     ${QM_FILE_DIR}
# )
