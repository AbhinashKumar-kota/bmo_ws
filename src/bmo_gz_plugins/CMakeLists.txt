cmake_minimum_required(VERSION 3.10)
project(bmo_gz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------
# Find Gazebo Harmonic packages.
# gz-sim8   : simulation framework (System interface, ECM, components)
# gz-msgs10 : protobuf message types (gz.msgs.Double for sensor output)
# gz-transport13 : publish/subscribe transport layer
# gz-plugin2 : plugin registration macros
# -----------------------------------------------------------------------
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# -----------------------------------------------------------------------
# Build the LightIntensitySensor plugin as a shared library.
# Gazebo loads this at runtime via <plugin filename="..."> in the SDF.
# -----------------------------------------------------------------------
add_library(LightIntensitySensor SHARED
  src/light_intensity_sensor.cpp
)

target_include_directories(LightIntensitySensor PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(LightIntensitySensor PRIVATE
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-plugin2::register
)

# -----------------------------------------------------------------------
# Install the shared library so Gazebo can find it.
# The plugin is loaded by filename from GZ_SIM_SYSTEM_PLUGIN_PATH.
# -----------------------------------------------------------------------
install(TARGETS LightIntensitySensor
  LIBRARY DESTINATION lib
)
