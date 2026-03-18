cmake_minimum_required(VERSION 3.10)
project(bmo_sim)

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------------
# Install all data files (worlds, models, config, rviz, launch).
# These are pure data — no compilation needed.
# -----------------------------------------------------------------------
install(DIRECTORY
  worlds
  models
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
