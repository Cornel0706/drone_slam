cmake_minimum_required(VERSION 3.8)
project(drone_sim)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories (launch, config, worlds, models)
install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/drone_teleop.py
  scripts/auto_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
