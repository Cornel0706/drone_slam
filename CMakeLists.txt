cmake_minimum_required(VERSION 3.8)
project(drone_slam)

# ── Compilator: activare warnings standard ──────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependente ROS2 ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Instalare resurse (launch, config, urdf, worlds, scripts) ────────
install(DIRECTORY
  launch
  config
  urdf
  worlds
  scripts
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── Instalare scripturi Python ca executabile ────────────────────────
install(PROGRAMS
  scripts/vision_to_mavros.py
  scripts/check_topics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
