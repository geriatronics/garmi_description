# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(garmi_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package providing the rqt joystick plugin.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/demo_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

# rqt plugin descriptor (referenced from package.xml <export><rqt_gui/>).
install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
