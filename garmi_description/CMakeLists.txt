# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(garmi_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
