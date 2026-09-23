cmake_minimum_required(VERSION 3.8)
project(terrain_layer)

# std::filesystem / C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/terrain_layer.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# 导出插件描述文件，让 Nav2 能通过 pluginlib 找到本 layer
pluginlib_export_plugin_description_file(nav2_costmap_2d terrain_layer_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES terrain_layer_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/build
    ${CMAKE_CURRENT_SOURCE_DIR}/install
    ${CMAKE_CURRENT_SOURCE_DIR}/log
  )
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_lint_cmake
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ros_log)
  ament_add_gtest(test_terrain_layer_tf
    test/test_terrain_layer_tf.cpp
    ENV
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log
  )
  if(TARGET test_terrain_layer_tf)
    target_include_directories(test_terrain_layer_tf PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_terrain_layer_tf
      rclcpp
      geometry_msgs
      nav2_costmap_2d
      nav2_util
      tf2
      tf2_geometry_msgs
      tf2_ros
    )
    target_link_libraries(test_terrain_layer_tf
      ${PROJECT_NAME}
      yaml-cpp
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml-cpp
)

ament_package()
