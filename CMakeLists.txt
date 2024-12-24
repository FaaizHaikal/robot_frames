cmake_minimum_required(VERSION 3.8)
project(robot_frames)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/node/robot_frames_node.cpp"
  "src/${PROJECT_NAME}/model/robot_wrapper.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  urdf
  kdl_parser
  geometry_msgs
  tachimawari_interfaces
  kansei_interfaces
  keisan)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(main "src/main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(main ${PROJECT_NAME})

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  tf2_ros
  urdf
  kdl_parser
  geometry_msgs
  tachimawari_interfaces
  kansei_interfaces
  keisan
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
