cmake_minimum_required(VERSION 3.8)
project(grp_pibot23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Python scripts
install( PROGRAMS lib_import/scan_echo.py DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/pathfinding_1 DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/vision_robot_1 DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/vision_robot_2 DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/ghost_marker_publisher DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/pacman_marker_publisher DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/pathfinding_2 DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/goal_handler DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/global_goal_publisher DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/map_subscriber DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS src/operator_node DESTINATION lib/${PROJECT_NAME} )

# Install resource files.
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
