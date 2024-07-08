cmake_minimum_required(VERSION 3.8)
project(mirte_gazebo)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
  
# )
# catkin_package(
# CATKIN_DEPENDS roscpp std_msgs mirte_msgs
# )

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_ros pcl_conversions mirte_msgs)


# include_directories(${catkin_INCLUDE_DIRS})


# add_executable(rotate src/rotate.cpp)
# add_dependencies(rotate ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rotate ${catkin_LIBRARIES} )

# add_executable(convert_sonar src/convert_ultrasonic.cpp)
# add_dependencies(convert_sonar ${catkin_EXPORTED_TARGETS})
# target_link_libraries(convert_sonar ${catkin_LIBRARIES} )
# add_executable(convert_ir src/convert_ir.cpp)
# add_dependencies(convert_ir ${catkin_EXPORTED_TARGETS})
# target_link_libraries(convert_ir ${catkin_LIBRARIES} )
# add_executable(convert_encoder src/convert_encoder.cpp)
# add_dependencies(convert_encoder ${catkin_EXPORTED_TARGETS})
# target_link_libraries(convert_encoder ${catkin_LIBRARIES} )
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_C_STANDARD 11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# add_executable(listen src/main.cpp)
# ament_target_dependencies(listen rclcpp std_msgs sensor_msgs)
# install(TARGETS
# listen
#   DESTINATION lib/${PROJECT_NAME})

#   add_executable(listen_re src/main_re.cpp)
#   ament_target_dependencies(listen_re rclcpp std_msgs sensor_msgs)
#   install(TARGETS
#   listen_re
#     DESTINATION lib/${PROJECT_NAME})
  
#     add_executable(listen_pre src/main_pre.cpp)
#     ament_target_dependencies(listen_pre rclcpp std_msgs sensor_msgs)
#     install(TARGETS
#     listen_pre
#       DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY
# launch
# DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY
# urdf
# DESTINATION share/${PROJECT_NAME}/
# )
# install(DIRECTORY
# worlds
# DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY
# meshes
# DESTINATION share/${PROJECT_NAME}/
# )
foreach(X IN ITEMS "launch" "urdf" "meshes" "maps" "media" "models" "rviz" "config" "params")
    message(STATUS "X=${X}")
    install(DIRECTORY
${X}
DESTINATION share/${PROJECT_NAME}/
)
endforeach()


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
