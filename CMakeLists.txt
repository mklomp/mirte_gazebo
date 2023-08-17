cmake_minimum_required(VERSION 3.0.2)
project(mirte_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
)
catkin_package(
CATKIN_DEPENDS roscpp std_msgs mirte_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS roscpp prius_msgs sensor_msgs pcl_ros pcl_conversions mirte_msgs)


include_directories(${catkin_INCLUDE_DIRS})


add_executable(rotate src/rotate.cpp)
add_dependencies(rotate ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotate ${catkin_LIBRARIES} )

add_executable(convert_sonar src/convert_ultrasonic.cpp)
add_dependencies(convert_sonar ${catkin_EXPORTED_TARGETS})
target_link_libraries(convert_sonar ${catkin_LIBRARIES} )
add_executable(convert_ir src/convert_ir.cpp)
add_dependencies(convert_ir ${catkin_EXPORTED_TARGETS})
target_link_libraries(convert_ir ${catkin_LIBRARIES} )
add_executable(convert_encoder src/convert_encoder.cpp)
add_dependencies(convert_encoder ${catkin_EXPORTED_TARGETS})
target_link_libraries(convert_encoder ${catkin_LIBRARIES} )
