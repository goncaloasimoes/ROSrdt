cmake_minimum_required(VERSION 2.8.3)
project(rdt)
                
find_package(catkin REQUIRED COMPONENTS costmap_2d message_generation roscpp rospy map_msgs pluginlib actionlib)

catkin_python_setup()

add_service_files(
  DIRECTORY srv 
  FILES init.srv make_plan.srv
)

generate_messages(DEPENDENCIES map_msgs nav_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rdt
    CATKIN_DEPENDS costmap_2d roscpp message_runtime map_msgs nav_msgs geometry_msgs pluginlib actionlib
)
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS src/rdt2_guiless.py src/goals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(global_planner src/global_planner.cpp)
add_dependencies(global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES rdtgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)