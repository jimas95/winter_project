cmake_minimum_required(VERSION 3.0.2)
project(simple_node)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp rospy)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})



add_executable(${PROJECT_NAME}_hello src/hello.cpp)
add_executable(${PROJECT_NAME}_set_goal src/set_goal.cpp)

set_target_properties(${PROJECT_NAME}_hello PROPERTIES OUTPUT_NAME hello PREFIX "")
set_target_properties(${PROJECT_NAME}_set_goal PROPERTIES OUTPUT_NAME set_goal PREFIX "")


add_dependencies(${PROJECT_NAME}_hello ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_set_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_hello ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_set_goal ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_hello RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_set_goal  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


