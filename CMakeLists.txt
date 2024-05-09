cmake_minimum_required(VERSION 3.0.2)
project(potential_fields)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
# INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_library(field
#   include/potential_fields/field.h
#   include/potential_fields/field.cpp)

add_executable(potential_fields src/potential_fields.cpp)
add_dependencies(potential_fields ${potential_fields_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(potential_fields
  ${catkin_LIBRARIES}
)

