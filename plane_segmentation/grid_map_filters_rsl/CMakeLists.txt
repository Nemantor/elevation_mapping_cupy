# Project configuration
cmake_minimum_required (VERSION 3.5.1)
project(grid_map_filters_rsl)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
# catkin_package(
#   INCLUDE_DIRS 
#     include
#       ${EIGEN3_INCLUDE_DIR}
#   LIBRARIES 
#     ${PROJECT_NAME}
#   CATKIN_DEPENDS
#     ${CATKIN_PACKAGE_DEPENDENCIES}
# )

ament_export_dependencies(grid_map_core)
ament_export_dependencies(grid_map_cv)
ament_export_dependencies(Eigen3)


###########
## Build ##
###########
add_library(${PROJECT_NAME}
  src/GridMapDerivative.cpp
  src/inpainting.cpp
  src/lookup.cpp
  src/smoothing.cpp
  src/processing.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${EIGEN3_INCLUDE_DIR}
)


ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
  grid_map_cv
  Eigen3
)

##########
## Test ##
##########

# ## GTest.
# if(CATKIN_ENABLE_TESTING)
#   ## Find catkin dependencies, including test dependencies.
#   find_package(catkin REQUIRED
#     COMPONENTS
#     ${CATKIN_PACKAGE_DEPENDENCIES}
#     )

#   catkin_add_gtest(test_${PROJECT_NAME}
#     test/TestDerivativeFilter.cpp
#     test/TestFilters.cpp
#     test/TestLookup.cpp
#     )
# endif()

## Link GTest.
# if(TARGET test_${PROJECT_NAME})
#   target_link_libraries(test_${PROJECT_NAME}
#     ${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     gtest_main
#     )

#   target_include_directories(test_${PROJECT_NAME} PRIVATE
#     include
#     )

#   target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS}
#     )

#   # Generate test coverage report
#   find_package(cmake_code_coverage QUIET)
#   if(cmake_code_coverage_FOUND)
#     add_gtest_coverage(
#       TEST_BUILD_TARGETS test_${PROJECT_NAME}
#     )
#   endif(cmake_code_coverage_FOUND)
# endif()

#############
## Install ##
#############
# install(
#   TARGETS
#     ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   DIRECTORY
#     include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)


###########
## Clang ##
###########
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

ament_package()