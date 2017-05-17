#pkg_version=2.0.3
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(stargazer)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

###################
## find packages ##
###################
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

############################
## read source code files ##
############################
file(GLOB_RECURSE PROJECT_HEADER_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "include/*.h" "include/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.h" "src/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include ${catkin_EXPORT_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME} ${catkin_EXPORT_LIBRARIES}
    CATKIN_DEPENDS ${catkin_EXPORT_DEPENDS}
    )

###########
## Build ##
###########
# Add include and library directories
include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )

link_directories(
    ${catkin_LIBRARY_DIRS}
    )

# Declare a cpp library
add_library(${PROJECT_NAME}
    ${PROJECT_HEADER_FILES_INC}
    ${PROJECT_SOURCE_FILES_INC}
    ${PROJECT_SOURCE_FILES_SRC}
    )
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    )


#############
## Install ##
#############
if (TARGET ${PROJECT_NAME})
    # Mark library for installation
    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
endif ()
# Mark c++ header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############
# Add test targets for cpp tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_config_handler test/test_ConfigHandler.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/test)
  add_dependencies(test_config_handler
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(test_config_handler
    ${catkin_LIBRARIES}
    yaml-cpp
    )
  catkin_add_gtest(test_landmark test/test_landmark)
  add_dependencies(test_landmark
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
  target_link_libraries(test_landmark
    ${catkin_LIBRARIES}
    )
endif()
