#pkg_version=1.3
cmake_minimum_required(VERSION 2.8.3)
project(stargazer)

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
file(GLOB_RECURSE PROJECT_HEADER_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "include/*.h")
file(GLOB PROJECT_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.h")
file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")
set(LIBRARY_NAME ${PROJECT_NAME})

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
        INCLUDE_DIRS include
        LIBRARIES ${LIBRARY_NAME}
)


###########
## Build ##
###########
#compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" Cpp14CompilerFlag)
if (${Cpp14CompilerFlag})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "17")
    #no additional flag is required
else ()
    message(FATAL_ERROR "Compiler does not have c++14 support. Use at least g++4.9 or Visual Studio 2013 and newer.")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g")

# Add include and library directories
include_directories(
        include/${LIBRARY_NAME}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV2_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(${LIBRARY_NAME}
        ${PROJECT_HEADER_FILES_INC}
        ${PROJECT_SOURCE_FILES_INC}
        ${PROJECT_SOURCE_FILES_SRC}
        )
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
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
#install(FILES
#  res/test.png
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#############
## Testing ##
#############
