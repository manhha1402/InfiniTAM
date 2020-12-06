cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
PROJECT(infinitam_ros)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_BUILD_TYPE "RELEASE")


#################################
# set catkin packages that are needed when running the code
set(catkin_RUN_PACKAGES
    cv_bridge
    image_transport
    message_generation
    roscpp
    roslib
    message_filters
    sensor_msgs
    std_msgs
    tf_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs
    )
# set catkin packages that are needed when building the code
set(catkin_BUILD_PACKAGES
    ${catkin_RUN_PACKAGES}
    cmake_modules
    )

find_package(catkin REQUIRED COMPONENTS ${catkin_BUILD_PACKAGES})
# Cuda module
INCLUDE(CheckLanguage)
CHECK_LANGUAGE(CUDA)
IF(CMAKE_CUDA_COMPILER)
ENABLE_LANGUAGE(CUDA)
ELSE()
MESSAGE(STATUS "No CUDA support")
ENDIF()
find_package(Boost REQUIRED
    system
    )
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
###Open3D############
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
MESSAGE("Open3D version : ${Open3D_VERSION}")
MESSAGE("Open3D DIR : ${Open3D_DIR}")
list(APPEND Open3D_LIBRARIES dl)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")
link_directories(${Open3D_LIBRARY_DIRS})
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)


catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    ${catkin_RUN_PACKAGES}
    DEPENDS

    LIBRARIES
    ${PROJECT_NAME}
    )


include_directories(
    ${CMAKE_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

ADD_SUBDIRECTORY(libs/ITMLib)
ADD_SUBDIRECTORY(libs/ORUtils)
ADD_SUBDIRECTORY(libs/MiniSlamGraphLib)
ADD_SUBDIRECTORY(libs/FernRelocLib)
ADD_SUBDIRECTORY(libs/InputSource)


add_executable(infinitam_ros_node
    infinitam_ros_node.cpp

    )
target_link_libraries(infinitam_ros_node
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Open3D_LIBRARIES}
    ${PCL_LIBRARIES}
    InputSource
    ITMLib
    ORUtils
    MiniSlamGraphLib
    FernRelocLib

    )


add_executable(infinitam_app
    infinitam_app.cpp
    )
target_link_libraries(infinitam_app
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Open3D_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    InputSource
    ITMLib
    ORUtils
    MiniSlamGraphLib
    FernRelocLib

    )

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )
#install(TARGETS infiniTAM_ros_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#install(TARGETS ITMLib
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#install(TARGETS ORUtils
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#install(TARGETS MiniSlamGraphLib
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#install(TARGETS FernRelocLib
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )





