cmake_minimum_required(VERSION 2.8.3)
project(image_utils)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find CUDA
FIND_PACKAGE(CUDA)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find required packeges
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.7 REQUIRED)
# Find OpenCV
find_package (OpenCV 3.4.0 EXACT REQUIRED 
	NO_MODULE  # Should be optional, tells CMake to use config mode
	PATHS /usr/local # Look here
	NO_DEFAULT_PATH # And don't look anywhere else
)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    actionlib
    image_transport
    dynamic_reconfigure
  	pcl_conversions
	  pcl_ros  
)

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

generate_dynamic_reconfigure_options(
  cfg/GeneralRQT.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
    ${Boost_INCLUDE_DIRS}
  LIBRARIES
    utils_lib
    im_grab_lib
    im_proc_lib
    pcl_utils_lib
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    pcl_conversions
    pcl_ros
    std_msgs
    image_transport
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if (CUDA_FOUND)
  link_directories(/usr/local/cuda/lib64)
  
  cuda_add_library(utils_lib src/libraries/Utilities.cpp)
  cuda_add_library(im_grab_lib src/libraries/ImageGrabber.cpp)
  cuda_add_library(im_proc_lib src/libraries/ImageProcessor.cpp)
  cuda_add_library(pcl_utils_lib src/libraries/PCLUtilities.cpp)
  cuda_add_executable(testNode src/test_node.cpp)
else()
  add_library(utils_lib src/libraries/Utilities.cpp)
  add_library(im_grab_lib src/libraries/ImageGrabber.cpp)
  add_library(im_proc_lib src/libraries/ImageProcessor.cpp)
  add_library(pcl_utils_lib src/libraries/PCLUtilities.cpp)
  
  add_executable(testNode src/test_node.cpp)
endif()

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

target_link_libraries(utils_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(im_grab_lib utils_lib)
target_link_libraries(im_proc_lib utils_lib dlib)
target_link_libraries(pcl_utils_lib utils_lib)

add_dependencies(utils_lib ${PROJECT_NAME}_gencfg)
target_link_libraries(testNode utils_lib im_grab_lib im_proc_lib pcl_utils_lib)
