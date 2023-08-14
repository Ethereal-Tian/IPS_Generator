cmake_minimum_required(VERSION 2.8.3)
project(rgbd2polari)

ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(pfm_io src/PFM_ReadWrite/PFMReadWrite.cpp)
target_link_libraries(pfm_io ${OpenCV_LIBS})

add_library(rgbd2polari src/rgbd2polari.cpp)
target_link_libraries(rgbd2polari pfm_io ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(polar_processor src/polar_processor.cpp)
target_link_libraries(polar_processor pfm_io ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(DolpAolpToZenithAzimuth src/DolpAolpToZenithAzimuth.cpp)
target_link_libraries(DolpAolpToZenithAzimuth pfm_io ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(utils src/utils.cpp)
target_link_libraries(utils pfm_io ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(DAoLP2Normal src/DAoLP2Normal.cpp)
target_link_libraries(DAoLP2Normal utils)

add_executable(process_single_image src/process_single_image.cpp)
target_link_libraries(process_single_image rgbd2polari)

add_executable(process_data src/process_data.cpp)
target_link_libraries(process_data rgbd2polari)

add_executable(process_dataset src/process_dataset.cpp)
target_link_libraries(process_dataset rgbd2polari)

add_executable(valid_disparity src/valid_disparity.cpp)
target_link_libraries(valid_disparity rgbd2polari)

add_executable(norm_dataset src/norm_dataset.cpp)
target_link_libraries(norm_dataset rgbd2polari)

add_executable(read_pfm_node src/read_pfm_node.cpp)
target_link_libraries(read_pfm_node rgbd2polari)

add_executable(transform_exr2pfm src/transform_exr2pfm.cpp)
target_link_libraries(transform_exr2pfm pfm_io ${OpenCV_LIBS})

add_executable(show_saved_data src/show_saved_data.cpp)
target_link_libraries(show_saved_data rgbd2polari)

add_executable(random_reflection src/random_reflection.cpp)
target_link_libraries(random_reflection rgbd2polari)

add_executable(rebuild_by_ratio_distribution src/rebuild_by_ratio_distribution.cpp)
target_link_libraries(rebuild_by_ratio_distribution rgbd2polari utils polar_processor)
