CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
project(octomap_dufo
	VERSION 1.0.0
	DESCRIPTION "Octomap in benchmark on dufo, dynamic in map"
	LANGUAGES CXX
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)
ADD_SUBDIRECTORY(octomap)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(default_build_type "Release")

#############
# PACKAGES  #
#############

find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
set(FUN_LIB Eigen3::Eigen)

include_directories(include)
add_library(dufo_lib SHARED octomap_mapping/src/octomapper.cpp)
target_link_libraries(dufo_lib ${LOG_LIB} yaml-cpp ${FUN_LIB} ${PCL_LIBRARIES})

############
# BINARIES #
############
add_executable(octomap_run octomap_PCD.cpp)
target_link_libraries(octomap_run dufo_lib)