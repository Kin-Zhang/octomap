CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
project(octomap_dufo
	VERSION 1.0.0
	DESCRIPTION "Octomap in benchmark on dufo, dynamic in map"
	LANGUAGES CXX
)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # enables -fPIC in applicable compilers (required to avoid link errors in some cases)

ADD_SUBDIRECTORY(octomap)


