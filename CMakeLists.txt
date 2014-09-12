cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("symmetry_detection")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
#SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)

add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
#add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing -Wno-sign-compare)
set(CMAKE_COMPILER_EXECUTABLE "ccache colorgcc")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
  message(STATUS "Using Debug build type")
endif(NOT CMAKE_BUILD_TYPE)

# Implied by PCL
find_package(Boost REQUIRED COMPONENTS
  filesystem
  program_options
)

find_package(PCL REQUIRED COMPONENTS
  common
  io
  features
  filters
  surface
  octree
  visualization
)

### Include directories
include_directories(
  include/
  #include/${PROJECT_NAME}
  ${PCL_INCLUDE_DIRS}
)

### Normal projection
add_library(symmetry SHARED
  src/symmetry.cpp
)

target_link_libraries(symmetry
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_OCTREE_LIBRARIES}
  ${PCL_SURFACE_LIBRARIES}
)

add_executable(symmetry_detection
  src/main.cpp
)

target_link_libraries(symmetry_detection
  symmetry
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_SURFACE_LIBRARIES}
  ${PCL_FEATURES_LIBRARIES}
  ${PCL_OCTREE_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

