################################################################################
# project
cmake_minimum_required(VERSION 3.0)
project(Gait1848MomentArm VERSION 1.0.0)

# cmake modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/CMakeHelpers.cmake)

# enable testing
include(CTest)
enable_testing()

# define library postfix
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "Release postfix")
set(CMAKE_DEBUG_POSTFIX "" CACHE STRING "Debug postfix")
set(CMAKE_RELWITHDEBINFO_POSTFIX "" CACHE STRING "Release with debug info postfix")
set(CMAKE_MINSIZEREL_POSTFIX "" CACHE STRING "Minimum size release postfix")
mark_as_advanced(CMAKE_RELEASE_POSTFIX)
mark_as_advanced(CMAKE_DEBUG_POSTFIX)
mark_as_advanced(CMAKE_RELWITHDEBINFO_POSTFIX)
mark_as_advanced(CMAKE_MINSIZEREL_POSTFIX)

# # build executables and libraries in the same place
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL "")
# set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL "")

################################################################################

# find OpenSim
find_package(OpenSim REQUIRED)
include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

################################################################################

# global files
file(GLOB includes MomentArm.h MomentArmExports.h)
file(GLOB sources MomentArm.cpp)
file(GLOB tests
  # sometimes this produces seg fault during CI
  TestMomentArmGait1848.cpp
)

# dynamic library
set(target Gait1848MomentArm)
addLibrary(
  KIT ${target}
  SOURCES ${sources}
  INCLUDES ${includes}
  LINKLIBS ${OpenSim_LIBRARIES})
set_target_properties(${target} PROPERTIES LINKER_LANGUAGE CXX PREFIX "")

# tests
addTests(
  TESTPROGRAMS ${tests}
  LINKLIBS ${target} ${OpenSim_LIBRARIES})

################################################################################
