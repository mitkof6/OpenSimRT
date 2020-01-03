# find OpenSim
find_package(OpenSim REQUIRED)
include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

include_directories(.)
add_subdirectory(Common)
add_subdirectory(RealTime)
if(BUILD_VICON)
  add_subdirectory(Vicon)
endif()
