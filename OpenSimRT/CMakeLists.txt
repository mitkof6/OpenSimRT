include_directories(
  .
  Common/)

add_subdirectory(RealTime)
if(BUILD_VICON)
  add_subdirectory(Vicon)
endif()
