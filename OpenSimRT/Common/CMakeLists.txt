# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB tests tests/*.cpp)
file(GLOB tests
  # tests/TestCircularBuffer.cpp
  # tests/TestLowPassSmoothFilter.cpp
  # tests/TestLowPassSmoothFilterTS.cpp
  # tests/TestSyncManager.cpp
  )
# dependencies
include_directories(include/)
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES})

# dynamic library
set(target Common)
addLibrary(
  KIT ${target}
  SOURCES ${sources}
  INCLUDES ${includes}
  LINKLIBS ${DEPENDENCY_LIBRARIES}
  )

# tests
addTests(
  TESTPROGRAMS ${tests}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
  )
