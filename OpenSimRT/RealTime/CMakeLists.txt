# global files
file(GLOB includes include/*.h
  include/experimental/*.h)
file(GLOB sources src/*.cpp
  src/experimental/*.cpp)
# file(GLOB tests tests/*.cpp)
file(GLOB tests
  tests/TestIKFromFile.cpp
  tests/TestIKIMUFromFile.cpp
  tests/TestIDFromFile.cpp
  tests/TestSOFromFile.cpp
  tests/TestJRFromFile.cpp
  tests/TestRTFromFile.cpp
  tests/experimental/TestAccelerationGRFMPredictionFromFile.cpp
  tests/experimental/TestContactForceGRFMPredictionFromFile.cpp
  tests/experimental/TestMarkerReconstruction.cpp
  tests/experimental/TestRTExtFromFile.cpp
  )

# dependencies
include_directories(include/)
include_directories(include/experimental/)
include_directories(../Common/include/)
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} Common)

# dynamic library
set(target RealTime)
addLibrary(
  # UNITY_EXCLUDE "TRUE"
  KIT ${target}
  SOURCES ${sources}
  INCLUDES ${includes}
  LINKLIBS ${DEPENDENCY_LIBRARIES}
  # UNITY_EXCLUDED_SOURCES ${exclude_sources}
  )

# tests
addTests(
  TESTPROGRAMS ${tests}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
  )
