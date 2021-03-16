# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB tests tests/*.cpp)
file(GLOB tests
  tests/TestIKFromFile.cpp
  tests/TestIKIMUFromFile.cpp
  tests/TestIDFromFile.cpp
  tests/TestSOFromFile.cpp
  tests/TestJRFromFile.cpp
  tests/TestRTFromFile.cpp
  )

# dependencies
include_directories(include/)
include_directories(../Common/include/)
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} Common)

if(USE_EXPERIMENTAL)
  # global experimental files
  file(GLOB add_includes include/experimental/*.h)
  file(GLOB add_sources src/experimental/*.cpp)
  # file(GLOB add_tests tests/experimental/*.cpp)
  file(GLOB add_tests
    tests/experimental/TestAccelerationGRFMPredictionFromFile.cpp
    tests/experimental/TestContactForceGRFMPredictionFromFile.cpp
    tests/experimental/TestMarkerReconstruction.cpp
    tests/experimental/TestRTExtFromFile.cpp
    )

  # append files
  set(includes ${includes} ${add_includes})
  set(sources ${sources} ${add_sources})
  set(tests ${tests} ${add_tests})

  # include experimental headers
  include_directories(include/experimental/)
endif(USE_EXPERIMENTAL)

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
