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
  include_directories(experimental/GroundWrenchPrediction/include)
  include_directories(experimental/MarkerReconstruction/include)
  include_directories(experimental/RealTimeAnalysisExtended/include)
  add_subdirectory(experimental)
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
