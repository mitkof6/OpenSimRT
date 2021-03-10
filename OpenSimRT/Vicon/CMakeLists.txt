# configure Vicon data stream
set(
  VICONDATASTREAM_DIR $ENV{VICONDATASTREAM_DIR} CACHE PATH
  "Top-level directory of ViconDataStreamSDK"
)
set(ViconSDK_INCLUDE_DIRS ${VICONDATASTREAM_DIR})
include_directories(${ViconSDK_INCLUDE_DIRS})
find_library(
  ViconDataStreamSDK_CPP
  NAMES ViconDataStreamSDK_CPP
  PATHS ${VICONDATASTREAM_DIR}
  PATH_SUFFIXES lib)

set(ViconSDK_LIBRARIES ${ViconDataStreamSDK_CPP})
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} ${ViconSDK_LIBRARIES} Common RealTime)

# files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
file(GLOB tests tests/*.cpp)
file(GLOB applications applications/*.cpp)

# dependencies
include_directories(include/)
include_directories(../Common/include/)
include_directories(../RealTime/include/)

# dynamic library
set(target Vicon)
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

# applications
addApplications(
  SOURCES ${applications}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
)
