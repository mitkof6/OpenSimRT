# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB tests tests/*.cpp)

# dependencies
include_directories(include/)
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES})

# dynamic library
set(target Common)
addLibrary(
    UNITY_EXCLUDE "TRUE"
    KIT ${target}
    SOURCES ${sources}
    INCLUDES ${includes}
    LINKLIBS ${DEPENDENCY_LIBRARIES}
    # UNITY_EXCLUDED_SOURCES ${exclude_sources}
)

# # tests
# addTests(
#     TESTPROGRAMS ${tests}
#     LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
# )
