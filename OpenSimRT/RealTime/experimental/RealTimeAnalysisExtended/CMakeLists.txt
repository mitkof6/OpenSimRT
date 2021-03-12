# global files
file(GLOB add_includes include/*.h)
file(GLOB add_sources src/*.cpp)
# file(GLOB tests tests/*.cpp)
file(
  GLOB add_tests
  tests/TestRTExtFromFile.cpp
)

set(includes ${includes} ${add_includes} PARENT_SCOPE)
set(sources ${sources} ${add_sources} PARENT_SCOPE)
set(tests ${tests} ${add_tests} PARENT_SCOPE)
