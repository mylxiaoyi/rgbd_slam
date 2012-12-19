ADD_LIBRARY(math_groups ${G2O_LIB_TYPE}
  se3quat_gradients.cpp
  se3quat_gradients.h
)

SET_TARGET_PROPERTIES(math_groups PROPERTIES OUTPUT_NAME ${LIB_PREFIX}math_groups)

IF (NOT MSVC)
TARGET_LINK_LIBRARIES(math_groups m)
ENDIF()

ADD_EXECUTABLE(test_math_groups
  test_math_groups.cpp)

INSTALL(TARGETS math_groups
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

ADD_EXECUTABLE(plane_test
  plane_test.cpp
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/math_groups)
