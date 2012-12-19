INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_EXECUTABLE(simulator_3d_plane
  simulator_3d_plane.cpp
)

TARGET_LINK_LIBRARIES(simulator_3d_plane types_slam3d ${OPENGL_gl_LIBRARY} solver_csparse math_groups core)


SET_TARGET_PROPERTIES(simulator_3d_plane PROPERTIES OUTPUT_NAME simulator_3d_plane${EXE_POSTFIX})


