ADD_LIBRARY(types_slam3d ${G2O_LIB_TYPE}
  vertex_se3_quat.cpp
  vertex_se3_quat.h
  vertex_se3_euler.cpp
  vertex_se3_euler.h
  edge_se3_quat.cpp
  edge_se3_quat.h
  edge_se3_euler.cpp
  edge_se3_euler.h
  vertex_trackxyz.cpp
  vertex_trackxyz.h
  se3_offset_parameters.cpp
  se3_offset_parameters.h
  edge_se3_offset.cpp
  edge_se3_offset.h
  edge_se3_prior.cpp
  edge_se3_prior.h
  edge_se3_trackxyz.cpp
  edge_se3_trackxyz.h
  camera_parameters.cpp
  camera_parameters.h
  edge_project_disparity.cpp
  edge_project_disparity.h
  edge_project_depth.cpp
  edge_project_depth.h
  vertex_plane.cpp
  vertex_plane.h
  edge_se3_plane_calib.cpp
  edge_se3_plane_calib.h
  types_six_dof_quat.cpp
  types_six_dof_quat.h
)


SET_TARGET_PROPERTIES(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)

TARGET_LINK_LIBRARIES(types_slam3d ${OPENGL_gl_LIBRARY} math_groups core)



INSTALL(TARGETS types_slam3d
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam3d)


