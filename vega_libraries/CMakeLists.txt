cmake_minimum_required(VERSION 2.8.3)
project(vega_libraries)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  #all src files
  src/CGSolver.cpp
  src/computeStiffnessMatrixNullspace.cpp
  src/configFile.cpp
  src/cubicMesh.cpp
  src/eig3.cpp
  src/forceModel.cpp
  src/forceModelAssembler.cpp
  src/generateInterpolationMatrix.cpp
  src/generateMassMatrix.cpp
  src/generateMeshGraph.cpp
  src/generateSurfaceMesh.cpp
  src/graph.cpp
  src/implicitNewmarkSparse.cpp
  src/integratorBase.cpp
  src/integratorBaseSparse.cpp
  src/interpolationWeightsMultiLoad.cpp
  src/linearSolver.cpp
  src/listIO.cpp
  src/massSpringStencilForceModel.cpp
  src/massSpringSystem.cpp
  src/massSpringSystemFromObjMesh.cpp
  src/massSpringSystemFromObjMeshConfigFile.cpp
  src/mat3d.cpp
  src/matrixIO.cpp
  src/sparseMatrix.cpp
  src/vec2d.cpp
  src/vec3.cpp
  src/vec3d.cpp
  src/vec3i.cpp
  src/vec4i.cpp
  src/volumetricMesh.cpp
  src/volumetricMeshDeformationGradient.cpp
  src/volumetricMeshENuMaterial.cpp
  src/volumetricMeshLoader.cpp
  src/volumetricMeshMooneyRivlinMaterial.cpp
  src/volumetricMeshOrthotropicMaterial.cpp
  src/volumetricMeshParser.cpp
  src/objMesh.cpp
  src/constrainedDOFs.cpp
  src/triKey.cpp
  src/geometryQuery.cpp
  src/rectKey.cpp
  src/disjointSet.cpp
  src/tribox3.cpp
  src/boundingBox.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
