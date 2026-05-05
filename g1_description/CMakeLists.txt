cmake_minimum_required(VERSION 3.8)
project(g1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(FILES g1_23dof.urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
