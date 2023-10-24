###
set(TARGET PyCGALMesh)

add_cnoid_python_module(${TARGET}
  PyCGALModule.cpp
)

target_link_libraries(${TARGET}
  ${target_lib}
  ${catkin_LIBRARIES}
  ${CHOREONOID_UTIL_LIBRARIES}
)
