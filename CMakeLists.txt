cmake_minimum_required(VERSION 3.5.0)
project(cnoid_cgal)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

find_package(Choreonoid REQUIRED)
find_package(CGAL REQUIRED)
find_package(OCTOMAP REQUIRED)

## find_package(PCL REQUIRED COMPONENTS kdtree)
## define PCLSEARCH
if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++17)
endif()

#add_definitions(-DDEBUG_MSG)

set(target_lib CnoidCGAL)

choreonoid_add_library(
  ${target_lib}
  SHARED
  src/CnoidCGAL.cpp
  src/CGALUtil.cpp
  src/CnoidOctomap.cpp
  src/MergeBoxes.cpp
  )
target_include_directories(${target_lib}
  PUBLIC
  ${CGAL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ## ${PCL_KDTREE_INCLUDE_DIRS}
  )
target_link_libraries(${target_lib}
  PUBLIC
  Choreonoid::CnoidUtil
  ${CGAL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ## ${PCL_KDTREE_LIBRARIES}
  )

add_subdirectory(pybind11)

install(FILES
  python/CGALUtil.py
  DESTINATION
  ${CHOREONOID_ROOT_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid
  )
