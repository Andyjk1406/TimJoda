cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(YodaOcclusion)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable (YodaOcclusion YodaOcclusion.cpp)
 
if(VTK_LIBRARIES)
  target_link_libraries(YodaOcclusion ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(YodaOcclusion ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()