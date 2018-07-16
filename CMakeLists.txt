cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(YodaOcclusion)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable (YodaOcclusion YodaOcclusion.cpp)
add_executable (AutoAlign AutoAlign.cpp)
add_executable (ScreenShot ScreenShot.cpp)
 
if(VTK_LIBRARIES)
  target_link_libraries(YodaOcclusion ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(YodaOcclusion ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()

if(VTK_LIBRARIES)
  target_link_libraries(AutoAlign ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(AutoAlign ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()

if(VTK_LIBRARIES)
  target_link_libraries(ScreenShot ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(ScreenShot ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()