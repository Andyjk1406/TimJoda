cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(YodaOcclusion)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.7 REQUIRED)

find_package(Open3D    QUIET HINTS "~/open3d_install/lib/CMake")

include_directories(${PCL_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS} "c:\\repos\\autoscanning\\include")
link_directories(${PCL_LIBRARY_DIRS} ${Open3D_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


 
add_executable (YodaOcclusion YodaOcclusion.cpp)
add_executable (AutoAlign AutoAlign.cpp)
add_executable (ScreenShot ScreenShot.cpp)
add_executable (ClipandMeasure ClipandMeasure.cpp)
add_executable (FeatureMeasure FeatureMeasure.cpp)
add_executable (RegionClipMeasure RegionClipMeasure.cpp)
add_executable (MeasureScalars MeasureScalars.cpp)
add_executable (GetTopologicalMatch GetTopologicalMatch.cpp)
add_executable (CreateDistanceVTP CreateDistanceVTP.cpp VTK_LoadSTLorPLY.hpp)
 
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

if(VTK_LIBRARIES)
  target_link_libraries(ClipandMeasure ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(ClipandMeasure ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()

if(VTK_LIBRARIES)
  target_link_libraries(FeatureMeasure ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(FeatureMeasure ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()

if(VTK_LIBRARIES)
  target_link_libraries(RegionClipMeasure ${PCL_LIBRARIES} ${VTK_LIBRARIES} "c:\\myProjects\\autoscanning\\release\\scanner_libs.lib")
else()
  target_link_libraries(RegionClipMeasure ${PCL_LIBRARIES} vtkHybrid vtkWidgets "c:\\myProjects\\autoscanning\\release\\scanner_libs.lib")
endif()

if(VTK_LIBRARIES)
  target_link_libraries(MeasureScalars ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(MeasureScalars ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()

if(VTK_LIBRARIES)
  target_link_libraries(GetTopologicalMatch ${PCL_LIBRARIES} ${VTK_LIBRARIES})
else()
  target_link_libraries(GetTopologicalMatch ${PCL_LIBRARIES} vtkHybrid vtkWidgets)
endif()

if(VTK_LIBRARIES)
  target_link_libraries(CreateDistanceVTP ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${Open3D_LIBRARIES})
else()
  target_link_libraries(CreateDistanceVTP ${PCL_LIBRARIES} ${Open3D_LIBRARIES} vtkHybrid vtkWidgets)
endif()

  # vtk_module_autoinit is needed
  vtk_module_autoinit(
    TARGETS CreateDistanceVTP
    MODULES ${VTK_LIBRARIES}
    )

vtk_module_autoinit(
    TARGETS ScreenShot
    MODULES ${VTK_LIBRARIES}
    )