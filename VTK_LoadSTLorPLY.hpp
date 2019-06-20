#ifndef LOAD_VTK
#define LOAD_VTK

#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkPolyData.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


vtkSmartPointer<vtkPolyData> loadSTLorPLY(std::string filename)
{
	vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
	std::string extension = boost::filesystem::extension(filename);

	if (boost::iequals(extension, ".ply"))
	{
		// Load a PLY source
		vtkSmartPointer<vtkPLYReader> reader1 = vtkSmartPointer<vtkPLYReader>::New();
		reader1->SetFileName(filename.c_str());
		reader1->Update();	
		poly = reader1->GetOutput();
	}
	else if (boost::iequals(extension, ".stl"))
	{
		// Load an STL source
		vtkSmartPointer<vtkSTLReader> reader1 = vtkSmartPointer<vtkSTLReader>::New();
		reader1->SetFileName(filename.c_str());
		reader1->Update();
		poly = reader1->GetOutput();
	}
	else {

		std::cout << "Error! Must provide a source STL or PLY!" << std::endl;
		poly = nullptr;
	
	}

	return poly;

}




#endif // LOAD_VTK