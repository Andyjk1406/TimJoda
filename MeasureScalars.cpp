/**
* File created by Andrew Keeling, University of Leeds
* Measure scalars in a file.
* The basic idea is that the scalars hold a distance measure to the nearest point in an opposing model
* The models have also been uniformly sampled at (eg) 0.025mm
* So each point represents a defined surface area
*/

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
 
#include <vtkActor.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSTLReader.h>
#include <vtkCleanPolyData.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkScalarBarActor.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkAdaptiveSubdivisionFilter.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkImplicitPolyDataDistance.h>

#include <iostream>
#include <fstream>


void writeData(std::string output_filename, std::string filename, int count, float max_dist);
bool fileExists(const char* fileName);

using namespace std;
 
int main(int argc, char* argv[])
{
  	
  vtkSmartPointer<vtkPolyData> model = vtkSmartPointer<vtkPolyData>::New();
  std::string model_filename;
  std::string output_filename;

  float max_distance = 0.1;
 
  // Load the VTP file (a uniform cloud) 
  if (argc == 4)
   {

	 model_filename = argv[1];
	 
	 vtkNew<vtkXMLPolyDataReader> reader;
	 reader->SetFileName(model_filename.c_str());
	 reader->Update();
	 model->DeepCopy(reader->GetOutput());

	 max_distance = atof(argv[2]);

	 output_filename = argv[3];
			
    }
  else
    {
	  cout << "Usage : ...exe model.vtp max_distance (all points within max distance will be added)";
	  return 0;
  }

  std::cout << "Loaded cloud and counting all points within " << max_distance << "mm" << std::endl;

  vtkIdType nPoints = model->GetNumberOfPoints();
  std::cout << "Number of points in model : " << nPoints << std::endl;

  vtkSmartPointer<vtkDoubleArray> distances = vtkDoubleArray::SafeDownCast(model->GetPointData()->GetScalars());

  int counter = 0;

  for (vtkIdType i = 0; i < distances->GetNumberOfTuples(); i++)
  {
	  double dist;
	  distances->GetTuple(i, &dist);

	  if (dist <= max_distance && dist > -0.5) counter++;
  }

  std::cout << counter << " valid points found" << std::endl;
  writeData(output_filename, model_filename, counter, max_distance);
	  
 
 
  // Display colour mapped image if we want
/*
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection( distanceFilter->GetOutputPort(1) );
  mapper->SetScalarRange(distanceFilter->GetOutput(1)->GetPointData()->GetScalars()->GetRange()[0],
   distanceFilter->GetOutput(1)->GetPointData()->GetScalars()->GetRange()[1]);
 
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper( mapper );
 
  vtkSmartPointer<vtkScalarBarActor> scalarBar =  vtkSmartPointer<vtkScalarBarActor>::New();
  scalarBar->SetLookupTable(mapper->GetLookupTable());
  scalarBar->SetTitle("Distance");
  scalarBar->SetNumberOfLabels(10);
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
 
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer( renderer );
 
  vtkSmartPointer<vtkRenderWindowInteractor> renWinInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renWinInteractor->SetRenderWindow( renWin );
 
  renderer->AddActor( actor );
  renderer->AddActor2D(scalarBar);
 
  renWin->Render();
  renWinInteractor->Start();
 
 */

  return EXIT_SUCCESS;
}


bool fileExists(const char *fileName)
{
	ifstream infile(fileName);
	return infile.good();
}


void writeData(std::string output_filename, std::string filename, int count, float max_dist)
{
	ofstream out;
	if (!fileExists(output_filename.c_str()))
	{
		out.open(output_filename.c_str(), ios::app);
		out << "Source nPointsinRange Area(mm2) RangeMax" << endl;

	}
	else {
		out.open(output_filename.c_str(), ios::app);
	}

	double area = (double)count * 0.000625;

	out << filename << " " << count<< " "<<area<<" ";
	out << " " << max_dist << std::endl;
	
	out.close();
}

