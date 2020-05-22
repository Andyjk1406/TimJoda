/**
* File created by Andrew Keeling, University of Leeds
* Create a distance VTP file, uniformly sampled from a source mesh, and signed distances measured to target mesh
*/

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
 
#include <vtkActor.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCleanPolyData.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkScalarBarActor.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPLYWriter.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkAdaptiveSubdivisionFilter.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkPoints.h>

#include "VTK_LoadSTLorPLY.hpp"
#include <boost\filesystem.hpp>

#include "Open3D/Open3D.h"


float standard_deviation(vtkSmartPointer<vtkDistancePolyDataFilter> dist);

bool fileExists(const char *fileName);
 
int main(int argc, char* argv[])
{
  	
  vtkSmartPointer<vtkPolyData> source;
  vtkSmartPointer<vtkPolyData> target;
  double point_sampling_spacing;

  std::string source_filename, target_filename;
  std::string source_filename_vtp;
   
  
  // Load the 2 STL/PLY files, source and target and get the sampling spacing
  if (argc == 4 || argc== 5)
  {

	  source_filename = argv[1];
	  target_filename = argv[2];

	  source = loadSTLorPLY(source_filename);
	  if (!source) return -1;

	  auto o3d_source_mesh = open3d::io::CreateMeshFromFile(source_filename);
	  o3d_source_mesh->ComputeVertexNormals();
	  double mesh_area = o3d_source_mesh->GetSurfaceArea();
	  std::cout << "Surface area of source mesh is : " << mesh_area << endl;

	  boost::filesystem::path p(source_filename);
	  p.replace_extension("vtp");
	  source_filename_vtp = p.string();
	  if (argc == 5) source_filename_vtp = argv[4];

	  target = loadSTLorPLY(target_filename);
	  if (!target) return -1;

	  point_sampling_spacing = atof(argv[3]);

	  // Upsampling the source mesh
	  /*
	  double dnumber_of_samples = mesh_area / (point_sampling_spacing*point_sampling_spacing);
	  size_t number_of_samples = (size_t)dnumber_of_samples;
	  std::cout << "Aiming for " << number_of_samples << " point samples to create a spacing of " <<point_sampling_spacing << endl;
	  auto sampled = o3d_source_mesh->SamplePointsPoissonDisk(number_of_samples);
	  std::cout << sampled->points_.size() << " samples taken" << endl;

	  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	  points->SetNumberOfPoints(sampled->points_.size());
	  for (int i = 0; i < sampled->points_.size(); i++)
		  points->SetPoint(i, sampled->points_[i].x(), sampled->points_[i].y(), sampled->points_[i].z());
*/

	  
	  vtkSmartPointer<vtkPolyDataPointSampler> pointSampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
	  pointSampler->SetDistance(point_sampling_spacing);
	  pointSampler->SetInputData(source);
	  pointSampler->Update();
	  

	  vtkIdType nPoints = pointSampler->GetOutput()->GetNumberOfPoints();
	  std::cout << "Number of upsampled points in source : " << nPoints << std::endl;
	  

	  vtkSmartPointer<vtkPolyData> pd_sampled = vtkSmartPointer<vtkPolyData>::New();
	 // pd_sampled->SetPoints(points);
	  pd_sampled->DeepCopy(pointSampler->GetOutput());

	  // Need to add some cells (vertices) to thee geometry 
	  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	  vertexGlyphFilter->AddInputData(pd_sampled);
	  vertexGlyphFilter->Update();

	  vtkSmartPointer<vtkDoubleArray> distances = vtkSmartPointer<vtkDoubleArray>::New(); // The scalar array holding the distances
	  distances->SetNumberOfValues(nPoints);

	  // We want the signed distance to the target mesh
	  vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilter = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	  distanceFilter->SetInput(target);
	  for (vtkIdType i = 0; i < nPoints; i++)
	  {
		  double test_point[3];
		  pd_sampled->GetPoint(i, test_point);
		  double opposing_point_test[3];
		  double signedDistanceTest = distanceFilter->EvaluateFunctionAndGetClosestPoint(test_point, opposing_point_test);
		  distances->SetValue(i, signedDistanceTest);
	  }

	  pd_sampled->GetPointData()->SetScalars(distances);

	  // Get the mean/SD data and save it
	  //float sd = standard_deviation(distanceFilter);

	  // Save the upper with the distance scalars as a vtp file
	  vtkNew<vtkXMLPolyDataWriter> writer;
	  writer->SetFileName(source_filename_vtp.c_str());
	  //writer->SetInputConnection(distanceFilter->GetOutputPort(0));
	  writer->SetInputData(pd_sampled);
	  writer->Write();

  }
  else {

	  std::cout << "Takes a source and target mesh, subsamples the source every point_spacing mm,\nthen measures the signed distance from each point to the target mesh.\nSaves the source vtp file with the distances attached to the points" << std::endl;
	  std::cout << "Usage: CreateDistanceVTP.exe src.stl (or .ply) tgt.stl (or.ply) 0.025 (for 0.025mm spacing) [optional c:\\path\\save_filename.vtp]" << std::endl;
  }
  return EXIT_SUCCESS;
}


float standard_deviation(vtkSmartPointer<vtkDistancePolyDataFilter> dist)
{
	double meanNeg = 0.0, sum_deviationNeg = 0.0;
	double meanPos = 0.0, sum_deviationPos = 0.0;
	double mean = 0.0, sum_deviation = 0.0;
	double unsigned_mean = 0.0, unsigned_sum_dev = 0.0;
	vtkIdType nValues;
	int meanNegctr = 0, meanPosctr = 0;
	nValues = dist->GetOutput()->GetPointData()->GetScalars()->GetSize();

	for (vtkIdType i = 0; i<nValues; ++i)
	{
		double val = dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i);
		mean += val;
		if (val < 0) {	meanNeg += val; meanNegctr++;}
		if (val >= 0) { meanPos += val; meanPosctr++; }
		unsigned_mean += fabs(dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i));
	}

	//cout << "MeanNeg : " << meanNeg << endl;
	//cout << "Neg count : " << meanNegctr << endl;

	mean = mean / (double)nValues;
	meanNeg = meanNeg / (double)meanNegctr;
	meanPos = meanPos / (double)meanPosctr;
	unsigned_mean = unsigned_mean / (double)nValues;

	double negCtr = 0; double posCtr = 0;

	for (vtkIdType i = 0; i < nValues; ++i) {
		sum_deviation += (dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) - mean)*(dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) - mean);
		if (dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) < 0) {
			sum_deviationNeg += (dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) - meanNeg)*(dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) - meanNeg);
			negCtr++;
		}
		else {
			sum_deviationPos += (dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) - meanPos)*(dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i) - meanPos);
			posCtr++;
		}
	}
		
	sum_deviation = sqrt(sum_deviation / (double)nValues);
	sum_deviationNeg = sqrt(sum_deviationNeg / negCtr);
	sum_deviationPos = sqrt(sum_deviationPos / posCtr);

	/*
	ofstream out;
	if (!fileExists("DataResults.txt"))
	{
		out.open("DataResults.txt", ios::app);
		out << "Source Target Mean UMean Mean_Neg Mean_Pos SD SDpos SDneg RangeMin RangeMax" << endl;

	}
	else {
		out.open("DataResults.txt", ios::app);
	}

	out << src_filename << " " << tgt_filename;
	out << " "<< mean;
	out << " " << unsigned_mean;
	out << " " << meanNeg;
	out << " " << meanPos;
	out << " " << sum_deviation;
	out << " " << sum_deviationPos;
	out << " " << sum_deviationNeg;
	out << " " << dist->GetOutput()->GetPointData()->GetScalars()->GetRange()[0] << " " << dist->GetOutput()->GetPointData()->GetScalars()->GetRange()[1] << std::endl;

	out.close();
	*/


	std::cout << "Mean " << mean;
	std::cout << " UMean " << unsigned_mean;
	std::cout << " Mean_Neg " << meanNeg;
	std::cout << " Mean_Pos " << meanPos;
	std::cout << " SD " << sum_deviation;
	std::cout << " SDpos " << sum_deviationPos;
	std::cout << " SDneg " << sum_deviationNeg;
	std::cout << " Range " << dist->GetOutput()->GetPointData()->GetScalars()->GetRange()[0] << "  " << dist->GetOutput()->GetPointData()->GetScalars()->GetRange()[1] << std::endl;


	return sum_deviation;
}

bool fileExists(const char *fileName)
{
	ifstream infile(fileName);
	return infile.good();
}


