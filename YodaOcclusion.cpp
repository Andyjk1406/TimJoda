/**
* File created by Andrew Keeling, University of Leeds
* Measure Occlusion. For research in conjunction with Tim Yoda, Basel, Switzerland
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
#include <vtkScalarBarActor.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>


float standard_deviation(vtkSmartPointer<vtkDistancePolyDataFilter> dist);

bool fileExists(const char *fileName);
 
int main(int argc, char* argv[])
{
  	
  vtkSmartPointer<vtkPolyData> upper;
  vtkSmartPointer<vtkPolyData> lower;

  std::string upper_filename, lower_filename;
  std::string upper_filename_stl, lower_filename_stl;
  std::string upper_filename_vtp, lower_filename_vtp;
 
  // Load the 2 STL files, upper and lower
  if (argc == 3)
   {

	 upper_filename = argv[1];
	 lower_filename = argv[2];

	 upper_filename_stl = upper_filename + ".stl";
	 lower_filename_stl = lower_filename + ".stl";

	 upper_filename_vtp = upper_filename + ".vtp";
	 lower_filename_vtp = lower_filename + ".vtp";

	vtkSmartPointer<vtkSTLReader> reader1 = vtkSmartPointer<vtkSTLReader>::New();
	reader1->SetFileName(upper_filename_stl.c_str());
	reader1->Update();
	upper = reader1->GetOutput();	

	vtkSmartPointer<vtkSTLReader> reader2 = vtkSmartPointer<vtkSTLReader>::New();
	reader2->SetFileName(lower_filename_stl.c_str());
	reader2->Update();
	lower = reader2->GetOutput();		
		
    }
  else
    {
	  cout << "Usage : ...exe upper(.stl) lower(.stl) [The '.stl' will be appended automatically]";
	  return 0;
  }

  // Calculate the distance from target_crop to source
  vtkSmartPointer<vtkDistancePolyDataFilter> distanceFilter = vtkSmartPointer<vtkDistancePolyDataFilter>::New();
  distanceFilter->SetInputData( 0, upper );
  distanceFilter->SetInputData( 1, lower );
  distanceFilter->ComputeSecondDistanceOn();
  distanceFilter->Update();

  // Get the mean/SD data and save it
  //float sd = standard_deviation(distanceFilter);

  // Save the upper with the distance scalars as a vtp file
  vtkNew<vtkXMLPolyDataWriter> writer;
  writer->SetFileName(upper_filename_vtp.c_str());
  writer->SetInputConnection(distanceFilter->GetOutputPort(0));
  writer->Write();
  writer->SetFileName(lower_filename_vtp.c_str());
  writer->SetInputConnection(distanceFilter->GetOutputPort(1));
  writer->Update();
  writer->Write();

/*  
  vtkNew<vtkPolyData> orig;
  vtkNew<vtkPolyData> loaded;

  orig->DeepCopy(distanceFilter->GetOutput());

  // Load the upper with the distance scalars as a vtp file
  vtkNew<vtkXMLPolyDataReader> reader;
  reader->SetFileName(lower_filename_vtp.c_str());
  reader->Update();
  loaded->DeepCopy(reader->GetOutput());

  // Check Scalars
  vtkSmartPointer<vtkDoubleArray> orig_scalars;
  vtkSmartPointer<vtkDoubleArray> loaded_scalars;

  orig_scalars = vtkDoubleArray::SafeDownCast(orig->GetPointData()->GetScalars());
  loaded_scalars = vtkDoubleArray::SafeDownCast(loaded->GetPointData()->GetScalars());

  std::cout << "Original : " << orig_scalars->GetTuple1(10) << std::endl;
  std::cout << "Loaded : " << loaded_scalars->GetTuple1(10) << std::endl;

  */
 
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


