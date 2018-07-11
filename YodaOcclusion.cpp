#include <vtkVersion.h>
#include <vtkSmartPointer.h>
 
#include <vtkActor.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPLYReader.h>
#include <vtkCleanPolyData.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkScalarBarActor.h>
#include <vtkSphereSource.h>
#include <vtkBooleanOperationPolyDataFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/gicp.h>

typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT>  PointCloudT;

float standard_deviation(vtkSmartPointer<vtkDistancePolyDataFilter> dist);
void getWorstFraction(vtkSmartPointer<vtkDistancePolyDataFilter> dist, float fraction);

bool saveALNFile(std::string filename, std::vector<std::string>& v_meshNames, std::vector<Eigen::Matrix4f> v_mats);

bool fileExists(const char *fileName);

std::string src_filename, tgt_filename, base_dir;
 
int main(int argc, char* argv[])
{
  	
  vtkSmartPointer<vtkPolyData> source;
  vtkSmartPointer<vtkPolyData> target;
  vtkSmartPointer<vtkPolyData> target_cropped;
  float fraction;

  // Load the 3 PLY files, source, target, target_cropped
  if (argc == 6)
    {

	base_dir = argv[1];
	std::string temp;

	vtkSmartPointer<vtkPLYReader> reader1 = vtkSmartPointer<vtkPLYReader>::New();
	temp = base_dir;
	temp +=argv[2];
	reader1->SetFileName(temp.c_str());
	reader1->Update();
	source = reader1->GetOutput();	

	vtkSmartPointer<vtkPLYReader> reader2 = vtkSmartPointer<vtkPLYReader>::New();
	temp = base_dir;
	temp += argv[3];
	reader2->SetFileName(temp.c_str());
	reader2->Update();
	target = reader2->GetOutput();		

	vtkSmartPointer<vtkPLYReader> reader3 = vtkSmartPointer<vtkPLYReader>::New();
	temp = base_dir;
	temp += argv[4];
	reader3->SetFileName(temp.c_str());
	reader3->Update();
	target_cropped = reader3->GetOutput();

	fraction = atof(argv[5]);
	
	src_filename = argv[2];
	tgt_filename = argv[4];

    }
  else
    {
	  cout << "Usage : ...exe path/to/dir/ source.ply target.ply target_cropped.ply fraction";
	  return 0;
  }

  // Create PCL clouds for alignment (source and target)
  PointCloudT::Ptr source_cloud(new PointCloudT);
  pcl::io::vtkPolyDataToPointCloud(source, *source_cloud);
 
  PointCloudT::Ptr target_cloud(new PointCloudT);
  pcl::io::vtkPolyDataToPointCloud(target, *target_cloud);

  // Align source to target using GICP
  Eigen::Matrix4f icpMatrix;
  PointCloudT::Ptr icp_alignedCloud(new PointCloudT);

  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-12);
  icp.setEuclideanFitnessEpsilon(1e-12);
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);

  icp.setMaxCorrespondenceDistance(2); // Start with 2mm search zone
  icp.align(*icp_alignedCloud);
  icpMatrix = icp.getFinalTransformation();

  icp.setMaxCorrespondenceDistance(0.3); // Reduce to 0.3mm search zone
  icp.align(*icp_alignedCloud, icpMatrix);
  icpMatrix = icp.getFinalTransformation();

  // Convert the transformation into a vtk readable form
  vtkSmartPointer<vtkMatrix4x4> VTKmatrix = vtkSmartPointer<vtkMatrix4x4>::New();
  for (unsigned int r = 0; r<4; r++) {
	  for (unsigned int c = 0; c<4; c++)
		  VTKmatrix->Element[r][c] = icpMatrix(r, c);
  }

  // Transform the source PLY
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(VTKmatrix);

  vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  filter->SetTransform(transform);
  filter->SetInputData(source);
  filter->Update();

 // Clean the transformed source and the cropped target (might need this for the vtkDistanceFilter?)
  vtkSmartPointer<vtkCleanPolyData> cleanSource = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanSource->SetInputData( filter->GetOutput());
 
  vtkSmartPointer<vtkCleanPolyData> cleanTarget_cropped =vtkSmartPointer<vtkCleanPolyData>::New();
  cleanTarget_cropped->SetInputData( target_cropped);

 
  // Calculate the distance from target_crop to source
  vtkSmartPointer<vtkDistancePolyDataFilter> distanceFilter = vtkSmartPointer<vtkDistancePolyDataFilter>::New();
  distanceFilter->SetInputConnection( 0, cleanTarget_cropped->GetOutputPort() );
  distanceFilter->SetInputConnection( 1, cleanSource->GetOutputPort() );
  distanceFilter->Update();

/*
  // Boolean (Not used in the distance code)
  vtkSmartPointer<vtkBooleanOperationPolyDataFilter> booleanOp = vtkSmartPointer<vtkBooleanOperationPolyDataFilter>::New();
  booleanOp->SetOperationToIntersection();
  booleanOp->SetInputConnection(0, clean1->GetOutputPort());
  booleanOp->SetInputConnection(1, clean2->GetOutputPort());

  vtkSmartPointer<vtkPolyDataMapper> bmapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  bmapper->SetInputConnection(booleanOp->GetOutputPort());
  bmapper->ScalarVisibilityOff();
  vtkSmartPointer<vtkActor> bactor =vtkSmartPointer<vtkActor>::New();
  bactor->SetMapper(bmapper);
*/

  // Get the mean/SD data and save it
  float sd = standard_deviation(distanceFilter);

  // Get the worst xx% and save the data
  getWorstFraction(distanceFilter, fraction);

  // Save a meshlab ALN file--------------------------------------------------------------------------------------------
  std::string aln_filename = base_dir + src_filename.erase(src_filename.length() - 4) + "_" + tgt_filename.erase(tgt_filename.length() - 4)+".aln";
  std::vector<std::string> v_meshNames;
  std::vector<Eigen::Matrix4f> v_mats;

  v_meshNames.push_back(src_filename); // Now with no '.ply'
  v_meshNames.push_back(tgt_filename); // Now with no '.ply'

  v_mats.push_back(icpMatrix);
  v_mats.push_back(Eigen::Matrix4f::Identity());

  saveALNFile(aln_filename, v_meshNames, v_mats);
  //----------------------------------------------------------------------------------------------------------------------

 
  // Display colour mapped image if we want
/*
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection( distanceFilter->GetOutputPort() );
  mapper->SetScalarRange(distanceFilter->GetOutput()->GetPointData()->GetScalars()->GetRange()[0],
   distanceFilter->GetOutput()->GetPointData()->GetScalars()->GetRange()[1]);
 
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper( mapper );
 
  vtkSmartPointer<vtkScalarBarActor> scalarBar =  vtkSmartPointer<vtkScalarBarActor>::New();
  scalarBar->SetLookupTable(mapper->GetLookupTable());
  scalarBar->SetTitle("Distance");
  scalarBar->SetNumberOfLabels(4);
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

void getWorstFraction(vtkSmartPointer<vtkDistancePolyDataFilter> dist, float fraction)
{

	vtkIdType nValues;
	nValues = dist->GetOutput()->GetPointData()->GetScalars()->GetSize();
	std::vector<float> vDistances(nValues);

	for (vtkIdType i = 0; i<nValues; ++i)
	{
		double val = dist->GetOutput()->GetPointData()->GetScalars()->GetTuple1(i);
		vDistances[i] = fabs(val);
	}

	std::sort(vDistances.begin(), vDistances.end(), std::greater<float>());

	int nDistsToSave =(int) ((float)nValues * fraction);

	ofstream out("WorstPercentage.txt", ios::app);
	out << src_filename << " " << tgt_filename <<" ";

	for (int i = 0; i < nDistsToSave; i++)
		out << vDistances[i] << " ";

	out << std::endl;

	out.close();

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

bool saveALNFile(std::string filename, std::vector<std::string>& v_meshNames, std::vector<Eigen::Matrix4f> v_mats) {

	Eigen::Matrix4f trans_mat;

	ofstream myfile;  // The write stream
	myfile.open(filename, std::ios_base::out);  // Open the file

	if (!myfile) { return false; } // File opening failed 

	else {
		myfile << v_mats.size() << std::endl;

		for (int i = 0; i < v_mats.size(); i++) {

			//boost::filesystem::path p(v_meshNames[i]);
			//std::cout << "filename and extension : " << p.filename() << std::endl; // file.ext
			//std::cout << "filename only          : " << p.stem() << std::endl;     // file
			//std::string stem = p.stem().string(); // filename only
			std::string name = v_meshNames[i];

			myfile << name + ".ply" << std::endl;
			myfile << "#" << std::endl;

			for (int row = 0; row < 4; row++) {
				myfile << v_mats[i](row, 0) << " " << v_mats[i](row, 1) << " " << v_mats[i](row, 2) << " " << v_mats[i](row, 3) << std::endl;
			}

		}

		myfile << "0" << std::endl;
	}

	return true;

}
