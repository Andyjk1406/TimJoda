/**
* File created by Andrew Keeling, University of Leeds
* For research in conjunction with Tim Yoda, Basel, Switzerland
* Load two pairs of upper and lower scans
* Clip one of the scans using a regional radius (eg 10mm)
* Align (ICP point-to-plane) to the base scan and save the transformation
* Use a preselected point on base upper and find equivalent point on test upper (closest point)
* Clip lower test by 10mm radius and refine the alignment to the base lower
* Grab a pre-selected closest point as above for the lower
* Transform the lower point and upper point back by inv() trans of the fine alignments
* Measure point to point distances
* Save the distances and the 3D coordinates and the clipped meshes in the fine-aligned-to-upper coordinate system
*/

#define PCL_NO_PRECOMPILE

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
#include <vtkImplicitSelectionLoop.h>
#include <vtkClipPolyData.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCellLocator.h>
#include <vtkImplicitPolyDataDistance.h>
#include <vtkSphereSource.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>

// Types
typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool loadClipFunction(vtkSmartPointer<vtkPoints> pts, std::string filename);

bool fileExists(const char *fileName);

void TestPointNormals(vtkSmartPointer<vtkPolyData>& polydata);
bool GetPointNormals(vtkSmartPointer<vtkPolyData>& polydata);

int main(int argc, char* argv[])
{

	vtkSmartPointer<vtkPolyData> upper_base = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> lower_base = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> upper_test = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> lower_test = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();;
	
	std::string upper_filename_base, lower_filename_base;
	std::string upper_filename_test, lower_filename_test;
	std::string clp_filename, pnt_filename;
	float clip_radius;
	int upper_base_point_idx, lower_base_point_idx;
	std::string tooth_name;

	// Load the 4 PLY files, uppers and lowers, a clip radius and upper and lower seed points from base mesh
	if (argc == 9)
	{

		// Assumes the '.ply' has been given
		upper_filename_test = argv[1];
		lower_filename_test = argv[2];
		upper_filename_base = argv[3];
		lower_filename_base = argv[4];

		clip_radius = atof(argv[5]);
		upper_base_point_idx = atoi(argv[6]);
		lower_base_point_idx = atoi(argv[7]);
		tooth_name = argv[8];

		// Load the polydatas
		vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
		reader->SetFileName(upper_filename_test.c_str());
		std::cout << "Loading : " << upper_filename_test << std::endl;
		reader->Update();
		upper_test->DeepCopy(reader->GetOutput());

		reader->SetFileName(lower_filename_test.c_str());
		std::cout << "Loading : " << lower_filename_test << std::endl;
		reader->Update();
		lower_test->DeepCopy(reader->GetOutput());

		reader->SetFileName(upper_filename_base.c_str());
		std::cout << "Loading : " << upper_filename_base << std::endl;
		reader->Update();
		upper_base->DeepCopy(reader->GetOutput());

		reader->SetFileName(lower_filename_base.c_str());
		std::cout << "Loading : " << lower_filename_base << std::endl;
		reader->Update();
		lower_base->DeepCopy(reader->GetOutput());

	}
	else
	{
		cout << "Usage : ...exe upper_test.ply lower_test.ply upper_base.ply lower_base.ply clip_radius upper_base_point_idx lower_base_point_idx tooth_name";
		return 0;
	}

	// Check the vtk normals and add point normals if needed
	TestPointNormals(upper_test);
	TestPointNormals(lower_test);
	TestPointNormals(upper_base);
	TestPointNormals(lower_base);

	// Create PCL clouds for alignment (object and scene uppers and lowers)
	PointCloudT::Ptr test_cloud_upper(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(upper_test, *test_cloud_upper);
	PointCloudT::Ptr base_cloud_upper(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(upper_base, *base_cloud_upper);
	PointCloudT::Ptr test_cloud_lower(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(lower_test, *test_cloud_lower);
	PointCloudT::Ptr base_cloud_lower(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(lower_base, *base_cloud_lower);

	// Now crop/filter the test upper and lower by the clip_radius to create 'local' clouds (about the size of a single tooth)
	// Neighbors within radius search
	std::cout << "Clipping test surfaces..." << std::endl;
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(test_cloud_upper);
	PointT searchPoint_base_upper = base_cloud_upper->points[upper_base_point_idx];
	pcl::PointIndices::Ptr inliers_upper(new pcl::PointIndices());
	std::vector<float> pointRadiusSquaredDistance;
	if (kdtree.radiusSearch(searchPoint_base_upper, clip_radius, inliers_upper->indices, pointRadiusSquaredDistance) == 0)
	{
		std::cout << "No matching points found - clipping upper failed" << std::endl;
		return -1;
	}

	// Extract the inliers
	pcl::ExtractIndices<PointT> extract;
	PointCloudT::Ptr clipped_upper_test_cloud(new PointCloudT);
	extract.setInputCloud(test_cloud_upper);
	extract.setIndices(inliers_upper);
	extract.setNegative(false);
	extract.filter(*clipped_upper_test_cloud);

	kdtree.setInputCloud(test_cloud_lower);
	PointT searchPoint_base_lower = base_cloud_lower->points[lower_base_point_idx];
	pcl::PointIndices::Ptr inliers_lower(new pcl::PointIndices());
	pointRadiusSquaredDistance.clear();
	if (kdtree.radiusSearch(searchPoint_base_lower, clip_radius, inliers_lower->indices, pointRadiusSquaredDistance) == 0)
	{
		std::cout << "No matching points found - clipping lower failed" << std::endl;
	return -1;
	}

	// Extract the inliers
	pcl::ExtractIndices<PointT> extract_lower;
	pcl::PointCloud<PointT>::Ptr clipped_lower_test_cloud(new PointCloudT);
	extract_lower.setInputCloud(test_cloud_lower);
	extract_lower.setIndices(inliers_lower);
	extract_lower.setNegative(false);
	extract_lower.filter(*clipped_lower_test_cloud);

	// Now we have upper and lower test clipped pointclouds

	// Align upper test clipped cloud to upper base cloud///////////////////////////////////////////////////////////////////////////
	Eigen::Matrix4f icpMatrix_upper;
	PointCloudT::Ptr icp_alignedCloud(new PointCloudT);

	//pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
	pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
	icp.setMaximumIterations(1000);
	icp.setTransformationEpsilon(1e-12);
	icp.setEuclideanFitnessEpsilon(1e-12);
	icp.setInputSource(clipped_upper_test_cloud);
	icp.setInputTarget(base_cloud_upper);

	icp.setMaxCorrespondenceDistance(0.2); // Start with 0.5mm search zone
	icp.align(*icp_alignedCloud);
	icpMatrix_upper = icp.getFinalTransformation();
	icp.setMaxCorrespondenceDistance(0.05); // The a 0.05mm search zone
	icp.align(*icp_alignedCloud, icpMatrix_upper);
	icpMatrix_upper = icp.getFinalTransformation();
	//.........................................................................................................

	// Convert the transformation into a vtk readable form
	vtkSmartPointer<vtkMatrix4x4> VTKmatrix_upper = vtkSmartPointer<vtkMatrix4x4>::New();
	for (unsigned int r = 0; r < 4; r++) {
		for (unsigned int c = 0; c < 4; c++)
			VTKmatrix_upper->Element[r][c] = icpMatrix_upper(r, c);
	}

	// Transform the test polydata............................................................................
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(VTKmatrix_upper);

	vtkSmartPointer<vtkPolyData> upper_test_transformed = vtkSmartPointer<vtkPolyData>::New();
	
	vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter->SetTransform(transform);
	filter->SetInputData(upper_test);
	filter->Update();
	upper_test_transformed->DeepCopy(filter->GetOutput());
	//............................................................................................................

	// Same for lower - align it to the lower_base////////////////////////////////////////////////////////////////////////////
	Eigen::Matrix4f icpMatrix_lower;
	PointCloudT::Ptr icp_alignedCloud_lower(new PointCloudT);

	//pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
	pcl::IterativeClosestPointWithNormals<PointT, PointT> icp_lower;
	icp_lower.setMaximumIterations(1000);
	icp_lower.setTransformationEpsilon(1e-12);
	icp_lower.setEuclideanFitnessEpsilon(1e-12);
	icp_lower.setInputSource(clipped_lower_test_cloud);
	icp_lower.setInputTarget(base_cloud_lower);

	icp_lower.setMaxCorrespondenceDistance(0.2); // Start with 0.5mm search zone
	icp_lower.align(*icp_alignedCloud_lower);
	icpMatrix_lower = icp_lower.getFinalTransformation();
	icp_lower.setMaxCorrespondenceDistance(0.05); // The a 0.05mm search zone
	icp_lower.align(*icp_alignedCloud_lower, icpMatrix_lower);
	icpMatrix_lower = icp_lower.getFinalTransformation();
	//.........................................................................................................

	// Convert the transformation into a vtk readable form
	vtkSmartPointer<vtkMatrix4x4> VTKmatrix_lower = vtkSmartPointer<vtkMatrix4x4>::New();
	for (unsigned int r = 0; r < 4; r++) {
		for (unsigned int c = 0; c < 4; c++)
			VTKmatrix_lower->Element[r][c] = icpMatrix_lower(r, c);
	}

	// Transform the test polydata............................................................................
	vtkSmartPointer<vtkTransform> transform_lower = vtkSmartPointer<vtkTransform>::New();
	transform_lower->SetMatrix(VTKmatrix_lower);

	vtkSmartPointer<vtkPolyData> lower_test_transformed = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkTransformPolyDataFilter> filter_lower = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter_lower->SetTransform(transform_lower);
	filter_lower->SetInputData(lower_test);
	filter_lower->Update();
	lower_test_transformed->DeepCopy(filter_lower->GetOutput());
	//............................................................................................................

	// We now have the transformed test polydatas (upper and lower), precisely aligned to one tooth in the upper and one in the lower
	// So we can select a topologically matching point from test and base scans

	// First, we need the signed distance from the base seed upper point to the upper test (and save the upper test point)
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilterUpperToUpper = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilterUpperToUpper->SetTolerance(1e-12);
	distanceFilterUpperToUpper->SetInput(upper_test_transformed);
	double test_point_upper_transformed[3];
	double seed_upper[3];
	seed_upper[0] = searchPoint_base_upper.x; seed_upper[1] = searchPoint_base_upper.y; seed_upper[2] = searchPoint_base_upper.z;
	//double signedDistanceBase = distanceFilter2->EvaluateFunction(seed);
	double signedDistanceUpperToUpper = distanceFilterUpperToUpper ->EvaluateFunctionAndGetClosestPoint(seed_upper, test_point_upper_transformed);

	// Next, we need the same for the lower to lower
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilterLowerToLower = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilterLowerToLower->SetTolerance(1e-12);
	distanceFilterLowerToLower->SetInput(lower_test_transformed);
	double test_point_lower_transformed[3];
	double seed_lower[3];
	seed_lower[0] = searchPoint_base_lower.x; seed_lower[1] = searchPoint_base_lower.y; seed_lower[2] = searchPoint_base_lower.z;
	//double signedDistanceBase = distanceFilter2->EvaluateFunction(seed);
	double signedDistanceLowerToLower = distanceFilterLowerToLower->EvaluateFunctionAndGetClosestPoint(seed_lower, test_point_lower_transformed);

	// Transform the upper and lower test points back to their original positions using their respective inverse transformations
	PointT pcl_pt_upper_transformed, pcl_upper_pt_back_to_start;
	pcl_pt_upper_transformed.x = test_point_upper_transformed[0];
	pcl_pt_upper_transformed.y = test_point_upper_transformed[1];
	pcl_pt_upper_transformed.z = test_point_upper_transformed[2];
	Eigen::Matrix4f u_inv = icpMatrix_upper.inverse();
	Eigen::Affine3f aff_t_upper(u_inv);
	pcl_upper_pt_back_to_start = pcl::transformPoint(pcl_pt_upper_transformed, aff_t_upper);

	PointT pcl_pt_lower_transformed, pcl_lower_pt_back_to_start;
	pcl_pt_lower_transformed.x = test_point_lower_transformed[0];
	pcl_pt_lower_transformed.y = test_point_lower_transformed[1];
	pcl_pt_lower_transformed.z = test_point_lower_transformed[2];
	Eigen::Matrix4f l_inv = icpMatrix_lower.inverse();
	Eigen::Affine3f aff_t_lower(l_inv);
	pcl_lower_pt_back_to_start = pcl::transformPoint(pcl_pt_lower_transformed, aff_t_lower);

	double parity_distance_check = pcl::euclideanDistance(pcl_pt_upper_transformed, pcl_pt_lower_transformed);
	double test_distance = pcl::euclideanDistance(pcl_upper_pt_back_to_start, pcl_lower_pt_back_to_start);
	double base_distance = pcl::euclideanDistance(searchPoint_base_upper, searchPoint_base_lower);

	double upper_location_error = pcl::euclideanDistance(pcl_pt_upper_transformed, searchPoint_base_upper);
	double lower_location_error = pcl::euclideanDistance(pcl_pt_lower_transformed, searchPoint_base_lower);

	double upper_local_motion_error = pcl::euclideanDistance(pcl_pt_upper_transformed, pcl_upper_pt_back_to_start);
	double lower_local_motion_error = pcl::euclideanDistance(pcl_pt_lower_transformed, pcl_lower_pt_back_to_start);



	// Report
	std::cout << "Base point : " << seed_upper[0] << ", " << seed_upper[1] << ", " << seed_upper[2] << std::endl;
	std::cout << "Base distance : " << base_distance << std::endl;
	std::cout << "Base opposing point : " << seed_lower[0] << ", " << seed_lower[1] << ", " << seed_lower[2] << std::endl;
	std::cout << "Test point : " << pcl_upper_pt_back_to_start.x << ", " << pcl_upper_pt_back_to_start.y << ", " << pcl_upper_pt_back_to_start.z << std::endl;
	std::cout << "Test distance : " << test_distance << std::endl;
	std::cout << "Test opposing point : " << pcl_lower_pt_back_to_start.x << ", " << pcl_lower_pt_back_to_start.y << ", " << pcl_lower_pt_back_to_start.z << std::endl;
	std::cout << "Test parity distance : " << parity_distance_check << std::endl;
	std::cout << "Test upper point in base coord system : " << pcl_pt_upper_transformed.x << ", " << pcl_pt_upper_transformed.y << ", " << pcl_pt_upper_transformed.z << std::endl;
	std::cout << "Test lower point in base coord system : " << pcl_pt_lower_transformed.x << ", " << pcl_pt_lower_transformed.y << ", " << pcl_pt_lower_transformed.z << std::endl;
	std::cout << "Upper location error " << upper_location_error << std::endl;
	std::cout << "Lower location error " << lower_location_error << std::endl;
	std::cout << "Upper local motion error " << upper_local_motion_error << std::endl;
	std::cout << "Lower local motion error " << lower_local_motion_error << std::endl;


	// Save the data
	ofstream out;
	if (!fileExists("DataResults.txt"))
	{
		out.open("DataResults.txt", ios::app);
		out << "Tooth,Base,Test,Base_point_x,Base_point_y,Base_point_z,Base_distance,Base_opposing_point_x,Base_opposing_point_y,Base_opposing_point_z,Test_point_x,Test_point_y,Test_point_z,Test_distance,Test_opposing_point_x,Test_opposing_point_y,Test_opposing_point_z, Test_trans_to_base_x,Test_trans_to_base_y,Test_trans_to_base_z,Test_trans_to_base_opposing_x,Test_trans_to_base_opposing_y,Test_trans_to_base_opposing_z, upper_location_error, lower_location_error, upper_local_motion, lower_local_motion" << std::endl;

	}
	else {
		out.open("DataResults.txt", ios::app);
	}

	out << tooth_name << "," << upper_filename_base << "," << upper_filename_test;
	out << "," << seed_upper[0] << "," << seed_upper[1] << "," << seed_upper[2];
	out << "," << base_distance;
	out << "," << seed_lower[0] << "," << seed_lower[1] << "," << seed_lower[2];
	out << "," << pcl_upper_pt_back_to_start.x << "," << pcl_upper_pt_back_to_start.y << "," << pcl_upper_pt_back_to_start.z;
	out << "," << test_distance;
	out << "," << pcl_lower_pt_back_to_start.x << "," << pcl_lower_pt_back_to_start.y << "," << pcl_lower_pt_back_to_start.z;
	out << "," << pcl_pt_upper_transformed.x << "," << pcl_pt_upper_transformed.y << "," << pcl_pt_upper_transformed.z; 
	out << "," << pcl_pt_lower_transformed.x << "," << pcl_pt_lower_transformed.y << "," << pcl_pt_lower_transformed.z;
	out << "," << upper_location_error << "," << lower_location_error;
	out << "," << upper_local_motion_error << "," << lower_local_motion_error << std::endl;

	out.close();

	// For completeness, we will save all the PLYs here

	// Upper clip aligned to upper base
	pcl::PLYWriter pcl_plywriter;
	std::string upper_filename_test_clip = upper_filename_test;
	upper_filename_test_clip.erase(upper_filename_test_clip.length() - 4);
	upper_filename_test_clip += "_clip_" + tooth_name + ".ply";
	pcl_plywriter.write(upper_filename_test_clip, *icp_alignedCloud, true, false);

	// Lower clip aligned to lower base
	std::string lower_filename_test_clip = lower_filename_test;
	lower_filename_test_clip.erase(lower_filename_test_clip.length() - 4);
	lower_filename_test_clip += "_clip_" + tooth_name + ".ply";
	pcl_plywriter.write(lower_filename_test_clip, *icp_alignedCloud_lower, true, false);

	// Create spheres to show where the base and test points were and save them
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetRadius(0.3);
	sphere->SetCenter(seed_upper);
	sphere->Update();

	// Upper base seed point
	std::string upper_filename_base_sphere = upper_filename_base;
	upper_filename_base_sphere.erase(upper_filename_base_sphere.length() - 4);
	upper_filename_base_sphere += "_sphere_" + tooth_name + ".ply";
	vtkNew<vtkPLYWriter> writerPLY;
	writerPLY->SetFileName(upper_filename_base_sphere.c_str());
	writerPLY->SetInputData(sphere->GetOutput());
	writerPLY->Update();
	writerPLY->Write();

	// Lower base point
	sphere->SetCenter(seed_lower);
	sphere->Update();
	std::string lower_filename_base_sphere = lower_filename_base;
	lower_filename_base_sphere.erase(lower_filename_base_sphere.length() - 4);
	lower_filename_base_sphere += "_sphere_" + tooth_name + ".ply";
	writerPLY->SetFileName(lower_filename_base_sphere.c_str());
	writerPLY->SetInputData(sphere->GetOutput());
	writerPLY->Update();
	writerPLY->Write();

	// Upper test point
	sphere->SetCenter(test_point_upper_transformed);
	sphere->Update();
	std::string upper_filename_test_sphere = upper_filename_test;
	upper_filename_test_sphere.erase(upper_filename_test_sphere.length() - 4);
	upper_filename_test_sphere += "_sphere_" + tooth_name + ".ply";
	writerPLY->SetFileName(upper_filename_test_sphere.c_str());
	writerPLY->SetInputData(sphere->GetOutput());
	writerPLY->Update();
	writerPLY->Write();

	//Lower test point
	sphere->SetCenter(test_point_lower_transformed);
	sphere->Update();
	std::string lower_filename_test_sphere = lower_filename_test;
	lower_filename_test_sphere.erase(lower_filename_test_sphere.length() - 4);
	lower_filename_test_sphere += "_sphere_" + tooth_name + ".ply";
	writerPLY->SetFileName(lower_filename_test_sphere.c_str());
	writerPLY->SetInputData(sphere->GetOutput());
	writerPLY->Update();
	writerPLY->Write();

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


bool loadClipFunction(vtkSmartPointer<vtkPoints> pts, std::string filename)
{
	pts->Reset();

	ifstream myFile(filename, ios::binary);

	if (myFile.is_open())
	{

		int n = 0;
		myFile.read(reinterpret_cast<char *>(&n), sizeof(n));

		for (int i = 0; i < n; i++) {

			double p[3];
			myFile.read(reinterpret_cast<char *>(&p), sizeof(p));
			pts->InsertNextPoint(p[0], p[1], p[2]);

		}

		myFile.close();

		for (int i = 0; i < pts->GetNumberOfPoints(); i++) {
			double p[3];
			pts->GetPoint(i, p);
			cout << "Point : " << p[0] << "," << p[1] << "," << p[2] << endl;
		}

		return true;
	}

	return false;

}

void TestPointNormals(vtkSmartPointer<vtkPolyData>& polydata)
{
	std::cout << "In TestPointNormals: " << polydata->GetNumberOfPoints() << std::endl;
	// Try to read normals directly
	bool hasPointNormals = GetPointNormals(polydata);

	if (!hasPointNormals)
	{
		std::cout << "No point normals were found. Computing normals..." << std::endl;

		// Generate normals
		vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
#if VTK_MAJOR_VERSION <= 5
		normalGenerator->SetInput(polydata);
#else
		normalGenerator->SetInputData(polydata);
#endif
		normalGenerator->ComputePointNormalsOn();
		normalGenerator->ComputeCellNormalsOff();
		normalGenerator->Update();
		/*
		// Optional settings
		normalGenerator->SetFeatureAngle(0.1);
		normalGenerator->SetSplitting(1);
		normalGenerator->SetConsistency(0);
		normalGenerator->SetAutoOrientNormals(0);
		normalGenerator->SetComputePointNormals(1);
		normalGenerator->SetComputeCellNormals(0);
		normalGenerator->SetFlipNormals(0);
		normalGenerator->SetNonManifoldTraversal(1);
		*/

		polydata = normalGenerator->GetOutput();

		// Try to read normals again
		hasPointNormals = GetPointNormals(polydata);

		std::cout << "On the second try, has point normals? " << hasPointNormals << std::endl;

	}
	else
	{
		std::cout << "Point normals were found!" << std::endl;
	}
}

bool GetPointNormals(vtkSmartPointer<vtkPolyData>& polydata)
{
	std::cout << "In GetPointNormals: " << polydata->GetNumberOfPoints() << std::endl;
	std::cout << "Looking for point normals..." << std::endl;

	// Count points
	vtkIdType numPoints = polydata->GetNumberOfPoints();
	std::cout << "There are " << numPoints << " points." << std::endl;

	// Count triangles
	vtkIdType numPolys = polydata->GetNumberOfPolys();
	std::cout << "There are " << numPolys << " polys." << std::endl;

	////////////////////////////////////////////////////////////////
	// Double normals in an array
	vtkDoubleArray* normalDataDouble =
		vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));

	if (normalDataDouble)
	{
		int nc = normalDataDouble->GetNumberOfTuples();
		std::cout << "There are " << nc
			<< " components in normalDataDouble" << std::endl;
		return true;
	}

	////////////////////////////////////////////////////////////////
	// Double normals in an array
	vtkFloatArray* normalDataFloat =
		vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));

	if (normalDataFloat)
	{
		int nc = normalDataFloat->GetNumberOfTuples();
		std::cout << "There are " << nc
			<< " components in normalDataFloat" << std::endl;
		return true;
	}

	////////////////////////////////////////////////////////////////
	// Point normals
	vtkDoubleArray* normalsDouble =
		vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetNormals());

	if (normalsDouble)
	{
		std::cout << "There are " << normalsDouble->GetNumberOfComponents()
			<< " components in normalsDouble" << std::endl;
		return true;
	}

	////////////////////////////////////////////////////////////////
	// Point normals
	vtkFloatArray* normalsFloat =
		vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());

	if (normalsFloat)
	{
		std::cout << "There are " << normalsFloat->GetNumberOfComponents()
			<< " components in normalsFloat" << std::endl;
		return true;
	}

	/////////////////////////////////////////////////////////////////////
	// Generic type point normals
	vtkDataArray* normalsGeneric = polydata->GetPointData()->GetNormals(); //works
	if (normalsGeneric)
	{
		std::cout << "There are " << normalsGeneric->GetNumberOfTuples()
			<< " normals in normalsGeneric" << std::endl;

		double testDouble[3];
		normalsGeneric->GetTuple(0, testDouble);

		std::cout << "Double: " << testDouble[0] << " "
			<< testDouble[1] << " " << testDouble[2] << std::endl;

		// Can't do this:
		/*
		float testFloat[3];
		normalsGeneric->GetTuple(0, testFloat);

		std::cout << "Float: " << testFloat[0] << " "
		<< testFloat[1] << " " << testFloat[2] << std::endl;
		*/
		return true;
	}


	// If the function has not yet quit, there were none of these types of normals
	std::cout << "Normals not found!" << std::endl;
	return false;

}

