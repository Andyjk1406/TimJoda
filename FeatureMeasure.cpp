/**
* File created by Andrew Keeling, University of Leeds
* For research in conjunction with Tim Yoda, Basel, Switzerland
* Load two pairs of upper and lower scans (broadly aligned already)
* Also load a reference point index for the master scan
* Calculate a feature for the reference point
* Find the matching feature on the target cloud within a small search distance
* Find closest point on equivalent lowers
* Save the distances and the 3D coordinates
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
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>


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
	double seed[3]; // The seed point from which to take the measurements

	std::string upper_filename_base, lower_filename_base;
	std::string upper_filename_test, lower_filename_test;
	int seed_point_idx;
	std::string tooth_name;

	std::cout << argc << std::endl;
	
	// Load the 4 PLY files, uppers and lowers and get index for the source seed point and the name for the data (eg UL7)
	if (argc == 7)
	{

		// Assumes the '.ply' has been given
		upper_filename_test = argv[1];
		lower_filename_test = argv[2];
		upper_filename_base = argv[3];
		lower_filename_base = argv[4];

		seed_point_idx = std::atoi(argv[5]);
		tooth_name = argv[6];

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

		std::cout << "Loaded 4 meshes and using seed index " << seed_point_idx << std::endl;

	}
	else
	{
		cout << "Usage : ...exe upper_test.ply lower_test.ply upper_base.ply lower_base.ply [seed_point_index] [tooth_name (eg UL7)]";
		return 0;
	}

	// Check the vtk normals and add point normals if needed
	TestPointNormals(upper_test);
	TestPointNormals(lower_test);
	TestPointNormals(upper_base);
	TestPointNormals(lower_base);

	// Create PCL clouds for feature extraction
	std::cout << "Converting clouds";
	PointCloudT::Ptr test_cloud_upper(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(upper_test, *test_cloud_upper);
	PointCloudT::Ptr base_cloud_upper(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(upper_base, *base_cloud_upper);

	// Feature parameters
	float feature_search_radius = 0.6;
	float base_to_test_max_distance = 0.3;

	//iss_salient_radius_ = 6 * model_resolution;
	//iss_non_max_radius_ = 4 * model_resolution;
	double support_radius = 6. * 0.1;
	double nms_radius = 4. * 0.1;
	pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
	iss_detector.setSalientRadius(support_radius);
	iss_detector.setNonMaxRadius(nms_radius);
	iss_detector.setInputCloud(base_cloud_upper);
	iss_detector.compute(*keypoints);

	std::cout << keypoints->points.size() << " keypoints found " << std::endl;
	pcl::PLYWriter writer;
	std::string upper_filename_keypoints = upper_filename_base;
	upper_filename_keypoints.erase(upper_filename_keypoints.length() - 4);
	upper_filename_keypoints += "_KPs_.ply";
	writer.write(upper_filename_keypoints, *keypoints, true, false);
	

	// Neighbors within radius search
	std::cout << "Performing feature calculation...." << std::endl;
	std::vector<int> indices;
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(base_cloud_upper);

	// Find the point closest to the keypoint
	PointT searchPoint_baseKP;
	searchPoint_baseKP.x = 22.958363; // 27.4266;   
	searchPoint_baseKP.y = 34.338516;// 31.05331;
	searchPoint_baseKP.z = 2.144796;// 2.784599;
	int Kn = 1;
	std::vector<int> pointIdxNKNSearchKP(Kn);
	std::vector<float> pointNKNSquaredDistanceKP(Kn);
	if (kdtree.nearestKSearch(searchPoint_baseKP, Kn, pointIdxNKNSearchKP, pointNKNSquaredDistanceKP) == 0)
	{
		std::cout << "No matching point found" << std::endl;
		return -1;
	}

	seed_point_idx = pointIdxNKNSearchKP[0];


	// Try SHOT for full clouds
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_base(new pcl::PointCloud<pcl::SHOT352>());
	pcl::SHOTEstimation<PointT, PointT, pcl::SHOT352> describer_base;
	describer_base.setRadiusSearch(feature_search_radius);
	describer_base.setInputCloud(base_cloud_upper);
	describer_base.setInputNormals(base_cloud_upper);
	//describer.setSearchSurface(cloud);
	describer_base.compute(*descriptors_base);

	pcl::SHOT352 shot_base = descriptors_base->points[seed_point_idx];

	// Test cloud SHOT
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_test(new pcl::PointCloud<pcl::SHOT352>());
	pcl::SHOTEstimation<PointT, PointT, pcl::SHOT352> describer_test;
	describer_test.setRadiusSearch(feature_search_radius);
	describer_test.setInputCloud(test_cloud_upper);
	describer_test.setInputNormals(test_cloud_upper);
	//describer.setSearchSurface(cloud);
	describer_test.compute(*descriptors_test);


	
	//describer.computePointSHOT()

	PointT searchPoint_base = base_cloud_upper->points[seed_point_idx];
	std::vector<float> pointRadiusSquaredDistance;

	if (kdtree.radiusSearch(searchPoint_base, feature_search_radius, indices, pointRadiusSquaredDistance) < 10) // presume we need at least 10 points 
	{
		std::cout << "Error calculating base feature - not enough neighbours found" << std::endl;
		return -1;
	}

	// Calculate the base cloud feature using the index
	Eigen::VectorXf base_pfh_histogram;
	base_pfh_histogram.setZero(125);
	pcl::PFHEstimation<PointT, PointT, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(base_cloud_upper);
	pfh.setInputNormals(base_cloud_upper);

	// Compute a single feature for the neighbourhood given by indices
	pfh.computePointPFHSignature(*base_cloud_upper, *base_cloud_upper, indices, 5, base_pfh_histogram);
	
	// Find all possible test points close to the base point
	kdtree.setInputCloud(test_cloud_upper);
	std::vector<int> test_point_indices;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	if (kdtree.radiusSearch(searchPoint_base, base_to_test_max_distance, inliers->indices, pointRadiusSquaredDistance) == 0)
	{
		std::cout << "Error finding putative test correspondences - no neighbours found" << std::endl;
		return -1;
	}
	kdtree.radiusSearch(searchPoint_base, base_to_test_max_distance, test_point_indices, pointRadiusSquaredDistance) == 0;

	std::cout << test_point_indices.size() << " possible matches found" << std::endl;

	// Use these for SHOT -closest point wins ?
	// Extract the inliers
	pcl::ExtractIndices<pcl::SHOT352> extract;
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_test_extracted(new pcl::PointCloud<pcl::SHOT352>());
	extract.setInputCloud(descriptors_test);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*descriptors_test_extracted);

	pcl::KdTreeFLANN<pcl::SHOT352> kd_feature_tree_shot;
	kd_feature_tree_shot.setInputCloud(descriptors_test_extracted);

	int Kshot = 1;
	std::vector<int> pointIdxNKNSearchShot(Kshot);
	std::vector<float> pointNKNSquaredDistanceShot(Kshot);
	if (kd_feature_tree_shot.nearestKSearch(shot_base , Kshot, pointIdxNKNSearchShot, pointNKNSquaredDistanceShot) == 0)
	{
		std::cout << "No matching histogram found" << std::endl;
		return -1;
	}

	// The index of the test match must be converted to the original cloud index
	int test_index_point_shot = test_point_indices[pointIdxNKNSearchShot[0]];


	// Create a vector of test features
	std::vector<Eigen::VectorXf> test_pfh_features;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr test_pfh_cloud(new pcl::PointCloud<pcl::PFHSignature125>);

	// Fill in the cloud data
	test_pfh_cloud->width = test_point_indices.size();
	test_pfh_cloud->height = 1;
	test_pfh_cloud->is_dense = true;
	test_pfh_cloud->points.resize(test_pfh_cloud->width * test_pfh_cloud->height);
	
	for (int i = 0; i < test_point_indices.size(); i++)
	{
		PointT sp = test_cloud_upper->points[test_point_indices[i]];
		std::vector<int> indices_for_current_feature;
		if (kdtree.radiusSearch(sp, feature_search_radius, indices_for_current_feature, pointRadiusSquaredDistance) == 0)
		{
			std::cout << "Error finding features in test cloud " << std::endl;
			return -1;
		}

		Eigen::VectorXf current_test_pfh_feature;
		current_test_pfh_feature.setZero(125);
		pfh.computePointPFHSignature(*test_cloud_upper, *test_cloud_upper, indices_for_current_feature, 5, current_test_pfh_feature);
		test_pfh_features.push_back(current_test_pfh_feature);

		// Copy into the resultant cloud
		for (int d = 0; d < current_test_pfh_feature.size(); ++d)
			test_pfh_cloud->points[i].histogram[d] = current_test_pfh_feature[d];
		
	}

	std::cout << "Features calculated for base and test. Searching for closest....";
	pcl::KdTreeFLANN<pcl::PFHSignature125> kd_feature_tree;
	kd_feature_tree.setInputCloud(test_pfh_cloud);

	pcl::PFHSignature125 pfh_hist_base;
	for (int d = 0; d < base_pfh_histogram.size(); ++d)
		pfh_hist_base.histogram[d] = base_pfh_histogram[d];

	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	if (kd_feature_tree.nearestKSearch(pfh_hist_base, K, pointIdxNKNSearch, pointNKNSquaredDistance) == 0)
	{
		std::cout << "No matching histogram found" << std::endl;
		return -1;
	}

	// The index of the test match must be converted to the original cloud index
	int test_index_point = test_point_indices[pointIdxNKNSearch[0]];

	// To check, lets print out the XYZ of the base and test point. They should be close (maybe within 0.2mm)
	std::cout << "Base point : " << searchPoint_base.x << "," << searchPoint_base.y << "," << searchPoint_base.z << std::endl;
	PointT test_point = test_cloud_upper->points[test_index_point_shot];
	std::cout << "Test point : " << test_point.x << "," << test_point.y << "," << test_point.z << std::endl;

	// We now have two points - the base upper point and the test upper point
	// Lets measure the distance to the closest opposing point for both
	
	// First, we need the signed distance from the base seed point to the opposing arch
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilter2 = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilter2->SetInput(lower_base);
	double opposing_point_base[3];
	double upper_base_seed[3];
	upper_base_seed[0] = searchPoint_base.x;
	upper_base_seed[1] = searchPoint_base.y;
	upper_base_seed[2] = searchPoint_base.z;
	double signedDistanceBase = distanceFilter2->EvaluateFunctionAndGetClosestPoint(upper_base_seed, opposing_point_base);


	// Next we use the equivalent closest point on the test mesh 
	// ie the closest thing to a corresponding point as judged by pfh histogram
	// and we find the opposing point
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilter3 = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilter3->SetInput(lower_test);
	double opposing_point_test[3];
	double upper_test_seed[3];
	upper_test_seed[0] = test_point.x;
	upper_test_seed[1] = test_point.y;
	upper_test_seed[2] = test_point.z;
	double signedDistanceTest = distanceFilter3->EvaluateFunctionAndGetClosestPoint(upper_test_seed, opposing_point_test);

	// Report
	std::cout << "Base distance : " << signedDistanceBase << std::endl;
	std::cout << "Base opposing point : " << opposing_point_base[0] << ", " << opposing_point_base[1] << ", " << opposing_point_base[2] << std::endl;
	std::cout << "Test distance : " << signedDistanceTest << std::endl;
	std::cout << "Test opposing point : " << opposing_point_test[0] << ", " << opposing_point_test[1] << ", " << opposing_point_test[2] << std::endl;

	// Save the data
	ofstream out;
	if (!fileExists("DataResults.txt"))
	{
		out.open("DataResults.txt", ios::app);
		out << "Tooth,Base,Test,Base_point_x,Base_point_y,Base_point_z,Base_distance,Base_opposing_point_x,Base_opposing_point_y,Base_opposing_point_z,Test_point_x,Test_point_y,Test_point_z,Test_distance,Test_opposing_point_x,Test_opposing_point_y,Test_opposing_point_z" << std::endl;

	}
	else {
		out.open("DataResults.txt", ios::app);
	}

	out << tooth_name << "," << upper_filename_base << "," << upper_filename_test;
	out << "," << upper_base_seed[0] << "," << upper_base_seed[1] << "," << upper_base_seed[2];
	out << "," << signedDistanceBase;
	out << "," << opposing_point_base[0] << "," << opposing_point_base[1] << "," << opposing_point_base[2];
	out << "," << upper_test_seed[0] << "," << upper_test_seed[1] << "," << upper_test_seed[2];
	out << "," << signedDistanceTest;
	out << "," << opposing_point_test[0] << "," << opposing_point_test[1] << "," << opposing_point_test[2] << std::endl;

	out.close();

	// Create spheres to show where the base and test points were and save them
	vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
	sphere->SetRadius(0.3);
	sphere->SetCenter(upper_base_seed);
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
	sphere->SetCenter(opposing_point_base);
	sphere->Update();
	std::string lower_filename_base_sphere = lower_filename_base;
	lower_filename_base_sphere.erase(lower_filename_base_sphere.length() - 4);
	lower_filename_base_sphere += "_sphere_" + tooth_name + ".ply";
	writerPLY->SetFileName(lower_filename_base_sphere.c_str());
	writerPLY->SetInputData(sphere->GetOutput());
	writerPLY->Update();
	writerPLY->Write();

	// Upper test point
	sphere->SetCenter(upper_test_seed);
	sphere->Update();
	std::string upper_filename_test_sphere = upper_filename_test;
	upper_filename_test_sphere.erase(upper_filename_test_sphere.length() - 4);
	upper_filename_test_sphere += "_sphere_" + tooth_name + ".ply";
	writerPLY->SetFileName(upper_filename_test_sphere.c_str());
	writerPLY->SetInputData(sphere->GetOutput());
	writerPLY->Update();
	writerPLY->Write();

	//Lower test point
	sphere->SetCenter(opposing_point_test);
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
		normalGenerator->SplittingOff();
		normalGenerator->ConsistencyOff();
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

