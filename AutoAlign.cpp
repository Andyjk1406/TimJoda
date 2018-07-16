/**
* File created by Andrew Keeling, University of Leeds
* Auto Align all arches into same coordinate frame. 
* For research in conjunction with Tim Yoda, Basel, Switzerland
*/

#define PCL_NO_PRECOMPILE

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <vtkSTLWriter.h>
#include <vtkSTLReader.h>
#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>

#include <fstream>
#include <boost/algorithm/string.hpp> // include Boost, a C++ library

void TestPointNormals(vtkSmartPointer<vtkPolyData>& polydata);
bool GetPointNormals(vtkSmartPointer<vtkPolyData>& polydata);

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::PFHSignature125 FeatureT;
//typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PFHEstimation<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

bool saveALNFile(std::string filename, std::vector<std::string>& v_meshNames, std::vector<Eigen::Matrix4f> v_mats);
void saveEigen4f(Eigen::Matrix4f m, std::string f_name);
void loadEigen4f(Eigen::Matrix4f m, std::string f_name);


// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{

	// Get input object and scene
	if (argc < 4)
	{
		pcl::console::print_error("Syntax is: %s object_upper(.vtp) object_lower(.vtp) scene_upper(.vtp)\n  [Optional -inv_norm 1 (to invert scene normals) -grid 0.02 (downsample grid size)\n", argv[0]);
		pcl::console::print_error("-inlier_thresh 0.25 (SCIA inlier threshold) -grid 0.02 (downsample grid size)  -iter 50000 (n iterations for SCIA)\n");
		pcl::console::print_error("-feat_rad 0.9 (feature radius) -edge_sim 0.5 (edge length similarity 0 to 1)  -corr_rand 5 (correspondence randomness)\n");
		pcl::console::print_error("-inlier_fract 0.25 (fraction of inliers)]\n");
		return (1);
	}

	// Load the VTK vtp files
	std::string objectUpper_fname = argv[1];
	std::string objectLower_fname = argv[2];
	std::string sceneUpper_fname = argv[3];

	std::string objectUpper_fname_vtp = objectUpper_fname + ".vtp";
	std::string objectLower_fname_vtp = objectLower_fname + ".vtp";
	std::string sceneUpper_fname_vtp = sceneUpper_fname + ".vtp";

	std::string objectUpperTrans_fname_vtp = objectUpper_fname + "_trans.vtp";
	std::string objectLowerTrans_fname_vtp = objectLower_fname + "_trans.vtp";

	std::string objectUpperTrans_fname_ply = objectUpper_fname + "_trans.ply";
	std::string objectLowerTrans_fname_ply = objectLower_fname + "_trans.ply";

	vtkSmartPointer<vtkPolyData> objectUpper = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> objectLower = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> sceneUpper = vtkSmartPointer<vtkPolyData>::New();

	vtkNew<vtkPolyData> objectUpper_transformed;
	vtkNew<vtkPolyData> objectLower_transformed;

	// Load the upper with the distance scalars as a vtp file, and the lower and the scene upper to which we will align
	vtkNew<vtkXMLPolyDataReader> reader;
	reader->SetFileName(objectUpper_fname_vtp.c_str());
	reader->Update();
	objectUpper->DeepCopy(reader->GetOutput());

	reader->SetFileName(objectLower_fname_vtp.c_str());
	reader->Update();
	objectLower->DeepCopy(reader->GetOutput());

	reader->SetFileName(sceneUpper_fname_vtp.c_str());
	reader->Update();
	sceneUpper->DeepCopy(reader->GetOutput());

	// Check the vtk normals and add point normals if needed
	std::cout << "Object Upper Mesh :" << std::endl;
	TestPointNormals(objectUpper);
	std::cout << "Scene Upper Mesh :" << std::endl;
	TestPointNormals(sceneUpper);

	// Create PCL clouds for alignment (object and scene uppers)
	PointCloudT::Ptr object_cloud(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(objectUpper, *object_cloud);
	PointCloudT::Ptr scene_cloud(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(sceneUpper, *scene_cloud);


  // Point clouds
  PointCloudT::Ptr object_aligned (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  PointCloudT::Ptr object_d(new PointCloudT);
  PointCloudT::Ptr scene_d(new PointCloudT);

  bool invert_normals = false;
  float vgrid = 0.2;
  int n_iter = 500;
  float feat_rad = 0.9;
  float edge_sim = 0.6;
  int corr_rand = 1;
  float inlier_fraction = 0.3;
  float inlier_thresh = 2.5f * vgrid;

  std::vector<std::string> v_filenames;
  std::vector<Eigen::Matrix4f> v_mats(2, Eigen::Matrix4f::Identity());

  pcl::console::parse_argument(argc, argv, "-inv_norm", invert_normals);
  pcl::console::parse_argument(argc, argv, "-grid", vgrid);
  inlier_thresh = 2.5* vgrid;

  pcl::console::parse_argument(argc, argv, "-iter", n_iter);
  pcl::console::parse_argument(argc, argv, "-feat_rad", feat_rad);
  pcl::console::parse_argument(argc, argv, "-edge_sim", edge_sim);
  pcl::console::parse_argument(argc, argv, "-corr_rand", corr_rand);
  pcl::console::parse_argument(argc, argv, "-inlier_fract", inlier_fraction);
  pcl::console::parse_argument(argc, argv, "-inlier_thresh", inlier_thresh);
  pcl::console::print_highlight("Invert normals = %i\n", invert_normals);
  pcl::console::print_highlight("Voxel grid size : %f\n", vgrid);
  pcl::console::print_highlight("SCIA iterations : %i\n", n_iter);
  pcl::console::print_highlight("Feature radius : %f\n", feat_rad);
  pcl::console::print_highlight("Edge Length Similarity Ratio : %f\n", edge_sim);
  pcl::console::print_highlight("Correspondence Randomness = %i\n", corr_rand);
  pcl::console::print_highlight("Inlier Fraction = %f\n", inlier_fraction);
  pcl::console::print_highlight("Inlier Threshold SCIA = %f\n", inlier_thresh);


  /*
  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimation<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.5);
  nest.setInputCloud(scene);
  nest.compute(*scene);

  pcl::console::print_highlight("Estimating object normals...\n");
  //pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.5);
  nest.setInputCloud(object);
  nest.compute(*object);
  */
  
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = vgrid;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object_cloud);
  grid.filter (*object_d);
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud (scene_cloud);
  grid.filter (*scene_d);
  
  // Invert normals for scene (this might be better than inverting the normals for the crowns
  if (invert_normals) {
	  pcl::console::print_highlight("Inverting scene normals...\n");
	  for (size_t i = 0; i < scene_d->size(); i++) {
		  scene_d->points[i].normal_x *= -1;
		  scene_d->points[i].normal_y *= -1;
		  scene_d->points[i].normal_z *= -1;
	  }
	  for (size_t i = 0; i < scene_cloud->size(); i++) { // For the fine ICP
		  scene_cloud->points[i].normal_x *= -1;
		  scene_cloud->points[i].normal_y *= -1;
		  scene_cloud->points[i].normal_z *= -1;
	  }
  }

  //pcl::visualization::PCLVisualizer visu("Alignment"); // Set up display first
 
  //for (feat_rad = 0.5; feat_rad < 1.5; feat_rad += 0.2) {

	  // Estimate features
	  pcl::console::print_highlight("Estimating features with radius %f...\n",feat_rad);
	  FeatureEstimationT fest;
	  fest.setRadiusSearch(feat_rad); // ALSO IMPORTANT TO GET RIGHT 0.9mm for prep fit surface, 1.8
	  fest.setInputCloud(object_d);
	  fest.setInputNormals(object_d);
	  //fest.setSearchSurface(object);
	  fest.compute(*object_features);
	  fest.setInputCloud(scene_d);
	  fest.setInputNormals(scene_d);
	//  fest.setSearchSurface(scene);
	  fest.compute(*scene_features);

	//  for (edge_sim = 0.5; edge_sim < 1; edge_sim += 0.1) {

	//	  for (corr_rand = 1; corr_rand < 10; corr_rand += 2) {

	// Perform alignment
	  
		  pcl::console::print_highlight("Starting alignment with edge_sim = %f and corr_ran = %i...\n", edge_sim, corr_rand);
		  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
		  align.setInputSource(object_d);
		  align.setSourceFeatures(object_features);
		  align.setInputTarget(scene_d);
		  align.setTargetFeatures(scene_features);
		  align.setMaximumIterations(n_iter); // Number of RANSAC iterations
		  align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
		  align.setCorrespondenceRandomness(corr_rand); // Number of nearest features to use5
		  align.setSimilarityThreshold(edge_sim); // Polygonal edge length similarity threshold !!!!! THIS ONE MADE THE DIFFERENCE !!!!0.5
		  align.setMaxCorrespondenceDistance(inlier_thresh); // Inlier threshold (was 2.5f * leaf)
		  align.setInlierFraction(inlier_fraction); // Required inlier fraction for accepting a pose hypothesis
	  
		  {
			pcl::ScopeTime t("Alignment");
			align.align(*object_aligned);
		  }
		  
		  if (align.hasConverged())
			  {
				  // Print results
				  printf("\n");
				  Eigen::Matrix4f transformation = align.getFinalTransformation();
				  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
				  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
				  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
				  pcl::console::print_info("\n");
				  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
				  pcl::console::print_info("\n");
				  pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object_d->size());

				  // Show alignment
				  {
					 
					  
					  pcl::visualization::PCLVisualizer visu("Alignment");
					  visu.removeAllPointClouds();
					  visu.addPointCloud(scene_d, ColorHandlerT(scene_d, 0.0, 255.0, 0.0), "scene");
					  visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
					  visu.resetCamera();

					  std::string object_screenshot = objectUpper_fname + ".png";
					 // while (!visu.wasStopped())
					 // {
						  visu.spinOnce();
						//  visu.saveScreenshot(object_screenshot); // doesnt work!
					 // }
					 
				  
					  // Screenshot  
					  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
						  vtkSmartPointer<vtkWindowToImageFilter>::New();

					  visu.getRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();
					  
					  windowToImageFilter->SetInput(visu.getRenderWindow());
					  windowToImageFilter->SetMagnification(1); //set the resolution of the output image (1 times the current resolution of vtk render window)
					  windowToImageFilter->SetInputBufferTypeToRGB(); //also record the alpha (transparency) channel
					  //windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer Not for this one - otherwise its sketchy
					  windowToImageFilter->Modified();
					  windowToImageFilter->Update();

					  vtkSmartPointer<vtkPNGWriter> writer =
						  vtkSmartPointer<vtkPNGWriter>::New();
					  writer->SetFileName(object_screenshot.c_str());
					  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
					  writer->Write();
					  // Save a screenshot
					 
					 // object_screenshot.replace(object_screenshot.length() - 4, 4, "_IA_FRad" + std::to_string(feat_rad) + "_edgeSim" + std::to_string(edge_sim) + "corrRand" + std::to_string(corr_rand) + ".png");
					  //object_screenshot.replace(object_screenshot.length() - 4, 4, "_IA_.png");
					  

					  //visu.close();
				  }

				  // Align source to target using GICP
				  Eigen::Matrix4f icpMatrix;
				  PointCloudT::Ptr icp_alignedCloud(new PointCloudT);

				  pcl::GeneralizedIterativeClosestPoint<PointNT, PointNT> icp;
				  icp.setMaximumIterations(100);
				  icp.setTransformationEpsilon(1e-12);
				  icp.setEuclideanFitnessEpsilon(1e-12);
				  icp.setInputSource(object_d);
				  icp.setInputTarget(scene_d);

				  icp.setMaxCorrespondenceDistance(2); // Start with 2mm search zone
				  icp.align(*icp_alignedCloud, transformation);
				  icpMatrix = icp.getFinalTransformation();

				  icp.setMaxCorrespondenceDistance(0.3); // Reduce to 0.3mm search zone
				  icp.align(*icp_alignedCloud, icpMatrix);
				  icpMatrix = icp.getFinalTransformation();

				  // Convert the transformation into a vtk readable form
				  vtkSmartPointer<vtkMatrix4x4> VTKmatrix = vtkSmartPointer<vtkMatrix4x4>::New();
				  for (unsigned int r = 0; r < 4; r++) {
					  for (unsigned int c = 0; c < 4; c++)
						  VTKmatrix->Element[r][c] = icpMatrix(r, c);
				  }

				  // Transform the object polydatas
				  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
				  transform->SetMatrix(VTKmatrix);

				  vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
				  filter->SetTransform(transform);
				  filter->SetInputData(objectUpper);
				  filter->Update();
				  objectUpper_transformed->DeepCopy(filter->GetOutput());

				  filter->SetInputData(objectLower);
				  filter->Update();
				  objectLower_transformed->DeepCopy(filter->GetOutput());

				  // Save the transformed polydatas
				  vtkNew<vtkXMLPolyDataWriter> writer;
				  writer->SetFileName(objectUpperTrans_fname_vtp.c_str());
				  writer->SetInputData(objectUpper_transformed);
				  writer->Write();
				  writer->SetFileName(objectLowerTrans_fname_vtp.c_str());
				  writer->SetInputData(objectLower_transformed);
				  writer->Update();
				  writer->Write();

				  // and PLYs so we can check in Meshlab
				  vtkNew<vtkPLYWriter> writerPLY;
				  writerPLY->SetFileName(objectUpperTrans_fname_ply.c_str());
				  writerPLY->SetInputData(objectUpper_transformed);
				  writerPLY->Write();
				  writerPLY->SetFileName(objectLowerTrans_fname_ply.c_str());
				  writerPLY->SetInputData(objectLower_transformed);
				  writerPLY->Update();
				  writerPLY->Write();
			  }
			
			  else
			  {
				  pcl::console::print_error("Alignment failed!\n");
				  //return (1);
			  }

  
  return (0);
}

bool saveALNFile(std::string filename, std::vector<std::string>& v_meshNames, std::vector<Eigen::Matrix4f> v_mats) {

	Eigen::Matrix4f trans_mat;

	ofstream myfile;  // The write stream
	myfile.open(filename, std::ios_base::out);  // Open the file

	if (!myfile) { return false; } // File opening failed 

	else {
		myfile << v_mats.size() << std::endl;

		for (int i = 0; i < v_mats.size(); i++) {

			boost::filesystem::path p(v_meshNames[i]);
			//std::cout << "filename and extension : " << p.filename() << std::endl; // file.ext
			//std::cout << "filename only          : " << p.stem() << std::endl;     // file
			std::string stem = p.stem().string(); // filename only

			myfile << stem+".ply" << std::endl;
			myfile << "#" << std::endl;

			for (int row = 0; row < 4; row++) {
				myfile << v_mats[i](row, 0) << " " << v_mats[i](row, 1) << " " << v_mats[i](row, 2) << " " << v_mats[i](row, 3) << std::endl;
			}

		}

		myfile << "0" << std::endl;
	}

	return true;

}



void saveEigen4f(Eigen::Matrix4f m, std::string f_name) {

	ofstream myFile2(f_name, ios::binary);

	if (myFile2.is_open()) {
		myFile2.write((const char*)&m(0, 0), 16 * sizeof(float));
	}
	myFile2.close();
}


void loadEigen4f(Eigen::Matrix4f m, std::string f_name) {

	ifstream myFile2(f_name, ios::binary);

	if (myFile2.is_open()) {
		myFile2.read((char*)&m(0, 0), 16 * sizeof(float));
	}
	myFile2.close();
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

