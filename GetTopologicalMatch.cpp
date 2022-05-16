/**
* File created by Andrew Keeling, University of Leeds
* For research in conjunction with Tim Yoda, Basel, Switzerland and extended to our Leeds experiments
* Load a source and target scans (full arch generally) which are broadly aligned
* Clip source using a regional radius (eg 10mm)
* Align using VTK ICP, the clipped source to the target
* Use a preselected point on target and find equivalent point on source (closest point)
* Add this point to the source (InsertPoint) and move it back to original position and save
* Grab a pre-selected closest point as above for the lower
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

#include <iostream>
using namespace std;

// Types
typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool loadClipFunction(vtkSmartPointer<vtkPoints> pts, std::string filename);

bool fileExists(const char *fileName);

void TestPointNormals(vtkSmartPointer<vtkPolyData>& polydata);
bool GetPointNormals(vtkSmartPointer<vtkPolyData>& polydata);

int main(int argc, char* argv[])
{

	vtkSmartPointer<vtkPolyData> source_mesh = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> target_mesh = vtkSmartPointer<vtkPolyData>::New();
	
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();;
	
	std::string target_filename_base, source_filename_base;
	float clip_radius;
	int target_base_point_idx, source_new_point_idx;
	std::string tooth_name;

	// Load the 2 PLY files, a clip radius and a seed point for target
	if (argc == 5)
	{

		// Assumes the '.ply' has been given
		target_filename_base = argv[1];
		source_filename_base = argv[2];

		clip_radius = atof(argv[3]);
		target_base_point_idx = atoi(argv[4]);

		// Load the polydatas
		vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
		reader->SetFileName(target_filename_base.c_str());
		std::cout << "Loading : " << target_filename_base << std::endl;
		reader->Update();
		target_mesh->DeepCopy(reader->GetOutput());

		reader->SetFileName(source_filename_base.c_str());
		std::cout << "Loading : " << source_filename_base << std::endl;
		reader->Update();
		source_mesh->DeepCopy(reader->GetOutput());

	}
	else
	{
		cout << "Usage : ...exe target_mesh.ply source_mesh.ply clip_radius target_base_point_idx";
		return 0;
	}

	// Check the vtk normals and add point normals if needed
	TestPointNormals(target_mesh);
	TestPointNormals(source_mesh);

	// Create PCL clouds for alignment (object and scene uppers and lowers)
	PointCloudT::Ptr source_cloud(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(source_mesh, *source_cloud);
	PointCloudT::Ptr target_cloud(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(target_mesh, *target_cloud);
	
	// Now crop/filter the test upper and lower by the clip_radius to create 'local' clouds (about the size of a single tooth)
	// Neighbors within radius search
	std::cout << "Clipping test surface..." << std::endl;
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(source_cloud);
	PointT searchPoint_target = target_cloud->points[target_base_point_idx];
	pcl::PointIndices::Ptr inliers_source(new pcl::PointIndices());
	std::vector<float> pointRadiusSquaredDistance;
	if (kdtree.radiusSearch(searchPoint_target, clip_radius, inliers_source->indices, pointRadiusSquaredDistance) == 0)
	{
		std::cout << "No matching points found - clipping source failed" << std::endl;
		return -1;
	}

	// Extract the inliers
	pcl::ExtractIndices<PointT> extract;
	PointCloudT::Ptr clipped_source_cloud(new PointCloudT);
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers_source);
	extract.setNegative(false);
	extract.filter(*clipped_source_cloud);


	// Now we have source clipped pointclouds

	// Align source clipped cloud to upper base cloud /////////////////////////////////////////////////////////////////////
	Eigen::Matrix4f icpMatrix_source;
	PointCloudT::Ptr icp_alignedCloud(new PointCloudT);

	//pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
	pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
	icp.setMaximumIterations(1000);
	icp.setTransformationEpsilon(1e-12);
	icp.setEuclideanFitnessEpsilon(1e-12);
	icp.setInputSource(clipped_source_cloud);
	icp.setInputTarget(target_cloud);

	icp.setMaxCorrespondenceDistance(0.2); // Start with 0.5mm search zone
	icp.align(*icp_alignedCloud);
	icpMatrix_source = icp.getFinalTransformation();
	icp.setMaxCorrespondenceDistance(0.05); // The a 0.05mm search zone
	icp.align(*icp_alignedCloud, icpMatrix_source);
	icpMatrix_source = icp.getFinalTransformation();
	//.........................................................................................................

	// Convert the transformation into a vtk readable form
	vtkSmartPointer<vtkMatrix4x4> VTKmatrix_source = vtkSmartPointer<vtkMatrix4x4>::New();
	for (unsigned int r = 0; r < 4; r++) {
		for (unsigned int c = 0; c < 4; c++)
			VTKmatrix_source->Element[r][c] = icpMatrix_source(r, c);
	}

	// Transform the test polydata............................................................................
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(VTKmatrix_source);

	vtkSmartPointer<vtkPolyData> source_mesh_transformed = vtkSmartPointer<vtkPolyData>::New();
	
	vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter->SetTransform(transform);
	filter->SetInputData(source_mesh);
	filter->Update();
	source_mesh_transformed->DeepCopy(filter->GetOutput());
	//............................................................................................................

	// We now have the transformed source polydata, precisely aligned to one tooth 
	// So we can select a topologically matching point from source and target

	// First, we need the signed distance from the target seed point to the transformed source (and save the new source point)
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilterTargetToSource = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilterTargetToSource->SetTolerance(1e-12);
	distanceFilterTargetToSource->SetInput(source_mesh_transformed);
	double source_point_transformed[3];
	double seed_upper[3];
	seed_upper[0] = searchPoint_target.x; seed_upper[1] = searchPoint_target.y; seed_upper[2] = searchPoint_target.z;
	//double signedDistanceBase = distanceFilter2->EvaluateFunction(seed);
	double signedDistanceUpperToUpper = distanceFilterTargetToSource ->EvaluateFunctionAndGetClosestPoint(seed_upper, source_point_transformed);

	
	// Transform the new source point back to its original position using the inverse transformation
	PointT pcl_pt_source_transformed, pcl_source_pt_back_to_start;
	pcl_pt_source_transformed.x = source_point_transformed[0];
	pcl_pt_source_transformed.y = source_point_transformed[1];
	pcl_pt_source_transformed.z = source_point_transformed[2];
	Eigen::Matrix4f u_inv = icpMatrix_source.inverse();
	Eigen::Affine3f aff_t_source(u_inv);
	pcl_source_pt_back_to_start = pcl::transformPoint(pcl_pt_source_transformed, aff_t_source);

	// Insert the point into the original source mesh (no cell will be associated!) and save
	const double p[3] = { pcl_source_pt_back_to_start.x, pcl_source_pt_back_to_start.y, pcl_source_pt_back_to_start.z };
	source_mesh->GetPoints()->InsertNextPoint(p);

	// Save
	source_filename_base.erase(source_filename_base.length() - 4);
	source_filename_base += "_new.ply";
	vtkSmartPointer<vtkPLYWriter> poly_writer = vtkSmartPointer<vtkPLYWriter>::New();
	poly_writer->SetInputData(source_mesh);
	poly_writer->SetFileTypeToBinary();
	poly_writer->SetFileName(source_filename_base.c_str());
	poly_writer->SetArrayName("RGB");
	poly_writer->Write();

	

	// Report
	double location_error = pcl::euclideanDistance(pcl_pt_source_transformed, searchPoint_target);
	double local_motion = pcl::euclideanDistance(pcl_pt_source_transformed, pcl_source_pt_back_to_start);
	std::cout << "Index of new source point : " << source_mesh->GetNumberOfPoints() - 1 << std::endl;
	std::cout << "Base point : " << seed_upper[0] << ", " << seed_upper[1] << ", " << seed_upper[2] << std::endl;
	std::cout << "Topological location error : " << location_error << std::endl;
	std::cout << "Point motion magnitude : " << local_motion << std::endl;




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

