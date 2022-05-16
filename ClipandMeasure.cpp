/**
* File created by Andrew Keeling, University of Leeds
* For research in conjunction with Tim Yoda, Basel, Switzerland
* Load two pairs of upper and lower scans
* Clip one of the scans using a loaded clip loop file (eg the occlusal surface of a molar)
* Align (GICP) to the other scan and move th lower too
* Take a standardised 3D line intersection (eg cusp tip) on both uppers
* Find closest point on equivalent lowers
* Save the distances and the 3D coordinates
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
#include <vtkImplicitSelectionLoop.h>
#include <vtkClipPolyData.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCellLocator.h>
#include <vtkImplicitPolyDataDistance.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>

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

	vtkSmartPointer<vtkPolyData> upper_base = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> lower_base = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> upper_test = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> lower_test = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();;
	double seed[3]; // The seed point from which to take the measurements

	std::string upper_filename_base, lower_filename_base;
	std::string upper_filename_test, lower_filename_test;
	std::string clp_filename, pnt_filename;

	// Load the 4 PLY files, uppers and lowers and a clip function
	if (argc == 7)
	{

		// Assumes the '.ply' has been given
		upper_filename_test = argv[1];
		lower_filename_test = argv[2];
		upper_filename_base = argv[3];
		lower_filename_base = argv[4];

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

		// Load the clip function
		clp_filename = argv[5];
		if (!loadClipFunction(pts, clp_filename)) {
			std::cout << "Error loading clip file : " << clp_filename << std::endl;
			return -1;
		}

		// Load the seed point
		pnt_filename = argv[6];
		ifstream myFile2(pnt_filename, ios::binary);
		if (myFile2.is_open()) {
			myFile2.read((char*)&seed[0], 3 * sizeof(double));
		}
		myFile2.close();

		std::cout << "Loaded seed point as " << seed[0] << "," << seed[1] << "," <<seed[2] << std::endl;

	}
	else
	{
		cout << "Usage : ...exe upper_test.ply lower_test.ply upper_base.ply lower_base.ply clipFile.clp seed.pnt";
		return 0;
	}

	// Check the vtk normals and add point normals if needed
	TestPointNormals(upper_test);
	TestPointNormals(lower_test);
	TestPointNormals(upper_base);
	TestPointNormals(lower_base);

	// Clip the test upper.........................................................................
	// Create the loop selector
	vtkNew<vtkImplicitSelectionLoop> loop_select;
	loop_select->SetLoop(pts);

	// Cut out the selected mesh
	vtkSmartPointer<vtkClipPolyData> clipPolyData = vtkSmartPointer<vtkClipPolyData>::New();
	clipPolyData->SetClipFunction(loop_select);
	clipPolyData->SetInputData(upper_test);
	clipPolyData->SetInsideOut(1);
	clipPolyData->Update();

	// Clean the mesh
	vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanPolyData->SetInputData(clipPolyData->GetOutput());
	cleanPolyData->Update();

	vtkSmartPointer<vtkPolyData> clippedPoly = vtkSmartPointer<vtkPolyData>::New();
	clippedPoly->DeepCopy(cleanPolyData->GetOutput());
	//.............................................................................................

	// Convert this to a pointcloud so we can do GICP
	TestPointNormals(clippedPoly);
	// Create PCL clouds for alignment (object and scene uppers)
	PointCloudT::Ptr test_cloud_clipped(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(clippedPoly, *test_cloud_clipped);
	PointCloudT::Ptr base_cloud(new PointCloudT);
	pcl::io::vtkPolyDataToPointCloud(upper_base, *base_cloud);

	// Align source to target using GICP........................................................................
	Eigen::Matrix4f icpMatrix;
	PointCloudT::Ptr icp_alignedCloud(new PointCloudT);

	//pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
	pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
	icp.setMaximumIterations(1000);
	icp.setTransformationEpsilon(1e-12);
	icp.setEuclideanFitnessEpsilon(1e-12);
	icp.setInputSource(test_cloud_clipped);
	icp.setInputTarget(base_cloud);

	icp.setMaxCorrespondenceDistance(0.2); // Start with 0.5mm search zone
	icp.align(*icp_alignedCloud);
	//icpMatrix = icp.getFinalTransformation();
	//icp.setMaxCorrespondenceDistance(0.05); // The a 0.05mm search zone
	//icp.align(*icp_alignedCloud, icpMatrix);
	icpMatrix = icp.getFinalTransformation();
	//.........................................................................................................

	// Convert the transformation into a vtk readable form
	vtkSmartPointer<vtkMatrix4x4> VTKmatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	for (unsigned int r = 0; r < 4; r++) {
		for (unsigned int c = 0; c < 4; c++)
			VTKmatrix->Element[r][c] = icpMatrix(r, c);
	}

	// Transform the test polydatas............................................................................
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->SetMatrix(VTKmatrix);

	vtkSmartPointer<vtkPolyData> upper_test_transformed = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> lower_test_transformed = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter->SetTransform(transform);
	filter->SetInputData(upper_test);
	filter->Update();
	upper_test_transformed->DeepCopy(filter->GetOutput());

	filter->SetInputData(lower_test);
	filter->Update();
	lower_test_transformed->DeepCopy(filter->GetOutput());
	//............................................................................................................

	// We now have the transformed test polydatas (upper and lower), precisely aligned to one tooth in the upper
	// So we can select a topologically matching point from test and base scan

	// First, we need the signed distance from the base seed point to the opposing arch
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilter2 = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilter2->SetInput(lower_base);
	double opposing_point_base[3];
	//double signedDistanceBase = distanceFilter2->EvaluateFunction(seed);
	double signedDistanceBase = distanceFilter2->EvaluateFunctionAndGetClosestPoint(seed, opposing_point_base);


	// Next we need the closest point on the test mesh patch that was aligned to the base mesh
	// ie the closest thing to a corresponding point
	/*  THIS ONE GIVES UNSIGNED DISTANCE */
	vtkSmartPointer<vtkCellLocator> distanceFilter = vtkSmartPointer<vtkCellLocator>::New();
	distanceFilter->SetDataSet(upper_test_transformed);
	distanceFilter->BuildLocator();
	double closestPoint_test[3];
	double closestPtDist2; //Squared distance
	vtkIdType cellId;
	int subId;
	distanceFilter->FindClosestPoint(seed, closestPoint_test, cellId, subId, closestPtDist2);

	// Finally we want the signed distance to the test opposing arch
	vtkSmartPointer<vtkImplicitPolyDataDistance> distanceFilter3 = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
	distanceFilter3->SetInput(lower_test_transformed);
	double opposing_point_test[3];
	double signedDistanceTest = distanceFilter3->EvaluateFunctionAndGetClosestPoint(closestPoint_test, opposing_point_test);

	// Report
	std::cout << "Base point : " << seed[0] << ", " << seed[1] << ", " << seed[2] << std::endl;
	std::cout << "Base distance : " << signedDistanceBase << std::endl;
	std::cout << "Base opposing point : " << opposing_point_base[0] << ", " << opposing_point_base[1] << ", " << opposing_point_base[2] << std::endl;
	std::cout << "Test point : " << closestPoint_test[0] << ", " << closestPoint_test[1] << ", " << closestPoint_test[2] << std::endl;
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

	out << clp_filename << "," << upper_filename_base << "," << upper_filename_test;
	out << "," << seed[0] << "," << seed[1] << "," << seed[2];
	out << "," << signedDistanceBase;
	out << "," << opposing_point_base[0] << "," << opposing_point_base[1] << "," << opposing_point_base[2];
	out << "," << closestPoint_test[0] << "," << closestPoint_test[1] << "," << closestPoint_test[2];
	out << "," << signedDistanceTest;
	out << "," << opposing_point_test[0] << "," << opposing_point_test[1] << "," << opposing_point_test[2] << std::endl;

	out.close();


	


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

