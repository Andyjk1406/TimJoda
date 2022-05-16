/**
* File created by Andrew Keeling, University of Leeds
* Take a screenshot using a specific scale bar and camera angle. 
* For research in conjunction with Tim Yoda, Basel, Switzerland
* For convenience I have also added a method to select a ray from camera view to mesh and save it
*/

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
 
#include <vtkActor.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
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
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>

#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPNGWriter.h>
#include <vtkCellPicker.h>

#include <vtkWindowToImageFilter.h>
#include <Eigen/dense>

#include <iostream>
#include <fstream>


bool fileExists(const char *fileName);
void saveVTKMat4x4(double position[3], double focal_point[3], double view_up[3], double view_angle, std::string f_name);
void loadVTKMat4x4(double position[3], double focal_point[3], double view_up[3], double& view_angle, std::string f_name);
void saveLinePoints(double start[3], double end[3], std::string f_name);
void savePoint(double point[3], std::string f_name);
void KeypressCallbackFunction(
	vtkObject* caller,
	long unsigned int eventId,
	void* clientData,
	void* callData);

using namespace std;

int main(int argc, char* argv[])
{
  	
  vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
 
  // The camera position data	
  double view_angle;
  double position[3];
  double focal_point[3];
  double view_up[3];
  bool have_camera = false;
 
  std::string filename;
  std::string filename_vtp;
  std::string filename_png;

  std::string filename_camera_data;

  // Load the 1 VTP file, (which holds distance data in their scalars)
  if (argc == 2 || argc ==3 || argc == 4)
   {

	 filename = argv[1];
	
	 filename_vtp = filename + ".vtp";
	 filename_png = filename + ".png";
	
	 vtkNew<vtkXMLPolyDataReader> reader1;
	 reader1->SetFileName(filename_vtp.c_str());
	 reader1->Update();
	 poly->DeepCopy(reader1->GetOutput());

	 std::cout << "Poly Loaded with " << poly->GetPoints()->GetNumberOfPoints() << " points" <<std::endl;

    }

  if (argc == 3 || argc==4)
  {
	  filename_camera_data = argv[2];
	  loadVTKMat4x4(position, focal_point, view_up, view_angle, filename_camera_data);
	  have_camera = true;
  }

  if (argc == 4)
  {
	  filename_png = argv[3];	 
	 
  }

  if (argc <2 || argc >4)
  {
	  cout << "Usage : ...exe filename(.vtp) [The '.vtp' will be appended automatically] camera_matrix.vmx [Optional viewing position] screenshot.png [Optional full filename for screenshot]";
	  return 0;
  }

  
  // Check Scalars
  vtkSmartPointer<vtkDoubleArray> poly_scalars; 
  poly_scalars = vtkDoubleArray::SafeDownCast(poly->GetPointData()->GetScalars());

  std::cout << "Loaded : " << poly_scalars->GetTuple1(10) << std::endl;

  // Display colour mapped image if we want
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData( poly );
 // mapper->SetScalarRange(poly->GetPointData()->GetScalars()->GetRange()[0], poly->GetPointData()->GetScalars()->GetRange()[1]);
  mapper->SetScalarRange(-0.2, 0.2);
  
 
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper( mapper );
 
  vtkSmartPointer<vtkScalarBarActor> scalarBar =  vtkSmartPointer<vtkScalarBarActor>::New();
  scalarBar->SetLookupTable(mapper->GetLookupTable());
  scalarBar->SetTitle("Distance");
  
  scalarBar->SetNumberOfLabels(5);
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
 
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer( renderer );
  renWin->SetSize(1500, 1000);

 // Need to set this up before the interactor
  if (have_camera)
  {
	  vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera();
	  cam->SetViewAngle(view_angle);
	  cam->SetPosition(position);
	  cam->SetFocalPoint(focal_point);
	  cam->SetViewUp(view_up);
	  cam->Modified();

	  renderer->Render();
	  renderer->Modified();

	  renWin->Render();
  }

 
  vtkSmartPointer<vtkRenderWindowInteractor> renWinInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renWinInteractor->SetRenderWindow( renWin );

  vtkSmartPointer<vtkCallbackCommand> keypressCallback =
	  vtkSmartPointer<vtkCallbackCommand>::New();
  keypressCallback->SetCallback(KeypressCallbackFunction);
  renWinInteractor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
	  vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); 

  renWinInteractor->SetInteractorStyle(style);
  vtkSmartPointer<vtkCellPicker> pointPicker = vtkSmartPointer<vtkCellPicker>::New();
  renWinInteractor->SetPicker(pointPicker);

 
  renderer->AddActor( actor );
  renderer->AddActor2D(scalarBar);
 
  renWin->Render();
 
  if (have_camera)
  {
	  // Screenshot  
	  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
		  vtkSmartPointer<vtkWindowToImageFilter>::New();
	 
	  windowToImageFilter->SetInput(renWin);
	  windowToImageFilter->SetScale(1); //set the resolution of the output image (1 times the current resolution of vtk render window)
	  windowToImageFilter->SetInputBufferTypeToRGB(); //don't record the alpha (transparency) channel
	  windowToImageFilter->ReadFrontBufferOn(); // read from the back buffer Not for this one - otherwise its sketchy
	  windowToImageFilter->Modified();
	  windowToImageFilter->Update();

	  vtkSmartPointer<vtkPNGWriter> writer =
		  vtkSmartPointer<vtkPNGWriter>::New();
	  writer->SetFileName(filename_png.c_str());
	  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	  writer->Write();
  }
  else
  {

	  renWinInteractor->Start();
  }


  return EXIT_SUCCESS;
}


bool fileExists(const char *fileName)
{
	ifstream infile(fileName);
	return infile.good();
}


void saveVTKMat4x4(double position[3], double focal_point[3], double view_up[3], double view_angle, std::string f_name) {

	ofstream myFile2(f_name, ios::binary);

	if (myFile2.is_open()) {
		myFile2.write((const char*)&position[0], 3 * sizeof(double));
		myFile2.write((const char*)&focal_point[0], 3 * sizeof(double));
		myFile2.write((const char*)&view_up[0], 3 * sizeof(double));
		myFile2.write((const char*)&view_angle, 1 * sizeof(double));

	}
	myFile2.close();

}


void loadVTKMat4x4(double position[3], double focal_point[3], double view_up[3], double& view_angle, std::string f_name) {

	ifstream myFile2(f_name, ios::binary);

	if (myFile2.is_open()) {
		myFile2.read((char*)&position[0], 3 * sizeof(double));
		myFile2.read((char*)&focal_point[0], 3 * sizeof(double));
		myFile2.read((char*)&view_up[0], 3 * sizeof(double));
		myFile2.read((char*)&view_angle, 1 * sizeof(double));
	}
	myFile2.close();

}

void saveLinePoints(double start[3], double end[3], std::string f_name)
{
	ofstream myFile2(f_name, ios::binary);

	if (myFile2.is_open()) {
		myFile2.write((const char*)&start[0], 3 * sizeof(double));
		myFile2.write((const char*)&end[0], 3 * sizeof(double));
		
	}
	myFile2.close();
}

void savePoint(double point[3], std::string f_name)
{
	ofstream myFile2(f_name, ios::binary);

	if (myFile2.is_open()) {
		myFile2.write((const char*)&point[0], 3 * sizeof(double));
	}
	myFile2.close();

	std::cout << "Saving point as " << f_name << std::endl;

}

void KeypressCallbackFunction(vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData))
{
	std::cout << "Keypress callback" << std::endl;

	vtkRenderWindowInteractor *iren =
		static_cast<vtkRenderWindowInteractor*>(caller);

	std::string letters = iren->GetKeySym();
	std::cout << "Pressed: " << iren->GetKeySym() << std::endl;

	if (letters == "p")
	{
		// Save the current camera position and the intersection on the surface
		vtkSmartPointer<vtkCamera> cam = iren->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();

		double position[3];
		cam->GetPosition(position);
		std::cout << "Camera Position: " << position[0] << " " << position[1] << " " << position[2] << std::endl;
		
		double picked[3];
		iren->GetRenderWindow()->GetInteractor()->GetPicker()->GetPickPosition(picked);
		std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;

		// Elongate the picked point along the line of sight
		double vec[3];
		vec[0] = picked[0] - position[0]; vec[1] = picked[1] - position[1]; vec[2] = picked[2] - position[2];
		double end[3];
		end[0] = position[0] + (5. * vec[0]); end[1] = position[1] + (5. * vec[1]); end[2] = position[2] + (5. * vec[2]);

		saveLinePoints(position, end, "Line.vln");
		savePoint(picked, "Point.pnt");

	}

	if (letters == "c")
	{
		// Save the current camera position, view_up, focal point and view_angle
		vtkSmartPointer<vtkCamera> cam = iren->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();

		double view_angle = cam->GetViewAngle();
		double position[3];
		cam->GetPosition(position);
		double focal_point[3];
		cam->GetFocalPoint(focal_point);
		double view_up[3];
		cam->GetViewUp(view_up);

		saveVTKMat4x4(position, focal_point, view_up, view_angle, "VTKCamera.vmx");

	}

	if (letters == "l")
	{
		// Load the camera position data	
		double view_angle;
		double position[3];
		double focal_point[3];
		double view_up[3];
		
		loadVTKMat4x4(position, focal_point, view_up, view_angle, "VTKCamera.vmx");

		vtkSmartPointer<vtkCamera> cam = iren->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera();
		cam->SetViewAngle(view_angle);
		cam->SetPosition(position);
		cam->SetFocalPoint(focal_point);
		cam->SetViewUp(view_up);
		cam->Modified();

		
		iren->GetRenderWindow()->Render();
		
		std::cout << "Camera data loaded" << std::endl;

	}
}

