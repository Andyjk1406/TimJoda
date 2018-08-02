# TimJoda
The occlusion project in vitro with Tim Joda from Basel 2018

YodaOcclusion.exe takes as input an upper and a lower stl (no file tag), and measures the distance between the meshes, storing a signed distance value at each vertex in both meshes
It then saves vtp (VTK) versions of the STLs, which contain a Scalar matrix <double> containing these inter-occlusal distances

ScreenShot.exe has several functionalities
a) If only a .vtp file is provided (created from YodaOcclusion.exe) [Note no file tag!] then this becomes a viewer program that enables us to set three things:
i) A camera viewing position. Move the view to a position weher you want a screenshot, then press 'c'. The current camera position is saved in the current working directory as 'VTKCamera.vmx'. You should copy this to another location and rename it to, for example, upper_occlusal_view.vmx
ii) A point to measure from. Move the view and press 'p' while hovering over, for example, a cusp tip. This will save the location of the cusp tip as Point.pnt. Again copy and rename appropriately.
iii) A line from the camera penetrating the selected point. When 'p' is pressed, a ray is also saved from the camera position to the point on the mesh (and beyond) as Line.vln. Copy and rename. This might be useful later

b) If a .vtp file AND a .vmx file are provided (in that order) then the system will just save the pre-designed screenshot as [vtpfilename].png
c) If a .vtp file AND a .vmx file AND a [customfilename].png filename are provided (in that order) then the screenshot will be saved as [customfilename].png

This functionality is useful for creating standard views of aligned meshes. A scale bar is also present so the system assumes the vtp files hold Scalars for surface deviation

AutoAlign.exe takes in upper.vtp lower.vtp and scene_upper.vtp
It adds vertex normals if needed, then auto aligns. Various parameters can be adjusted as described if you just run the exe with no filenames.
It autoaligns the input upper to the scene upper, then it also transforms the input lower by the same amount (to maintain the occlusal relationship). Then it saves transformed vtl and ply versions (the ply versions are useful for Meshlab visualisation)

ClipandMeasure.exe requires a .clp file ( a vtkSelctionLoop ) that we create currently using the GUI for ToothWearMonitor that I have written for Soirse O'Toole at Kings:

      Usage : ClipandMeasure.exe upper_test.ply lower_test.ply upper_base.ply lower_base.ply clipFile.clp seed.pnt
      
This loads a pair of U/L aligned full arches, applies the clip to the upper_test.ply (eg UL7 occlusal), fine aligns the occlusal surface to the upper_base.ply and transforms the full test.ply clouds accordingly.
It then measures the distance from seed point to opposing arch for the base scan. And from the closest point to seed point on the test arch (hopefully by now, very close indeed and topologically similar) to the opposing arch.
The data is saved/appended in DataResults.txt, so copy this file and rename it.


