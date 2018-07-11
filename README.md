# TimJoda
The occlusion project in vitro with Tim Joda from Basel 2018

The first program (YodaOcclusion.exe) takes as input an upper and a lower stl (no file tag), and measures the distance between the meshes, stroing a signed distance value at each vertex in both meshes
It then saves vtp (VTK) versions of the STLs, which contain a Scalar matrix <double> containing these inter-occlusal distances
