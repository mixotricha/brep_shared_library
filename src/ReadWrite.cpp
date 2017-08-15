#include <gp.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp_Explorer.hxx>
#include <Poly_Triangulation.hxx>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <OSD_Path.hxx>
#include <OSD_OpenFile.hxx>
#include <RWStl.hxx>

#include <StlAPI_Writer.hxx>

#include <StlMesh_Mesh.hxx>
#include <StlTransfer.hxx>

#include <BRep_Tool.hxx>
#include <BRepTools.hxx>

#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>

#include <BRepFeat_MakeCylindricalHole.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>

#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepOffsetAPI_Sewing.hxx>

// Brep To CGAL conversion 
#include <PrintUtils.h>
#include <ReadWrite.h>
 
// Streams 
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Math
#include <math.h>
#include <float.h>
#include <cmath>
#include <assert.h>

using namespace std;

ReadWrite::ReadWrite() {
}

// Write BREP 
std::string ReadWrite::WriteBREP(const TopoDS_Shape& shape)
{
		//std::cout << "Generating BREP" << std::endl; 
		std::stringstream stream;
    BRepTools::Write(shape,stream);		
		return stream.str(); 
}

// Read BREP 
TopoDS_Shape ReadWrite::ReadBREP(std::string brep)
{
		//std::cout << "Reading BREP" << std::endl; 
		BRep_Builder brepb;

		std::stringstream stream;
		TopoDS_Shape shape;
		stream << brep << std::endl; 
    BRepTools::Read(shape,stream,brepb);		
		return shape; 
}

// Write as STL
void ReadWrite::WriteSTL(const TopoDS_Shape& shape)
{
    StlAPI_Writer stl_writer;
		BRepMesh_IncrementalMesh ( shape, 0.01);	
    stl_writer.Write(shape, "demo1.stl");		
}

// Write a brep out to an STL string 
char* ReadWrite::ConvertBrepTostring(char *brep,float quality) { 
	
	// Tolerances 
	Standard_Real tolerance = quality;
  Standard_Real angular_tolerance = 0.5;
  Standard_Real minTriangleSize = Precision::Confusion();

	TopoDS_Shape shape = ReadBREP(brep);
	StlAPI_Writer stl_writer;

	// Set the tolerances
	BRepMesh_FastDiscret::Parameters m_MeshParams;
  m_MeshParams.ControlSurfaceDeflection = Standard_True; 
  m_MeshParams.Deflection = tolerance;
  m_MeshParams.MinSize = minTriangleSize;
  m_MeshParams.InternalVerticesMode = Standard_False;
  m_MeshParams.Relative=Standard_False;
  m_MeshParams.Angle = angular_tolerance;

	BRepMesh_IncrementalMesh ( shape, m_MeshParams );	
	char *new_buf = strdup((char*)stl_writer.Dump(shape).c_str());			
  return new_buf; 
}

