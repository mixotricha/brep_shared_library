// ==========================================================
// MAKERTRON Procedural Cad System  
// Damien V Towning 
// 2017

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL Damien Towning BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// ==========================================================


#include <iostream>

#include <gp.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <BRepFill_PipeShell.hxx>

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <TDocStd_Document.hxx>
//#include <Handle_TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
//#include <Handle_XCAFApp_Application.hxx>
#include <XCAFDoc_ShapeTool.hxx>
//#include <Handle_XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
//#include <STEPCAFControl_Writer.hxx>
#include <StlAPI_Writer.hxx>
#include <BRepTools.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepFeat_MakeCylindricalHole.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepOffsetAPI_Sewing.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <ShapeAnalysis.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeAnalysis_ShapeContents.hxx>
#include <ShapeFix.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Wireframe.hxx>
#include <BRepCheck_Analyzer.hxx> 
#include <TopExp_Explorer.hxx>

// Streams 
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Math
#include <cmath>
#include <assert.h>

// Namespaces

using namespace std;

extern "C" char *sphere(float radius, float x , float y , float z );
extern "C" char *box(float x , float y , float z , float xs , float ys , float zs);
extern "C" char *cone(float r1,float r2,float h,float z);
extern "C" char *polyhedron(int **faces,float *points,int f_length); 
extern "C" char *difference(char *a, char*b);
extern "C" char *uni(char *a, char*b);
extern "C" char *intersection(char *a, char*b);
extern "C" char *convert_brep_tostring(char *brep,float quality);
extern "C" char *translate(float x , float y , float z , char *a);
extern "C" char *rotateX(float x , char *a);
extern "C" char *rotateY(float y , char *a);
extern "C" char *rotateZ(float z , char *a);
extern "C" char *circle(float r1);
extern "C" char *extrude(float h1, char *a);
extern "C" char *cylinder(float r1,float h,float z);


std::string test()
{
	return std::string("hello"); 
}

std::string process_request(char *text)
{
	return std::string("hello"); 
}


// Write BREP 
std::string Write_BREP(const TopoDS_Shape& shape)
{
		//std::cout << "Generating BREP" << std::endl; 
		stringstream stream;
    BRepTools::Write(shape,stream);		
		return stream.str(); 
}

// Read BREP 
TopoDS_Shape Read_BREP(std::string brep)
{
		//std::cout << "Reading BREP" << std::endl; 
		BRep_Builder brepb;

		stringstream stream;
		TopoDS_Shape shape;
		stream << brep << std::endl; 
    BRepTools::Read(shape,stream,brepb);		
		return shape; 
}

// Write as STL
void Write_STL(const TopoDS_Shape& shape)
{
    StlAPI_Writer stl_writer;
		BRepMesh_IncrementalMesh ( shape, 0.01);	
    stl_writer.Write(shape, "demo1.stl");		
}

// Write a brep out to an STL string 
char *convert_brep_tostring(char *brep,float quality) { 
	TopoDS_Shape shape = Read_BREP(brep);
	StlAPI_Writer stl_writer;
	BRepMesh_IncrementalMesh ( shape, quality);	
	char *new_buf = strdup((char*)stl_writer.Dump(shape).c_str());			
  return new_buf; 
}

// Translate a brep 
char *translate( float x , float y , float z , char *a ) { 
	//std::cout << "Translating object" << std::endl; 
	TopoDS_Shape shape_a = Read_BREP(a);
	gp_Trsf translate;
  translate.SetTranslation(gp_Vec(x, y, z));
	TopoDS_Shape transformed = BRepBuilderAPI_Transform(shape_a, translate, false).Shape();
	char *new_buf = strdup((char*)Write_BREP(transformed).c_str());	
	return new_buf; 
}

// Rotate brep x axis 
char *rotateX( float x , char *a ) { 
	//std::cout << "Rotating object" << std::endl; 
	Standard_Real xr = x;
	TopoDS_Shape shape_a = Read_BREP(a);
	gp_Trsf rotate;
  rotate.SetRotation( gp::OX(), xr );
	TopoDS_Shape transformed = BRepBuilderAPI_Transform(shape_a, rotate).Shape();
	char *new_buf = strdup((char*)Write_BREP(transformed).c_str());	
	return new_buf; 
}

// Rotate brep y axis 
char *rotateY( float y , char *a ) { 
	//std::cout << "Rotating object" << std::endl; 
	Standard_Real yr = y;
	TopoDS_Shape shape_a = Read_BREP(a);
	gp_Trsf rotate;
  rotate.SetRotation( gp::OY(), yr );
	TopoDS_Shape transformed = BRepBuilderAPI_Transform(shape_a, rotate).Shape();
	char *new_buf = strdup((char*)Write_BREP(transformed).c_str());	
	return new_buf; 
}

// Rotate brep z axis 
char *rotateZ( float z , char *a ) { 
	//std::cout << "Rotating object" << std::endl; 
	Standard_Real zr = z;
	TopoDS_Shape shape_a = Read_BREP(a);
	gp_Trsf rotate;
  rotate.SetRotation( gp::OZ(), zr );
	TopoDS_Shape transformed = BRepBuilderAPI_Transform(shape_a, rotate).Shape();
	char *new_buf = strdup((char*)Write_BREP(transformed).c_str());	
	return new_buf; 
}

// Prim Sphere
char *sphere(float radius, float x , float y , float z ) {
	//std::cout << "Making Sphere" << radius << " " << x << " " << y << " " << z << std::endl;  	  	
	Standard_Real sphere_radius = radius;
	gp_Ax2 sphere_origin = gp_Ax2(gp_Pnt(x,y,z), gp_Dir(0,0,1));
	TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(sphere_origin, sphere_radius ).Shape();
	char *new_buf = strdup((char*)Write_BREP(sphere).c_str());	
	return new_buf; 
} 

// Prim box 
char *box(float x , float y , float z , float xs , float ys , float zs) {
	//std::cout << "Making Box" << xs << " " << ys << " " << zs << std::endl;  	  	
	Standard_Real box_xs = xs;
	Standard_Real box_ys = ys;
	Standard_Real box_zs = zs;
	gp_Ax2 box_origin = gp_Ax2(gp_Pnt(x,y,z), gp_Dir(0,0,1));
	TopoDS_Shape box = BRepPrimAPI_MakeBox(     box_origin,   box_xs , box_ys  , box_zs ).Shape();
	char *new_buf = strdup((char*)Write_BREP(box).c_str());	
	return new_buf; 
} 


// Prim Cylinder
char *cylinder(float r1, float h,float z) {
	//std::cout << "Making Cylinder" << r1 << " " << h << std::endl;  	  	
	Standard_Real cylinder_r1 = r1;
	Standard_Real cylinder_h  = h;
	Standard_Real cylinder_z = z; 
	gp_Ax2 cylinder_origin = gp_Ax2(gp_Pnt(0,0,cylinder_z), gp_Dir(0,0,1));

	TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder( cylinder_origin , cylinder_r1, cylinder_h ).Shape();
	char *new_buf = strdup((char*)Write_BREP(cylinder).c_str());	
	return new_buf; 
} 

// Prim cone 
char *cone(float r1,float r2,float h,float z) {
	//std::cout << "Making Cone" << r1 << " " << r2 << " " << h << std::endl;  	  	
	Standard_Real cone_r1 = r1;
	Standard_Real cone_r2 = r2;
	Standard_Real cone_h  = h;
	Standard_Real cone_z = z; 
	gp_Ax2 cone_origin = gp_Ax2(gp_Pnt(0,0,cone_z), gp_Dir(0,0,1));
	TopoDS_Shape cone = BRepPrimAPI_MakeCone( cone_origin , cone_r1, cone_r2 , cone_h ).Shape();
	char *new_buf = strdup((char*)Write_BREP(cone).c_str());	
	return new_buf; 
} 

// Prim 2d circle 
char *circle(float r1) { 
	Standard_Real radius = r1;
	gp_Circ c = gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)), radius );
	TopoDS_Edge Ec = BRepBuilderAPI_MakeEdge(c);
	TopoDS_Wire Wc = BRepBuilderAPI_MakeWire(Ec);
	TopoDS_Face F = BRepBuilderAPI_MakeFace(Wc);
	char *new_buf = strdup((char*)Write_BREP(F).c_str());	
	return new_buf; 	
}

char *polyhedron(int **faces,float *points,int f_length) {
	BRepOffsetAPI_Sewing sew(0.1);
	Standard_Integer a,b; 
	Standard_Integer width = 0; 
	Standard_Real x1, y1, z1;
	Standard_Real x2, y2, z2; 
	TopoDS_Wire wire;		
	BRep_Builder builder;
	std::vector<TopoDS_Vertex> foo; 
	for ( int i = 0; i < f_length; i++ ) { // through face sets
		width = faces[i][0]; 		
		for ( int ii = 1; ii < width; ii++ ) { // make list of points
			a = faces[i][ii]; 
			x1 = points[(a*3)+0]; y1 = points[(a*3)+1]; z1 = points[(a*3)+2];
			foo.push_back( BRepBuilderAPI_MakeVertex(gp_Pnt(x1,y1,z1)) ); 	
		}
		builder.MakeWire(wire);			
		for ( int iii = 0; iii < foo.size()-1; iii++ ) { // build wire from points in foo for a face set
			builder.Add(wire, BRepBuilderAPI_MakeEdge(foo[iii],foo[iii+1]));
		}
		builder.Add(wire, BRepBuilderAPI_MakeEdge(foo[foo.size()-1],foo[0])); // closing wire for face set 
		sew.Add( BRepBuilderAPI_MakeFace( wire ) ); // add to sewing object 
		foo.clear(); 
	}
	 
	sew.Perform(); // sew it together
	TopoDS_Shape obj = sew.SewedShape(); // get the shape
  
	BRepBuilderAPI_MakeSolid brep_solid(TopoDS::Shell(obj)); // Now some unclear foo that took a bit to find 
																													 // Yes the shape will show type three as a shell 
																													 // but you have to wrap it TopoDS::shel() anyway :| 	
	TopoDS_Solid solid = brep_solid.Solid();

	char *new_buf = strdup((char*)Write_BREP(solid).c_str());	
	return new_buf; 
}

// Extrusion ( does not include twist yet ) 
char *extrude(float h1,char *a) { 
	Standard_Real height = h1;
	TopoDS_Shape shape_a = Read_BREP(a);
	TopoDS_Shape S4 = BRepPrimAPI_MakePrism(shape_a,gp_Vec(0,0,height));
	char *new_buf = strdup((char*)Write_BREP(S4).c_str());	
	return new_buf; 	
}

// Difference between two objects
char *difference(char *a,char *b) { 
	//std::cout << "Generating Difference" << std::endl; 
	TopoDS_Shape shape_a = Read_BREP(a);
	TopoDS_Shape shape_b = Read_BREP(b);
	TopoDS_Shape boolean_result;
	boolean_result = BRepAlgoAPI_Cut( shape_a ,  shape_b ).Shape();
	char *new_buf = strdup((char*)Write_BREP(boolean_result).c_str());	
	return new_buf; 
}

// Union between two objects
char *uni(char *a,char *b) { 
	//std::cout << "Generating Union" << std::endl; 
	TopoDS_Shape shape_a = Read_BREP(a);
	TopoDS_Shape shape_b = Read_BREP(b);
	TopoDS_Shape boolean_result;
	boolean_result = BRepAlgoAPI_Fuse( shape_a ,  shape_b ).Shape();	
	char *new_buf = strdup((char*)Write_BREP(boolean_result).c_str());	
	return new_buf; 
}

// Intersection between two objects
char *intersection(char *a,char *b) { 
	//std::cout << "Generating Intersection" << std::endl; 
	TopoDS_Shape shape_a = Read_BREP(a);
	TopoDS_Shape shape_b = Read_BREP(b);
	TopoDS_Shape boolean_result = BRepAlgoAPI_Common( shape_a ,  shape_b ).Shape();
	char *new_buf = strdup((char*)Write_BREP(boolean_result).c_str());	
	return new_buf; 
}

 

