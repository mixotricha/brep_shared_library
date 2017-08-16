
/***************************************************************************
 *   Copyright (c) Damien Towning         (connolly.damien@gmail.com) 2017 *
 *                                                                         *
 *   This file is part of the Makertron CSG cad system.                    *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

// OpenCascade Includes 

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

// brep shared library includes
#include <PrintUtils.h>
#include <ReadWrite.h>
#include <BrepCgal.h>
#include <Geometry.h>
 
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

// ffi bindings 

extern "C" char* ffi_sphere(float radius, float x , float y , float z );
extern "C" char* ffi_cube(float x , float y , float z , float xs , float ys , float zs);
extern "C" char* ffi_cone(float r1,float r2,float h,float z);
extern "C" char* ffi_polyhedron(int **faces,float *points,int f_length); 
extern "C" char* ffi_difference(char *a, char*b);
extern "C" char* ffi_union(char *a, char*b);
extern "C" char* ffi_intersection(char *a, char*b);
extern "C" char* ffi_translate(float x , float y , float z , char *a);
extern "C" char* ffi_scale(float x , float y , float z , char *a);
extern "C" char* ffi_rotateX(float x , char *a);
extern "C" char* ffi_rotateY(float y , char *a);
extern "C" char* ffi_rotateZ(float z , char *a);
extern "C" char* ffi_circle(float r1);
extern "C" char* ffi_extrude(float h1, char *a);
extern "C" char* ffi_cylinder(float r1,float h,float z);
extern "C" char* ffi_minkowski(char *a, char*b);
extern "C" char* ffi_convert_brep_tostring(char *brep,float quality);

char* ffi_convert_brep_tostring(char *brep,float quality) { 
	ReadWrite readwrite;
	return readwrite.ConvertBrepTostring(brep,quality);
} 

char* ffi_sphere(float radius, float x , float y , float z ) { 
	ReadWrite readwrite; 
	Geometry geometry; 
	TopoDS_Shape shape_a; 
	geometry.sphere( radius , x , y , z , shape_a ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_cube(float x , float y , float z , float xs , float ys , float zs) { 
	ReadWrite readwrite; 
	Geometry geometry; 
	TopoDS_Shape shape_a; 
	geometry.cube( x , y , z , xs , ys , zs , shape_a ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_cylinder(float r1,float h,float z) { 
	ReadWrite readwrite; 
	Geometry geometry; 
	TopoDS_Shape shape_a; 
	geometry.cylinder( r1 , h , z , shape_a ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_circle(float r1) { 
	ReadWrite readwrite; 
	Geometry geometry; 
	TopoDS_Shape shape_a; 
	geometry.circle( r1 ,shape_a ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_cone(float r1,float r2,float h,float z) { 
	ReadWrite readwrite; 
	Geometry geometry; 
	TopoDS_Shape shape_a; 
	geometry.cone( r1 , r2 , h , z , shape_a ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_polyhedron(int **faces,float *points,int f_length) { 
	ReadWrite readwrite; 
	Geometry geometry; 
	TopoDS_Shape shape_a; 
	geometry.polyhedron( faces , points , f_length , shape_a ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_difference(char *a, char*b) { 
	ReadWrite readwrite;
	Geometry geometry; 
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	TopoDS_Shape shape_b = readwrite.ReadBREP(b);
	geometry.difference( shape_a , shape_b ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_union(char *a, char*b) { 
	ReadWrite readwrite;
	Geometry geometry; 
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	TopoDS_Shape shape_b = readwrite.ReadBREP(b);
	geometry.uni( shape_a , shape_b ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_intersection(char *a, char*b) { 
	ReadWrite readwrite;
	Geometry geometry; 
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	TopoDS_Shape shape_b = readwrite.ReadBREP(b);
	geometry.intersection( shape_a , shape_b ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_translate(float x , float y , float z , char *a) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	geometry.translate( x , y , z , shape_a  ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_scale(float x , float y , float z , char *a) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	geometry.scale( x , y , z , shape_a  ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 

}

char* ffi_rotateX(float x , char *a) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	geometry.rotateX( x , shape_a  ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_rotateY(float y , char *a) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	geometry.rotateY( y , shape_a  ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_rotateZ(float z , char *a) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	geometry.rotateZ( z , shape_a  ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 

}

char* ffi_extrude(float h1, char *a) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	geometry.extrude( h1 , shape_a  ); 
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

char* ffi_minkowski(char *a, char*b) { 
	ReadWrite readwrite;
	Geometry geometry; 		
	TopoDS_Shape shape_a = readwrite.ReadBREP(a);
	TopoDS_Shape shape_b = readwrite.ReadBREP(b);
	geometry.minkowski(shape_a,shape_b); 	
	char *new_buf = strdup((char*)readwrite.WriteBREP(shape_a).c_str());	
	return new_buf; 
}

#ifdef DEBUG
	int main() { 
		std::cout <<   
			ffi_scale( 1.0 , 1.0 , 2.0 ,ffi_difference(
		   ffi_minkowski(
		   	ffi_cube(-25.0,-25.0,-25.0,50.0,50.0,50.0), 
		    ffi_sphere(25.0, 0.0 , 0.0 , 0.0)
		  ),
			ffi_translate( 0.0 , 0.0 , 50.0 , ffi_sphere(15.0,0.0,0.0,0.0))
		));

		/*ffi_uni(
			ffi_minkowski(
		  	ffi_box(-25.0,-25.0,-25.0,50.0,50.0,50.0), 
		    ffi_sphere(25.0, 0.0 , 0.0 , 0.0)
		  ),
			ffi_translate( 0.0 , 0.0 , -100.0 , ffi_cylinder(25.0,200.0,0.0))
		);
	 ffi_scale( 1.0 , 1.0 , 2.0 , ffi_sphere(25.0, 0.0 , 0.0 , 0.0) );*/
	 
	 return 0;  
	}
#endif
