// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <CGAL/Exact_predicates_exact_constructions_kernel.h> 
#include <CGAL/Polyhedron_3.h> 
#include <CGAL/IO/Polyhedron_iostream.h> 
#include <CGAL/Nef_polyhedron_3.h> 
#include <CGAL/IO/Nef_polyhedron_iostream_3.h> 
#include <CGAL/OFF_to_nef_3.h>
#include <CGAL/IO/print_wavefront.h>

// Brep Includes
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <OSD_Path.hxx>
#include <OSD_OpenFile.hxx>
#include <RWStl.hxx>
#include <StlAPI_Writer.hxx>
#include <StlMesh_Mesh.hxx>
#include <StlTransfer.hxx>
#include <BrepCgal.h>
#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp_Explorer.hxx>
#include <Poly_Triangulation.hxx>

using namespace std;
 
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron; 
typedef Polyhedron::HalfedgeDS HalfedgeDS; 
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron; 

// Auxiliary tools
namespace
{
  // Tool to get triangles from triangulation taking into account face
  // orientation and location
  class TriangleAccessor
  {
  public:
    TriangleAccessor (const TopoDS_Face& aFace)
    {
      TopLoc_Location aLoc;
      myPoly = BRep_Tool::Triangulation (aFace, aLoc);
      myTrsf = aLoc.Transformation();
      myNbTriangles = (myPoly.IsNull() ? 0 : myPoly->Triangles().Length());
      myInvert = (aFace.Orientation() == TopAbs_REVERSED);
      if (myTrsf.IsNegative())
        myInvert = ! myInvert;
    }

    int NbTriangles () const { return myNbTriangles; } 

    // get i-th triangle and outward normal
    void GetTriangle (int iTri, gp_Vec &theNormal, gp_Pnt &thePnt1, gp_Pnt &thePnt2, gp_Pnt &thePnt3, int &iNode1, int &iNode2, int &iNode3)
    {
      // get positions of nodes
      //int iNode1, iNode2, iNode3;
      myPoly->Triangles()(iTri).Get (iNode1, iNode2, iNode3); 
      thePnt1 = myPoly->Nodes()(iNode1);
      thePnt2 = myPoly->Nodes()(myInvert ? iNode3 : iNode2);
      thePnt3 = myPoly->Nodes()(myInvert ? iNode2 : iNode3);

      // apply transormation if not identity
      if (myTrsf.Form() != gp_Identity)
      {
        thePnt1.Transform (myTrsf);
        thePnt2.Transform (myTrsf);
        thePnt3.Transform (myTrsf);
      }

      // calculate normal
      theNormal = (thePnt2.XYZ() - thePnt1.XYZ()) ^ (thePnt3.XYZ() - thePnt1.XYZ());
      Standard_Real aNorm = theNormal.Magnitude();
      if (aNorm > gp::Resolution())
        theNormal /= aNorm;
    }

  private:
    Handle(Poly_Triangulation) myPoly;
    gp_Trsf myTrsf;
    int myNbTriangles;
    bool myInvert;
  };

  // convert to float and, on big-endian platform, to little-endian representation
  inline double convertFloat (Standard_Real aValue)
  {
#ifdef OCCT_BINARY_FILE_DO_INVERSE
    return OSD_BinaryFile::InverseShortReal ((double)aValue);
#else
    return (double)aValue;
#endif
  }
}

// -------------------------------------------------------------------------------------------------------------
// Build a CGAL surface. Based on the http://jamesgregson.blogspot.com.au/2012/05/example-code-for-building.html 
// -------------------------------------------------------------------------------------------------------------
template<class HDS>
class surface_builder : public CGAL::Modifier_base<HDS> {
public:
 std::vector<double> &coords;
 std::vector<int>    &tris;
    surface_builder( std::vector<double> &_coords, std::vector<int> &_tris ) : coords(_coords), tris(_tris) {}
    void operator()( HDS& hds) {
  		typedef typename HDS::Vertex   Vertex;
      typedef typename Vertex::Point Point;
      CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
      B.begin_surface( coords.size()/3, tris.size()/3 );
  		for( int i=0; i<(int)coords.size(); i+=3 ){
  			B.add_vertex( Point( coords[i+0], coords[i+1], coords[i+2] ) );
  		}
  		for( int i=0; i<(int)tris.size(); i+=3 ){
   			B.begin_facet();
   			B.add_vertex_to_facet( tris[i+0] );
   			B.add_vertex_to_facet( tris[i+1] );
   			B.add_vertex_to_facet( tris[i+2] );
   			B.end_facet();
  		} 
    	B.end_surface();
  	}
};

BrepCgal::BrepCgal() {
}

// ---------------------------------------------------------------------------------
// export the result back out as vector list
// ---------------------------------------------------------------------------------
template <typename Polyhedron> bool createPolySetFromPolyhedron(const Polyhedron &p)
{
	int i = 0; 
	double xa,ya,za,xb,yb,zb,xc,yc,zc; 
	std::stringstream output;
	bool err = false;
	typedef typename Polyhedron::Vertex                                 Vertex;
	typedef typename Polyhedron::Vertex_const_iterator                  VCI;
	typedef typename Polyhedron::Facet_const_iterator                   FCI;
	typedef typename Polyhedron::Halfedge_around_facet_const_circulator HFCC;		
	output << "solid shape, STL ascii file, created with Makertron Technology\n" << "\n"; 
	for (FCI fi = p.facets_begin(); fi != p.facets_end(); ++fi) {
		HFCC hc = fi->facet_begin();
		HFCC hc_end = hc;
		do {
			Vertex const& v = *((hc++)->vertex());
			if ( i == 0 )  {
				xa = CGAL::to_double(v.point().x());
				ya = CGAL::to_double(v.point().y());
				za = CGAL::to_double(v.point().z());
			}
			if ( i == 1 )  {
				xb = CGAL::to_double(v.point().x());
				yb = CGAL::to_double(v.point().y());
				zb = CGAL::to_double(v.point().z());
			}
			if ( i == 2 ) { 
				xc = CGAL::to_double(v.point().x());
				yc = CGAL::to_double(v.point().y());
				zc = CGAL::to_double(v.point().z());	
				output << " facet normal 0.0 0.0 0.0\n";
        output <<  "   outer loop\n";
        output <<  "     vertex " << xa << " " << ya << " " << za << "\n";
        output <<  "     vertex " << xb << " " << yb << " " << zb << "\n";
        output <<  "     vertex " << xc << " " << yc << " " << zc << "\n";
        output <<  "   endloop\n";
        output <<  " endfacet\n";
			}
			i++; 
			if ( i == 3 ) i = 0;  
			 
		} while (hc != hc_end);
	}
	output << "endsolid shape\n";
	std::cout << output.str() << std::endl; 
	return err;
}

std::string BrepCgal::test(TopoDS_Shape& aShape) { 

	std::stringstream output;
	std::vector<int> faces; 
	std::vector<double> points;   

	Standard_Integer theNbTri;
  Standard_Integer theNbEdges;
  Standard_Integer theNbNodes;

  Handle(Poly_Triangulation) T;
  TopLoc_Location L;

  /*for ( TopExp_Explorer ex(aShape, TopAbs_FACE); ex.More(); ex.Next()) {
    TopoDS_Face F = TopoDS::Face(ex.Current());
    T = BRep_Tool::Triangulation(F, L); 
		if (!T.IsNull()) {
			for (int iTri = 1; iTri <= T->NbTriangles(); iTri++) { 
				Standard_Integer iNode1;
  			Standard_Integer iNode2;
  			Standard_Integer iNode3;
				T->Triangles()(iTri).Get (iNode1, iNode2, iNode3);
				std::cout << iNode1 << "  " << iNode2 << "  " << iNode3 << std::endl; 
			}
		}
	}*/
	
	for (TopExp_Explorer exp (aShape, TopAbs_FACE); exp.More(); exp.Next())
	{
		TriangleAccessor aTool (TopoDS::Face (exp.Current()));
		for (int iTri = 1; iTri <= aTool.NbTriangles(); iTri++)
		{
		 gp_Vec aNorm;
	   gp_Pnt aPnt1, aPnt2, aPnt3;
		 int iNode1, iNode2, iNode3;
		 aTool.GetTriangle (iTri, aNorm, aPnt1, aPnt2, aPnt3 , iNode1 , iNode2 , iNode3);
		 std::cout << iNode1 << " "  << iNode2 << " " << iNode3 << std::endl; 
		 //points.push_back( aPnt1.X() ); points.push_back( aPnt1.Y() ); points.push_back( aPnt1.Z() );
		 //points.push_back( aPnt2.X() ); points.push_back( aPnt2.Y() ); points.push_back( aPnt2.Z() );
		 //points.push_back( aPnt3.X() ); points.push_back( aPnt3.Y() ); points.push_back( aPnt3.Z() ); 			
		}
	}

	


	//for ( int i = 0; i < points.size()/3; i+=3 ) {
	//	faces.push_back( i+0 ); 
	//	faces.push_back( i+1 ); 
	//	faces.push_back( i+2 );  
	//}  

	//Polyhedron PA;
  //surface_builder<HalfedgeDS> builder( points, faces );
  //PA.delegate( builder );
	//createPolySetFromPolyhedron( PA );
  //Nef_polyhedron A(PA);

	output << "0]\n";	
	return output.str(); 
}


