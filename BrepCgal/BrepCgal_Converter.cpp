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
    void GetTriangle (int iTri, gp_Vec &theNormal, gp_Pnt &thePnt1, gp_Pnt &thePnt2, gp_Pnt &thePnt3)
    {
      // get positions of nodes
      int iNode1, iNode2, iNode3;
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

// --------------------------------------------------------------
// Convert a BREP in to a CGAL surface. Sure a more optimal way 
// exists to do this but for now will get the job done. 
// --------------------------------------------------------------
std::string BrepCgal::test(TopoDS_Shape& aShape) { 

	std::stringstream output;
	std::vector<double> points;   
	std::vector<double> nPoints;   
	std::vector<int> nFaces; 
	
	// complete winding of points in triangle order. Every 9 values is a face. 
	for (TopExp_Explorer exp (aShape, TopAbs_FACE); exp.More(); exp.Next())
	{
		TriangleAccessor aTool (TopoDS::Face (exp.Current()));
		for (int iTri = 1; iTri <= aTool.NbTriangles(); iTri++)
		{
		 gp_Vec aNorm;
	   gp_Pnt aPnt1, aPnt2, aPnt3;
		 int iNode1, iNode2, iNode3;
		 aTool.GetTriangle (iTri, aNorm, aPnt1, aPnt2, aPnt3);
		 points.push_back( aPnt1.X() ); points.push_back( aPnt1.Y() ); points.push_back( aPnt1.Z() );
		 points.push_back( aPnt2.X() ); points.push_back( aPnt2.Y() ); points.push_back( aPnt2.Z() );
		 points.push_back( aPnt3.X() ); points.push_back( aPnt3.Y() ); points.push_back( aPnt3.Z() ); 			
		}
	}

	// Now convert to reduced index form for CGAL. Convexity problems if we repeat vectors.  
	for ( int i = 0; i < points.size(); i+=9 ) { // complete face is always [x,y,z,x,y,z,x,y,z]  
		// Grab our face set fA,fB,fC 
		xa = points[i+0]; ya = points[i+1]; za = points[i+2]; 
		xb = points[i+3]; yb = points[i+4]; zb = points[i+5];
		xc = points[i+6]; yc = points[i+7]; zc = points[i+8];
		int fA = i+0; int fB = i+3; int fC = i+6; 
		int nFA = fA; int nFB = fB; int nFC = fC;   
		for ( int j = 0; j < nPoints.size(); j+=3 ) { // Have we used the same [x,y,z] before 	 
			xj = nPoints[j+0]; yj = nPoints[j+1]; zj = nPoints[j+2]; 
			if ( xa == xj && ya == yj && za == zj ) nFA = j+0; 
			if ( xb == xj && yb == yj && zb == zj ) nFB = j+0; 
			if ( xc == xj && yc == yj && zc == zj ) nFC = j+0; 			
		}
		// If we did not find it in nPoints then add it to nPoints 
		if ( fA == nFA ) { nPoints.push_back(xa); nPoints.push_back(ya); nPoints.push_back(za) } 
		if ( fB == nFB ) { nPoints.push_back(xb); nPoints.push_back(yb); nPoints.push_back(zb) } 
		if ( fC == nFC ) { nPoints.push_back(xc); nPoints.push_back(yc); nPoints.push_back(zc) } 
 		// Push out the reduced index 
		nFaces.push_back(nFA); nFaces.push_back(nFB); nFaces.push_back(nFC); 
	}

	output << nPoints.size() << " " << nFaces.size() << "\n"
	for ( int i = 0; i < nPoints.size(); i+=3 ) {
		output << "v " << nPoints[i+0] << " " nPoints[i+1] << " " << nPoints[i+2] << "\n"; 
	}  
	for ( int j = 0; j < nFaces.size(); i+=3 ) { 
		output << "f 3 " << nFaces[j+0] << " " << nFaces[j+1] << " " << nFaces[j+2] << "\n"; 
	}
	
	//Polyhedron PA;
  //surface_builder<HalfedgeDS> builder( points, faces );
  //PA.delegate( builder );
	//createPolySetFromPolyhedron( PA );
  //Nef_polyhedron A(PA);

	return output.str(); 
}


