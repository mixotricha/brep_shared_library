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
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Polyhedron_3.h> 
#include <CGAL/Nef_polyhedron_3.h> 

#include <CGAL/IO/Nef_polyhedron_iostream_3.h> 
#include <CGAL/IO/Polyhedron_iostream.h> 

#include <CGAL/OFF_to_nef_3.h>
#include <CGAL/IO/OFF_reader.h>

#include <CGAL/IO/print_wavefront.h>

#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>

#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

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

#include <boost/foreach.hpp>

using namespace std;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron; 
typedef Polyhedron::HalfedgeDS HalfedgeDS; 
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron; 

typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;

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

			int a = iNode1; 
			int b = (myInvert ? iNode3 : iNode2); 
			int c = (myInvert ? iNode2 : iNode3);

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

			std::cout << a << " " << thePnt1.X() << " " << thePnt1.Y() << " " << thePnt1.Z() << std::endl; 
			std::cout << b << " " << thePnt2.X() << " " << thePnt2.Y() << " " << thePnt2.Z() << std::endl; 
			std::cout << c << " " << thePnt3.X() << " " << thePnt3.Y() << " " << thePnt3.Z() << std::endl; 
	
    }

  private:
    Handle(Poly_Triangulation) myPoly;
    gp_Trsf myTrsf;
    int myNbTriangles;
    bool myInvert;
  };


}

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
	output << "solid shape, STL ascii file, created with Makertron Technology\n"; 
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

// --------------------------------------------------------------
// Convert a BREP in to a CGAL surface. Sure a more optimal way 
// exists to do this but for now will get the job done. 
// --------------------------------------------------------------
/*template <typename Polyhedron> bool BrepCgal::AlternateBrepToCgal(TopoDS_Shape& aShape, Polyhedron& p) { 
	std::vector<Kernel::Point_3> points; 
  std::vector<std::vector<std::size_t>> polygons;
	// complete winding of points in triangle order. Every 9 values is a face. 
	for (TopExp_Explorer exp (aShape, TopAbs_FACE); exp.More(); exp.Next())
	{
		TriangleAccessor aTool (TopoDS::Face (exp.Current()));
		for (int iTri = 1; iTri <= aTool.NbTriangles(); iTri++)
		{
		 gp_Vec aNorm;
	   gp_Pnt aPnt1, aPnt2, aPnt3;
		 aTool.GetTriangle (iTri, aNorm, aPnt1, aPnt2, aPnt3);
			points.push_back(	Kernel::Point_3( aPnt1.X() , aPnt1.Y() , aPnt1.Z() ) ); 
			points.push_back(	Kernel::Point_3( aPnt2.X() , aPnt2.Y() , aPnt2.Z() ) ); 
			points.push_back(	Kernel::Point_3( aPnt3.X() , aPnt3.Y() , aPnt3.Z() ) ); 		
		}
	}
	for ( std::size_t i = 0; i < points.size(); i+=3 ) { 
		std::vector<std::size_t> facet(3);
		facet[0] = i+0; facet[1] = i+1; facet[2] = i+2; 
		polygons.push_back( facet ); 	
	}		
	CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons );
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, p);
	if (CGAL::is_closed(p) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(p))) { 
  	CGAL::Polygon_mesh_processing::reverse_face_orientations(p);
	}
	return true; 
}*/

// --------------------------------------------------------------
// Convert a BREP in to a CGAL surface. Sure a more optimal way 
// exists to do this but for now will get the job done. 
// --------------------------------------------------------------
template <typename Polyhedron> bool BrepCgal::BrepToCgal(TopoDS_Shape& aShape, Polyhedron& p) { 
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
		 aTool.GetTriangle (iTri, aNorm, aPnt1, aPnt2, aPnt3);		
		 points.push_back( aPnt1.X() ); points.push_back( aPnt1.Y() ); points.push_back( aPnt1.Z() );
		 points.push_back( aPnt2.X() ); points.push_back( aPnt2.Y() ); points.push_back( aPnt2.Z() );
		 points.push_back( aPnt3.X() ); points.push_back( aPnt3.Y() ); points.push_back( aPnt3.Z() ); 			
		}
	}
	// Now convert to reduced index form for CGAL. Convexity problems if we repeat vectors.  
	for ( int i = 0; i < points.size(); i+=9 ) { // complete face is always [x,y,z,x,y,z,x,y,z]  
		// Grab our face set fA,fB,fC 
		double xa = points[i+0]; double ya = points[i+1]; double za = points[i+2]; 
		double xb = points[i+3]; double yb = points[i+4]; double zb = points[i+5];
		double xc = points[i+6]; double yc = points[i+7]; double zc = points[i+8];
		int fA = -1; int fB = -1; int fC = -1; 
		for ( int j = 0; j < nPoints.size(); j+=3 ) { // Have we used the same [x,y,z] before 	 
			double xj = nPoints[j+0]; double yj = nPoints[j+1]; double zj = nPoints[j+2]; 
			if ( xa == xj && ya == yj && za == zj ) fA = 0; 
			if ( xb == xj && yb == yj && zb == zj ) fB = 0; 
			if ( xc == xj && yc == yj && zc == zj ) fC = 0; 			
		}
		// If we did not find it in nPoints then add it to nPoints 
		if ( fA == -1 ) { nPoints.push_back(xa); nPoints.push_back(ya); nPoints.push_back(za); } 
		if ( fB == -1 ) { nPoints.push_back(xb); nPoints.push_back(yb); nPoints.push_back(zb); } 
		if ( fC == -1 ) { nPoints.push_back(xc); nPoints.push_back(yc); nPoints.push_back(zc); } 
	}
	// Now convert to reduced index form for CGAL. Convexity problems if we repeat vectors.  
	for ( int i = 0; i < points.size(); i+=9 ) { // complete face is always [x,y,z,x,y,z,x,y,z]  
		// Grab our face set nfA,nfB,nfC 
		double xa = points[i+0]; double ya = points[i+1]; double za = points[i+2]; 
		double xb = points[i+3]; double yb = points[i+4]; double zb = points[i+5];
		double xc = points[i+6]; double yc = points[i+7]; double zc = points[i+8];
		int nfA = -1; int nfB = -1; int nfC = -1; 
		for ( int j = 0; j < nPoints.size(); j+=3 ) { // Have we used the same [x,y,z] before 	 
			double xj = nPoints[j+0]; double yj = nPoints[j+1]; double zj = nPoints[j+2]; 
			if ( xa == xj && ya == yj && za == zj ) nfA = j/3; 
			if ( xb == xj && yb == yj && zb == zj ) nfB = j/3; 
			if ( xc == xj && yc == yj && zc == zj ) nfC = j/3; 			
		}	
		if ( nfA != -1 && nfB != -1 && nfC != -1 ) {  // Push out the reduced index 
			if ( nfA == nfB || nfA == nfC || nfB == nfA || nfB == nfC ) { 
				std::cout << "Self intersecting face because floating point tells lies! " << nfA << " " << nfB << " " << nfC << std::endl; 
			} else { 
				nFaces.push_back(nfA); nFaces.push_back(nfB); nFaces.push_back(nfC); 
			}
		}
		else { 
			std:cout << "Failed to find reindexed vector in BrepCGAL conversion" << std::endl; 
		}
	}

	//for ( int i = 0; i < nPoints.size(); i+=3 ) {
	//	output << "v " << nPoints[i+0] << " " << nPoints[i+1] << " " << nPoints[i+2] << "\n"; 
	//}  
	//for ( int j = 0; j < nFaces.size(); j+=3 ) { 
	//	output << "f " << nFaces[j+0]+1 << " " << nFaces[j+1]+1 << " " << nFaces[j+2]+1 << "\n"; 
	//}*/

  surface_builder<HalfedgeDS> builder( nPoints, nFaces );
  p.delegate( builder ); 
	return true; 
}

// -------------------------------------------------------------------------------
// Check if a Polyhedron is weakly convex. As per the OpenSCAD code. 
// Did not implement the shell checking part yet. Do I need to?
// ------------------------------------------------------------------------------- 
bool is_weakly_convex(Polyhedron const& p) {
	for (typename Polyhedron::Edge_const_iterator i = p.edges_begin(); i != p.edges_end(); ++i) {
  	typename Polyhedron::Plane_3 p(i->opposite()->vertex()->point(), i->vertex()->point(), i->next()->vertex()->point());
    if (p.has_on_positive_side(i->opposite()->next()->vertex()->point()) &&
       CGAL::squared_distance(p, i->opposite()->next()->vertex()->point()) > 1e-8) {
				std::cout << "Was false" << std::endl; 
     		return false;
 		 }
	}
  std::cout << "Was true" << std::endl; 
  return true; 
}


// -------------------------------------------------------------------------
// Following the existing openscad code as guide. Two Brep shapes converted
// to CGAL Polyhedrons. 
// -------------------------------------------------------------------------
 
bool BrepCgal::minkowski( TopoDS_Shape& aShape , TopoDS_Shape& bShape ) {
	
	Polyhedron aMesh;
	BrepToCgal(aShape,aMesh); 
	//std::ofstream out("a.off"); out << aMesh; out.close(); //createPolySetFromPolyhedron( p ); 
	//Nef_polyhedron A(aMesh);
	//if (A.is_convex()) { 
	//	std::cout << "So it passed" << std::endl; 
	//}

	//Polyhedron bMesh;
	//BrepToCgal(bShape,bMesh); 
	//std::ofstream out("b.off"); out << bMesh; out.close(); //createPolySetFromPolyhedron( p ); 

	// Incrementally fill the holes
  /*unsigned int nb_holes = 0;
  BOOST_FOREACH(Halfedge_handle h, halfedges(bMesh))
  {
    if(h->is_border())
    {
      std::vector<Facet_handle>  patch_facets;
      std::vector<Vertex_handle> patch_vertices;
      bool success = CGAL::cpp11::get<0>(
        CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                  bMesh,
                  h,
                  std::back_inserter(patch_facets),
                  std::back_inserter(patch_vertices),
     CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, bMesh)).
                  geom_traits(Kernel())) );
      std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
      std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
      std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
      ++nb_holes;
    }
  }*/

	//Nef_polyhedron B(bMesh);
	//if (B.is_convex()) { 
	//	std::cout << "So it passed" << std::endl; 
	//}

	/*std::string filename("cube.off"); 
  std::ifstream input(filename.c_str());
  if (!input)
  {
    std::cerr << "Cannot open file " << std::endl;
    return 1;
  }
  std::vector<Kernel::Point_3> points;
  std::vector< std::vector<std::size_t> > polygons;
  if (!CGAL::read_OFF(input, points, polygons))
  {
    std::cerr << "Error parsing the OFF file " << std::endl;
    return 1;
  }

	for ( std::size_t i = 0; i < polygons.size(); i++ ) { 
		std::cout << polygons[i][0]+1 << " " << polygons[i][1]+1 << " " << polygons[i][2]+1 << std::endl; 
	}

  CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
  Polyhedron mesh;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, mesh);
  if (CGAL::is_closed(mesh) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(mesh)))
    CGAL::Polygon_mesh_processing::reverse_face_orientations(mesh);*/

	
   return 0;
	

}


