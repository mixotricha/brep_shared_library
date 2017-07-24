#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Nef_3/SNC_indexed_items.h>
#include <CGAL/convex_decomposition_3.h> 
#include <list>

#include <CGAL/IO/Polyhedron_iostream.h> 
#include <CGAL/Nef_polyhedron_3.h> 
#include <CGAL/IO/Nef_polyhedron_iostream_3.h> 
#include <CGAL/OFF_to_nef_3.h>
#include <CGAL/IO/print_wavefront.h>

// Streams 
#include <fstream>
#include <iostream>
#include <string>
#include<algorithm>

// Vectors
#include <vector>

// Math
#include <cmath>
#include <assert.h>


// boost 
#include <boost/unordered_set.hpp>
#include <CGAL/convex_hull_3.h>

// Namespaces

using namespace std;
 
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron; 
typedef Polyhedron::HalfedgeDS HalfedgeDS; 
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron; 

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


int get_first_integer( const char *v ){
 int ival;
 std::string s( v );
 std::replace( s.begin(), s.end(), '/', ' ' );
 sscanf( s.c_str(), "%d", &ival );
 return ival;
}
 
void load_obj( const char *filename, std::vector<double> &coords, std::vector<int> &tris ){
 double x, y, z;
 char line[1024], v0[1024], v1[1024], v2[1024];
  
 // open the file, return if open fails
 FILE *fp = fopen(filename, "r" );
 if( !fp ) return;
  
 // read lines from the file, if the first character of the
 // line is 'v', we are reading a vertex, otherwise, if the
 // first character is a 'f' we are reading a facet
 while( fgets( line, 1024, fp ) ){
  if( line[0] == 'v' ){
   sscanf( line, "%*s%lf%lf%lf", &x, &y, &z );
   coords.push_back( x );
   coords.push_back( y );
   coords.push_back( z );
  } else if( line[0] == 'f' ){
   sscanf( line, "%*s%s%s%s", v0, v1, v2 );
   tris.push_back( get_first_integer( v0 )-1 );
   tris.push_back( get_first_integer( v1 )-1 );
   tris.push_back( get_first_integer( v2 )-1 );
  }
 }
 fclose(fp); 
}


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

void cgal_horrible() { 

 int i = 0; 
 std::vector<double> cube_points;
 std::vector<int>    cube_faces;
 load_obj( "cube.obj", cube_points, cube_faces );						
 std::cout << cube_points.size() << std::endl;
 Polyhedron PA;
 surface_builder<HalfedgeDS> builder( cube_points, cube_faces );
 PA.delegate( builder );
 Nef_polyhedron A(PA); 

 std::vector<double> sphere_points;
 std::vector<int>    sphere_faces;
 load_obj( "sphere.obj", sphere_points, sphere_faces );						
 std::cout << sphere_points.size() << std::endl;
 Polyhedron PB;
 surface_builder<HalfedgeDS> bbuilder( sphere_points, sphere_faces );
 PB.delegate( bbuilder );
 Nef_polyhedron B(PB); 

 typedef CGAL::Epick Hull_kernel;

 std::list<Polyhedron> P[2];
 std::list<CGAL::Polyhedron_3<Hull_kernel> > result_parts;	

 if (A.is_convex()) { 
	std::cout << "Minkowski: child A is convex" << std::endl; 
	P[0].push_back(PA);
 }
 else { 
	std::cout << "Minkowski: child A was not convex doing decomposition" << std::endl; 
	Nef_polyhedron decomposed_nef;
	CGAL::convex_decomposition_3(A);
	Nef_polyhedron::Volume_const_iterator ci = ++decomposed_nef.volumes_begin();
	for(; ci != decomposed_nef.volumes_end(); ++ci) {
		if(ci->mark()) {
			Polyhedron poly;
			decomposed_nef.convert_inner_shell_to_polyhedron(ci->shells_begin(), poly);
			P[0].push_back(poly);
		}
	}
 } 
 std::cout << "Minkowski: decomposed into " << P[0].size() << std::endl;

 //if (B.is_convex() ) { 
	std::cout << "Minkowski: child B is convex" << std::endl; 
	P[1].push_back(PB);
 /*}
 else { 
	std::cout << "Minkowski: child B was not convex doing decomposition" << std::endl; 
	Nef_polyhedron decomposed_nef;
	CGAL::convex_decomposition_3(B);
	Nef_polyhedron::Volume_const_iterator ci = ++decomposed_nef.volumes_begin();
	for(; ci != decomposed_nef.volumes_end(); ++ci) {
		if(ci->mark()) {
			Polyhedron poly;
			decomposed_nef.convert_inner_shell_to_polyhedron(ci->shells_begin(), poly);
			P[1].push_back(poly);
		}
	}
 } */

 std::cout << "Minkowski: decomposed into " << P[1].size() << std::endl;

 std::vector<Hull_kernel::Point_3> points[2];
 std::vector<Hull_kernel::Point_3> minkowski_points;

 for (size_t i = 0; i < P[0].size(); i++) {
 	for (size_t j = 0; j < P[1].size(); j++) {
		points[0].clear();
		points[1].clear();
		for (int k = 0; k < 2; k++) {
			std::list<Polyhedron>::iterator it = P[k].begin();
			std::advance(it, k==0?i:j);
			Polyhedron const& poly = *it;
			points[k].reserve(poly.size_of_vertices());
			for (Polyhedron::Vertex_const_iterator pi = poly.vertices_begin(); pi != poly.vertices_end(); ++pi) {
				Polyhedron::Point_3 const& p = pi->point();
				points[k].push_back(Hull_kernel::Point_3(to_double(p[0]),to_double(p[1]),to_double(p[2])));
			}
		}
		minkowski_points.clear();
		minkowski_points.reserve(points[0].size() * points[1].size());
		for (int i = 0; i < points[0].size(); i++) {
			for (int j = 0; j < points[1].size(); j++) {
				minkowski_points.push_back(points[0][i]+(points[1][j]-CGAL::ORIGIN));
			}
		}
		if (minkowski_points.size() <= 3) { continue;}
		CGAL::Polyhedron_3<Hull_kernel> result;
		CGAL::convex_hull_3(minkowski_points.begin(), minkowski_points.end(), result);
		std::vector<Hull_kernel::Point_3> strict_points;
		strict_points.reserve(minkowski_points.size());
		for (CGAL::Polyhedron_3<Hull_kernel>::Vertex_iterator i = result.vertices_begin(); i != result.vertices_end(); ++i) {
			Hull_kernel::Point_3 const& p = i->point();
			CGAL::Polyhedron_3<Hull_kernel>::Vertex::Halfedge_handle h,e;
			h = i->halfedge();
			e = h;
			bool collinear = false;
			bool coplanar = true;
			do {
				Hull_kernel::Point_3 const& q = h->opposite()->vertex()->point();
				if (coplanar && !CGAL::coplanar(p,q,
					h->next_on_vertex()->opposite()->vertex()->point(),
					h->next_on_vertex()->next_on_vertex()->opposite()->vertex()->point())) {
					coplanar = false;
				}
				for (CGAL::Polyhedron_3<Hull_kernel>::Vertex::Halfedge_handle j = h->next_on_vertex();
					j != h && !collinear && ! coplanar;
					j = j->next_on_vertex()) {
						Hull_kernel::Point_3 const& r = j->opposite()->vertex()->point();
						if (CGAL::collinear(p,q,r)) { collinear = true; }
					}
					h = h->next_on_vertex();
				} 
				while (h != e && !collinear);
					if (!collinear && !coplanar) strict_points.push_back(p);
				}
				result.clear();
				CGAL::convex_hull_3(strict_points.begin(), strict_points.end(), result);
				result_parts.push_back(result);
		}
 	}
 	//std::ofstream file;
 	//file.open ("test.obj");
 	//print_polyhedron_wavefront(file,result_parts);
 	//file.close(); 
 
}

int main() {
  cgal_horrible(); 
  return 0;
}
