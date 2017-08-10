# brep_shared_library

Code for building the brep shared library. Bare bones at the moment. 

Currently working on classical floating point problems. Having some success. 
First pass at passing objects between Brep and CGAL. Seems to be working quite well. 
Adding support now for message callbacks. 

Classic issues with floating point resolution. 

http://www.cs.unc.edu/~snoeyink/c/c205/Triangle.pdf

Feature        | Available
---------------|------------
sphere         | y
cube           | y
cylinder       | y
polyhedron     | y
polygon        | y
circle         | y
union          | y
intersection   | y 
difference     | y 
linear_extrude | y 
minkowski      | y
rotate         | y
translate      | y 
scale          | n 
color/colour   | n 

