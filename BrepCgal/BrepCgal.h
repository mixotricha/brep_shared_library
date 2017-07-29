
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

#include <Standard.hxx>
#include <Standard_DefineAlloc.hxx>
#include <Standard_Handle.hxx>

#include <Standard_Boolean.hxx>
#include <StlAPI_ErrorStatus.hxx>
#include <Standard_CString.hxx>
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

class StlMesh_Mesh;
class TopoDS_Shape;

class BrepCgal 
{
public:

	//DEFINE_STANDARD_ALLOC
	Standard_EXPORT BrepCgal();
	//Standard_EXPORT std::string Convert(const TopoDS_Shape& aShape);
	Standard_EXPORT template <typename Polyhedron> bool BrepToCgal(TopoDS_Shape& aShape, Polyhedron& p);
	Standard_EXPORT bool minkowski(TopoDS_Shape& aShape, TopoDS_Shape& bShape);
								
protected:


private:


};

