// Created on: 2000-06-23
// Created by: Sergey MOZOKHIN
// Copyright (c) 2000-2014 OPEN CASCADE SAS
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
	Standard_EXPORT std::string test(TopoDS_Shape& aShape);
									
protected:


private:


};

