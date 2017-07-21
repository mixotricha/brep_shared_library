
CXX = g++
DEFINES = 
CXXFLAGS = -O3 -std=gnu++11 -frounding-math -m64 -shared -fPIC
TARGET = brep.so

OC_INCL_DIR = xxxx/pencascade-7.1.0/inc
OC_LIB_DIR = xxxx/pencascade-7.1.0/lin64/gcc/lib
OC_LIB = -lTKOpenGl -lTKernel -lTKGeomBase -lTKTopAlgo -lTKOffset -lTKBool -lTKPrim -lTKFillet -lTKMath -lTKService -lTKV3d -lTKIGES -lTKSTL -lTKVRML -lTKSTEP -lTKG3d -lTKG2d -lTKXSBase -lTKShHealing -lTKBO -lTKBRep -lTKMesh 

BOOST_LIBS = -lboost_iostreams -lboost_system -lboost_chrono -lboost_date_time -lboost_atomic -lboost_thread

INCLUDES       = -I. \
		-I/usr/local/include/eigen3 \
	  -I/usr/lib/x86_64-linux-gnu/glib-2.0/include \
		-I$(OC_INCL_DIR) 
	
	
LIBS       = -L/usr/local/lib -ldl -lm -lz -lHalf -lmpfr -lgmp -lpthread \
		-Wl,-rpath,$(OC_LIB_DIR) -L$(OC_LIB_DIR) $(OC_LIB) \
		  $(BOOST_LIBS)
		
first: all

all:	
	$(CXX) brep.cpp $(CXXFLAGS) -o $(TARGET) $(DEFINES) $(INCLUDES) $(VTK_MENTAL) $(LIBS) 


