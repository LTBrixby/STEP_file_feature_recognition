#pragma once
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Cylinder.hxx>
#include <iostream>
#include <STEPControl_Reader.hxx>



class Thread_Detection
{
public:
    static TopoDS_Shape readStep( std::string& path);
    static void findCylinders( TopoDS_Shape& shape);
};



