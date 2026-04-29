#include "File_parsing.cpp"
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <STEPControl_Reader.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <filesystem>
#include <gp_Cylinder.hxx>
#include <iostream>

using namespace std;
using namespace TopoDS;
using namespace filesystem;

int main()
{

	//std::string path = R"(C:\Users\ljste\Downloads\.25-20 Screw 1 Lg.STEP)";

	std::string path = R"(C:\Users\ljste\source\repos\STEP_file_feature_recognition\STEP_file_feature_recognition\.25-20 Screw 1 Lg.STEP)";
	std::cout << "Checking: " << path << std::endl;

	if (!filesystem::exists(path))
	{
		std::cout << "Path does not exist!" << std::endl;
		return -1;
	}

	TopoDS_Shape shape;
	shape = STEPFileParsing::readStep(path);
	STEPFileParsing::findCylinders(shape);

}
