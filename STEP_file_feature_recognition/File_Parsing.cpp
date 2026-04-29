#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Cylinder.hxx>
#include <iostream>
#include <STEPControl_Reader.hxx>
#include <Interface_Static.hxx>



using namespace std;
using namespace TopoDS;

class STEPFileParsing
{
public:
    
    
    static TopoDS_Shape readStep(string& path)
    {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());

        if (status != IFSelect_RetDone)
            throw runtime_error("Failed to read STEP file");

        reader.TransferRoots();
        return reader.OneShape();
    }

    static void findCylinders(TopoDS_Shape& shape)
    {
        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
        {
            TopoDS_Face face = Face(ex.Current());
            BRepAdaptor_Surface surf(face);

            if (surf.GetType() == GeomAbs_Cylinder)
            {
                gp_Cylinder cyl = surf.Cylinder();

                cout << "Cylinder found\n";
                cout << "Radius: " << ((double)cyl.Radius()/ 25.4)<< " inches" << "\n";
                cout << "Axis location: "
                    << cyl.Axis().Location().X() << ", "
                    << cyl.Axis().Location().Y() << ", "
                    << cyl.Axis().Location().Z() << "\n";
            }
        }
    }

 

private:



};