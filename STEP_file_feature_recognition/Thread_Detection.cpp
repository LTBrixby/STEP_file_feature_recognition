#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>

#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Curve.hxx>

#include <gp_Cylinder.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>

#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>

#include <Standard_Real.hxx>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <optional>
#include <limits>

using namespace std;

class Thread_Detection
{
public:

    struct ThreadSize
    {
        string name;
        double majorDiameterInches;
        double majorRadiusInches;
        double pitchInches;
    };

    struct CylindricalFeature
    {
        gp_Cylinder cylinder;
        gp_Pnt axisLocation;
        gp_Dir axisDirection;

        double radiusMm = 0.0;
        double radiusInches = 0.0;

        bool looksHelical = false;
        double measuredPitchMm = 0.0;
        double measuredPitchInches = 0.0;

        optional<ThreadSize> matchedThread;
    };

    struct ThreadHit
    {
        std::string threadName;
        gp_Pnt centerPoint;
    };
    static constexpr double MM_PER_INCH = 25.4;

    static std::vector<ThreadSize> knownThreadSizes()
    {
        return {
            // name, major diameter inches, major radius inches, pitch inches

            // Number sizes - UNC
            { "#0-80 UNF",     0.0600, 0.0300, 1.0 / 80.0  },
            { "#1-64 UNC",     0.0730, 0.0365, 1.0 / 64.0  },
            { "#1-72 UNF",     0.0730, 0.0365, 1.0 / 72.0  },
            { "#2-56 UNC",     0.0860, 0.0430, 1.0 / 56.0  },
            { "#2-64 UNF",     0.0860, 0.0430, 1.0 / 64.0  },
            { "#3-48 UNC",     0.0990, 0.0495, 1.0 / 48.0  },
            { "#3-56 UNF",     0.0990, 0.0495, 1.0 / 56.0  },
            { "#4-40 UNC",     0.1120, 0.0560, 1.0 / 40.0  },
            { "#4-48 UNF",     0.1120, 0.0560, 1.0 / 48.0  },
            { "#5-40 UNC",     0.1250, 0.0625, 1.0 / 40.0  },
            { "#5-44 UNF",     0.1250, 0.0625, 1.0 / 44.0  },
            { "#6-32 UNC",     0.1380, 0.0690, 1.0 / 32.0  },
            { "#6-40 UNF",     0.1380, 0.0690, 1.0 / 40.0  },
            { "#8-32 UNC",     0.1640, 0.0820, 1.0 / 32.0  },
            { "#8-36 UNF",     0.1640, 0.0820, 1.0 / 36.0  },
            { "#10-24 UNC",    0.1900, 0.0950, 1.0 / 24.0  },
            { "#10-32 UNF",    0.1900, 0.0950, 1.0 / 32.0  },
            { "#12-24 UNC",    0.2160, 0.1080, 1.0 / 24.0  },
            { "#12-28 UNF",    0.2160, 0.1080, 1.0 / 28.0  },

            // Fractional sizes - UNC and UNF
            { "1/4-20 UNC",    0.2500, 0.1250, 1.0 / 20.0  },
            { "1/4-28 UNF",    0.2500, 0.1250, 1.0 / 28.0  },

            { "5/16-18 UNC",   0.3125, 0.15625, 1.0 / 18.0 },
            { "5/16-24 UNF",   0.3125, 0.15625, 1.0 / 24.0 },

            { "3/8-16 UNC",    0.3750, 0.1875, 1.0 / 16.0  },
            { "3/8-24 UNF",    0.3750, 0.1875, 1.0 / 24.0  },

            { "7/16-14 UNC",   0.4375, 0.21875, 1.0 / 14.0 },
            { "7/16-20 UNF",   0.4375, 0.21875, 1.0 / 20.0 },

            { "1/2-13 UNC",    0.5000, 0.2500, 1.0 / 13.0  },
            { "1/2-20 UNF",    0.5000, 0.2500, 1.0 / 20.0  },

            { "9/16-12 UNC",   0.5625, 0.28125, 1.0 / 12.0 },
            { "9/16-18 UNF",   0.5625, 0.28125, 1.0 / 18.0 },

            { "5/8-11 UNC",    0.6250, 0.3125, 1.0 / 11.0  },
            { "5/8-18 UNF",    0.6250, 0.3125, 1.0 / 18.0  },

            { "3/4-10 UNC",    0.7500, 0.3750, 1.0 / 10.0  },
            { "3/4-16 UNF",    0.7500, 0.3750, 1.0 / 16.0  },

            { "7/8-9 UNC",     0.8750, 0.4375, 1.0 / 9.0   },
            { "7/8-14 UNF",    0.8750, 0.4375, 1.0 / 14.0  },

            { "1-8 UNC",       1.0000, 0.5000, 1.0 / 8.0   },
            { "1-12 UNF",      1.0000, 0.5000, 1.0 / 12.0  },

            { "1 1/8-7 UNC",   1.1250, 0.5625, 1.0 / 7.0   },
            { "1 1/8-12 UNF",  1.1250, 0.5625, 1.0 / 12.0  },

            { "1 1/4-7 UNC",   1.2500, 0.6250, 1.0 / 7.0   },
            { "1 1/4-12 UNF",  1.2500, 0.6250, 1.0 / 12.0  },

            { "1 3/8-6 UNC",   1.3750, 0.6875, 1.0 / 6.0   },
            { "1 3/8-12 UNF",  1.3750, 0.6875, 1.0 / 12.0  },

            { "1 1/2-6 UNC",   1.5000, 0.7500, 1.0 / 6.0   },
            { "1 1/2-12 UNF",  1.5000, 0.7500, 1.0 / 12.0  }
        };
    }

    static TopoDS_Shape readStep(const string& path)
    {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());

        if (status != IFSelect_RetDone)
            throw runtime_error("Failed to read STEP file");

        reader.TransferRoots();
        return reader.OneShape();
    }

    static vector<CylindricalFeature> findThreadCandidates(const TopoDS_Shape& shape)
    {
        vector<CylindricalFeature> features;

        for (TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
            faceExplorer.More();
            faceExplorer.Next())
        {
            TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
            BRepAdaptor_Surface surface(face);

            if (surface.GetType() != GeomAbs_Cylinder)
                continue;

            gp_Cylinder cylinder = surface.Cylinder();

            CylindricalFeature feature;
            feature.cylinder = cylinder;
            feature.axisLocation = cylinder.Axis().Location();
            feature.axisDirection = cylinder.Axis().Direction();
            feature.radiusMm = cylinder.Radius();
            feature.radiusInches = cylinder.Radius() / MM_PER_INCH;

            auto pitch = estimateHelicalPitchFromFaceEdges(face, cylinder);

            if (pitch.has_value())
            {
                feature.looksHelical = true;
                feature.measuredPitchMm = pitch.value();
                feature.measuredPitchInches = pitch.value() / MM_PER_INCH;

                feature.matchedThread = matchKnownThread(
                    feature.radiusInches,
                    feature.measuredPitchInches
                );
            }

            features.push_back(feature);
        }

        return features;
    }

    static void printFeatures(const vector<CylindricalFeature>& features)
    {
        for (size_t i = 0; i < features.size(); ++i)
        {
            const auto& f = features[i];

            cout << "Feature " << i << "\n";
            cout << "  Cylinder radius: " << f.radiusInches << " in\n";
            cout << "  Axis location: "
                << f.axisLocation.X() << ", "
                << f.axisLocation.Y() << ", "
                << f.axisLocation.Z() << "\n";

            cout << "  Axis direction: "
                << f.axisDirection.X() << ", "
                << f.axisDirection.Y() << ", "
                << f.axisDirection.Z() << "\n";

            if (f.looksHelical)
            {
                cout << "  Looks helical: yes\n";
                cout << "  Measured pitch: " << f.measuredPitchInches << " in\n";

                if (f.matchedThread.has_value())
                {
                    cout << "  Thread match: "
                        << f.matchedThread->name << "\n";
                }
                else
                {
                    cout << "  Thread match: none\n";
                }
            }
            else
            {
                cout << "  Looks helical: no\n";
            }

            cout << "\n";
        }
    }

    static void printThreadSummary(
        const std::vector<CylindricalFeature>& features,
        const std::string& outputPath = "thread_report.txt")
    {
        std::vector<ThreadHit> hits;

        for (const auto& f : features)
        {
            if (!f.looksHelical)
                continue;

            if (!f.matchedThread.has_value())
                continue;

            ThreadHit hit;
            hit.threadName = normalizeThreadName(f.matchedThread->name);
            hit.centerPoint = estimateCylinderCenterPoint(f);

            if (!alreadyContainsHit(hits, hit))
                hits.push_back(hit);
        }

        std::ostringstream report;

        report << "Thread Sizes\n";

        for (const auto& hit : hits)
        {
            report << hit.threadName << ": "
                << "("
                << std::fixed << std::setprecision(4)
                << hit.centerPoint.X() << ", "
                << hit.centerPoint.Y() << ", "
                << hit.centerPoint.Z()
                << ")"
                << "\n";
        }

        std::cout << report.str();

        std::ofstream file(outputPath);

        if (!file)
            throw std::runtime_error("Failed to open output file: " + outputPath);

        file << report.str();
    }

    

private:

    static gp_Pnt estimateCylinderCenterPoint(const CylindricalFeature& f)
    {
        gp_Pnt base = f.axisLocation;
        gp_Dir dir = f.axisDirection;

        // Right now this just returns the axis origin from OpenCascade.
        // For a true center-through-depth point, you need min/max projection
        // of sampled points along the axis.
        return base;
    }

    static std::string normalizeThreadName(const std::string& fullName)
    {
        // Converts "1/4-20 UNC" to "1/4-20"
        // Converts "3/8-16 UNC" to "3/8-16"

        size_t firstSpace = fullName.find(' ');

        if (firstSpace == std::string::npos)
            return fullName;

        return fullName.substr(0, firstSpace);
    }


    static bool alreadyContainsHit(
        const std::vector<ThreadHit>& hits,
        const ThreadHit& candidate)
    {
        constexpr double coordinateToleranceMm = 0.25;

        for (const auto& existing : hits)
        {
            if (existing.threadName != candidate.threadName)
                continue;

            if (distanceBetweenPoints(existing.centerPoint, candidate.centerPoint)
                <= coordinateToleranceMm)
            {
                return true;
            }
        }

        return false;
    }
    static double distanceBetweenPoints(const gp_Pnt& a, const gp_Pnt& b)
    {
        double dx = a.X() - b.X();
        double dy = a.Y() - b.Y();
        double dz = a.Z() - b.Z();

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    struct CylindricalSample
    {
        double theta = 0.0;
        double z = 0.0;
        double radius = 0.0;
    };

    static optional<ThreadSize> matchKnownThread(
        double measuredRadiusInches,
        double measuredPitchInches)
    {
        const double radiusTolerance = 0.010; // inches
        const double pitchTolerance = 0.006;  // inches

        for (const auto& thread : knownThreadSizes())
        {
            double radiusError = fabs(measuredRadiusInches - thread.majorRadiusInches);
            double pitchError = fabs(measuredPitchInches - thread.pitchInches);

            if (radiusError <= radiusTolerance && pitchError <= pitchTolerance)
                return thread;
        }

        return nullopt;
    }

    static optional<double> estimateHelicalPitchFromFaceEdges(
        const TopoDS_Face& face,
        const gp_Cylinder& cylinder)
    {
        vector<CylindricalSample> samples;

        for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE);
            edgeExplorer.More();
            edgeExplorer.Next())
        {
            TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());

            Standard_Real first = 0.0;
            Standard_Real last = 0.0;

            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

            if (curve.IsNull())
                continue;

            constexpr int sampleCount = 40;

            vector<CylindricalSample> edgeSamples;

            for (int i = 0; i <= sampleCount; ++i)
            {
                double t = first + (last - first) * static_cast<double>(i) / sampleCount;
                gp_Pnt p = curve->Value(t);

                edgeSamples.push_back(toCylindricalSample(p, cylinder));
            }

            unwrapAngles(edgeSamples);

            for (const auto& s : edgeSamples)
                samples.push_back(s);
        }

        if (samples.size() < 10)
            return nullopt;

        unwrapAngles(samples);

        double thetaMin = numeric_limits<double>::max();
        double thetaMax = -numeric_limits<double>::max();

        for (const auto& s : samples)
        {
            thetaMin = min(thetaMin, s.theta);
            thetaMax = max(thetaMax, s.theta);
        }

        double totalAngle = fabs(thetaMax - thetaMin);

        // A true thread-like helix should usually sweep a meaningful angle.
        // This threshold is about 90 degrees.
        if (totalAngle < M_PI / 2.0)
            return nullopt;

        auto slope = linearRegressionSlopeThetaToZ(samples);

        if (!slope.has_value())
            return nullopt;

        // z = m * theta + b
        // One full turn is 2*pi radians, so pitch = m * 2*pi.
        double pitchMm = fabs(slope.value() * 2.0 * M_PI);

        if (pitchMm <= 0.0)
            return nullopt;

        return pitchMm;
    }

    static CylindricalSample toCylindricalSample(
        const gp_Pnt& point,
        const gp_Cylinder& cylinder)
    {
        gp_Ax3 position = cylinder.Position();

        gp_Pnt origin = position.Location();
        gp_Dir xDirection = position.XDirection();
        gp_Dir yDirection = position.YDirection();
        gp_Dir zDirection = position.Direction();

        gp_Vec v(origin, point);

        double x = v.Dot(gp_Vec(xDirection));
        double y = v.Dot(gp_Vec(yDirection));
        double z = v.Dot(gp_Vec(zDirection));

        CylindricalSample sample;
        sample.theta = atan2(y, x);
        sample.z = z;
        sample.radius = sqrt(x * x + y * y);

        return sample;
    }

    static void unwrapAngles(vector<CylindricalSample>& samples)
    {
        if (samples.empty())
            return;

        double offset = 0.0;
        double previous = samples[0].theta;

        for (size_t i = 1; i < samples.size(); ++i)
        {
            double current = samples[i].theta;
            double delta = current - previous;

            if (delta > M_PI)
                offset -= 2.0 * M_PI;
            else if (delta < -M_PI)
                offset += 2.0 * M_PI;

            samples[i].theta += offset;
            previous = current;
        }
    }

    static optional<double> linearRegressionSlopeThetaToZ(
        const vector<CylindricalSample>& samples)
    {
        if (samples.size() < 2)
            return nullopt;

        double sumTheta = 0.0;
        double sumZ = 0.0;
        double sumThetaZ = 0.0;
        double sumTheta2 = 0.0;

        for (const auto& s : samples)
        {
            sumTheta += s.theta;
            sumZ += s.z;
            sumThetaZ += s.theta * s.z;
            sumTheta2 += s.theta * s.theta;
        }

        double n = static_cast<double>(samples.size());

        double denominator = n * sumTheta2 - sumTheta * sumTheta;

        if (fabs(denominator) < 1e-9)
            return nullopt;

        double slope = (n * sumThetaZ - sumTheta * sumZ) / denominator;

        return slope;
    }
};