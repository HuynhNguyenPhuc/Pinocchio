#include <fstream>

#include "processor.h"
#include "skeleton.h"
#include "utils.h"
#include "debugging.h"
#include "attachment.h"
#include "pinocchioApi.h"

using namespace std;

struct ArgData
{
    ArgData() :
        stopAtMesh(false), stopAfterCircles(false), skelScale(1.), noFit(false),
        skeleton(HumanSkeleton())
    {
    }
    
    bool stopAtMesh;
    bool stopAfterCircles;
    string filename;
    Quaternion<> meshTransform;
    double skelScale;
    bool noFit;
    Skeleton skeleton;
    string skeletonname;
    int skinAlgorithm;	// Indicates which skinning algorithm to use
    float blendWeight;	// Indicates the blending weight for MIX algorithm
};

void printUsageAndExit()
{
    cout << "Usage: DemoUI filename.{obj | ply | off | gts | stl}" << endl;
    cout << "              [-skel skelname] [-rot x y z deg]* [-scale s]" << endl;
    cout << "              [-meshonly | -mo] [-circlesonly | -co]" << endl;
    cout << "              [-algo skinning_algorithm [blend_weight]]" << endl;

    exit(0);
}

ArgData processArgs(const vector<string> &args)
{
    ArgData out;
    int cur = 2;
    int num = args.size();
    if(num < 2)
        printUsageAndExit();

    out.filename = args[1];
    // set default skinning algorithm as LBS, and the default blending weight
	// as 0.5
    out.skinAlgorithm = Mesh::LBS;
    out.blendWeight = 0.5;

    while(cur < num) {
        string curStr = args[cur++];
        if(curStr == string("-skel")) {
            if(cur == num) {
                cout << "No skeleton specified; ignoring." << endl;
                continue;
            }
            curStr = args[cur++];
            if(curStr == string("human"))
                out.skeleton = HumanSkeleton();
            else if(curStr == string("horse"))
                out.skeleton = HorseSkeleton();
            else if(curStr == string("quad"))
                out.skeleton = QuadSkeleton();
            else if(curStr == string("centaur"))
                out.skeleton = CentaurSkeleton();
            else
                out.skeleton = FileSkeleton(curStr);
            out.skeletonname = curStr;
        } else if(curStr == string("-rot")) {
            if(cur + 3 >= num) {
                cout << "Too few rotation arguments; exiting." << endl;
                printUsageAndExit();
            }
            double x, y, z, deg;
            sscanf(args[cur++].c_str(), "%lf", &x);
            sscanf(args[cur++].c_str(), "%lf", &y);
            sscanf(args[cur++].c_str(), "%lf", &z);
            sscanf(args[cur++].c_str(), "%lf", &deg);
            
            out.meshTransform = Quaternion<>(Vector3(x, y, z), deg * M_PI / 180.) * out.meshTransform;
        } else if(curStr == string("-scale")) {
            if(cur >= num) {
                cout << "No scale provided; exiting." << endl;
                printUsageAndExit();
            }
            sscanf(args[cur++].c_str(), "%lf", &out.skelScale);
        } else if(curStr == string("-meshonly") || curStr == string("-mo")) {
            out.stopAtMesh = true;
        } else if(curStr == string("-circlesonly") || curStr == string("-co")) {
            out.stopAfterCircles = true;
        } else if (curStr == string("-nofit")) {
            out.noFit = true;
        } else if (curStr == string("-algo")) {
            /*  Option to use a different skinning algorithm than the
             *  default LBS. Currently, options are LBS, DQS, and MIX */
            string algo = args[cur++];
            if (algo == string("LBS"))
                out.skinAlgorithm = Mesh::LBS;
            else if (algo == string("DQS"))
                out.skinAlgorithm = Mesh::DQS;
            else if (algo == string("MIX")) {
                /*  Grab the desired blending weight for LBS, i.e
                 *  how much of the result of LBS you want to see */
                if(cur >= num) {
                    cout << "No blending weight given; exiting." << endl;
                    cout << args[cur] << endl;
                    printUsageAndExit();
                }
                out.skinAlgorithm = Mesh::MIX;
                sscanf(args[cur++].c_str(), "%f", &out.blendWeight);
            } else {
                cout << "Unrecognized skinning algorithm" << endl;
                printUsageAndExit();
            }
        } else {
            cout << "Unrecognized option: " << curStr << endl;
            printUsageAndExit();
        }
    }

    return out;
}

void process(const vector<string> &args) {
    ArgData a = processArgs(args);

    Mesh m(a.filename, a.skinAlgorithm, a.blendWeight);
    if (m.vertices.empty()) {
        cerr << "Error reading file. Aborting." << endl;
        return;
    }
 
    for (auto &v : m.vertices)
        v.pos = a.meshTransform * v.pos;
    m.normalizeBoundingBox();
    m.computeVertexNormals();

    Skeleton given = a.skeleton;
    given.scale(a.skelScale * 0.7);

    PinocchioOutput o;
    if (!a.noFit) {
        o = autorig(given, m);
    } else {
        auto *distanceField = constructDistanceField(m);
        VisTester<TreeType> tester(distanceField);

        o.embedding = a.skeleton.fGraph().verts;
        for (size_t i = 0; i < o.embedding.size(); ++i)
            o.embedding[i] = m.toAdd + o.embedding[i] * m.scale;

        o.attachment = new Attachment(m, a.skeleton, o.embedding, &tester);
        delete distanceField;
    }
    if (o.embedding.empty()) {
        cerr << "Error embedding. Aborting." << endl;
        return;
    }

    {
        ofstream os("skeleton.out");
        for (size_t i = 0; i < o.embedding.size(); ++i) {
            auto p = (o.embedding[i] - m.toAdd) / m.scale;
            os << i << " "
               << p[0] << " "
               << p[1] << " "
               << p[2] << " "
               << given.fPrev()[i] << "\n";
        }
    }

    {
        ofstream astrm("attachment.out");
        for (size_t i = 0; i < m.vertices.size(); ++i) {
            auto weights = o.attachment->getWeights(i);
            for (int j = 0; j < weights.size(); ++j) {
                double d = floor(0.5 + weights[j] * 10000.) / 10000.;
                astrm << d << " ";
            }
            astrm << "\n";
        }
    }

    delete o.attachment;
}

int main(int argc, char **argv) {
    vector<string> args;
    for (int i = 0; i < argc; ++i)
        args.push_back(argv[i]);

    if (argc < 2) {
        cerr << "Usage: export_only filename.obj [options]" << endl;
        return 1;
    }
    process(args);
    cout << "Export completed: skeleton.out, attachment.out" << endl;
    return 0;
}
