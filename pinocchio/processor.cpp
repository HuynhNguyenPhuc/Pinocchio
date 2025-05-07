#include <iostream>
#include <fstream>
#include <filesystem>

#include "processor.h"
#include "skeleton.h"
#include "utils.h"
#include "debugging.h"
#include "attachment.h"
#include "pinocchioApi.h"

namespace fs = std::filesystem;

struct ArgData
{
    ArgData() :
        stopAtMesh(false), stopAfterCircles(false), skelScale(1.), noFit(false),
        skeleton(HumanSkeleton()), outputDir(".")
    {
    }
    
    bool stopAtMesh;
    bool stopAfterCircles;
    std::string filename;
    Quaternion<> meshTransform;
    double skelScale;
    bool noFit;
    Skeleton skeleton;
    std::string skeletonName;
    int skinAlgorithm;   // Indicates which skinning algorithm to use
    float blendWeight;  // Indicates the blending weight for MIX algorithm
    std::string outputDir; // Directory to store output files
};

void printUsageAndExit()
{
    std::cout << "Usage: DemoUI filename.{obj | ply | off | gts | stl}" << std::endl;
    std::cout << "              [-outdir directory]" << std::endl;
    std::cout << "              [-skel skelname] [-rot x y z deg]* [-scale s]" << std::endl;
    std::cout << "              [-meshonly | -mo] [-circlesonly | -co]" << std::endl;
    std::cout << "              [-algo skinning_algorithm [blend_weight]]" << std::endl;
    exit(0);
}

ArgData processArgs(const std::vector<std::string> &args)
{
    ArgData out;
    int cur = 2;
    int num = args.size();
    if(num < 2)
        printUsageAndExit();

    out.filename = args[1];
    // set default skinning algorithm as LBS, and enlargelinebreak default blending weight as 0.5
    out.skinAlgorithm = Mesh::LBS;
    out.blendWeight = 0.5;

    while(cur < num) {
        std::string curStr = args[cur++];
        if(curStr == std::string("-skel")) {
            if(cur == num) {
                std::cout << "No skeleton specified; ignoring." << std::endl;
                continue;
            }
            curStr = args[cur++];
            if(curStr == std::string("human"))
                out.skeleton = HumanSkeleton();
            else if(curStr == std::string("horse"))
                out.skeleton = HorseSkeleton();
            else if(curStr == std::string("quad"))
                out.skeleton = QuadSkeleton();
            else if(curStr == std::string("centaur"))
                out.skeleton = CentaurSkeleton();
            else
                out.skeleton = FileSkeleton(curStr);
            out.skeletonName = curStr;
        } else if(curStr == std::string("-rot")) {
            if(cur + 3 >= num) {
                std::cout << "Too few rotation arguments; exiting." << std::endl;
                printUsageAndExit();
            }
            double x, y, z, deg;
            sscanf(args[cur++].c_str(), "%lf", &x);
            sscanf(args[cur++].c_str(), "%lf", &y);
            sscanf(args[cur++].c_str(), "%lf", &z);
            sscanf(args[cur++].c_str(), "%lf", &deg);
            
            out.meshTransform = Quaternion<>(Vector3(x, y, z), deg * M_PI / 180.) * out.meshTransform;
        } else if(curStr == std::string("-scale")) {
            if(cur >= num) {
                std::cout << "No scale provided; exiting." << std::endl;
                printUsageAndExit();
            }
            sscanf(args[cur++].c_str(), "%lf", &out.skelScale);
        } else if(curStr == std::string("-meshonly") || curStr == std::string("-mo")) {
            out.stopAtMesh = true;
        } else if(curStr == std::string("-circlesonly") || curStr == std::string("-co")) {
            out.stopAfterCircles = true;
        } else if(curStr == std::string("-nofit")) {
            out.noFit = true;
        } else if(curStr == std::string("-algo")) {
            std::string algo = args[cur++];
            if (algo == std::string("LBS"))
                out.skinAlgorithm = Mesh::LBS;
            else if (algo == std::string("DQS"))
                out.skinAlgorithm = Mesh::DQS;
            else if (algo == std::string("MIX")) {
                if(cur >= num) {
                    std::cout << "No blending weight given; exiting." << std::endl;
                    printUsageAndExit();
                }
                out.skinAlgorithm = Mesh::MIX;
                sscanf(args[cur++].c_str(), "%f", &out.blendWeight);
            } else {
                std::cout << "Unrecognized skinning algorithm" << std::endl;
                printUsageAndExit();
            }
        } else if(curStr == std::string("-outdir")) {
            if(cur >= num) {
                std::cout << "No output directory specified; exiting." << std::endl;
                printUsageAndExit();
            }
            out.outputDir = args[cur++];
        } else {
            std::cout << "Unrecognized option: " << curStr << std::endl;
            printUsageAndExit();
        }
    }

    return out;
}

void process(const std::vector<std::string> &args) {
    ArgData a = processArgs(args);

    Mesh m(a.filename, a.skinAlgorithm, a.blendWeight);
    if (m.vertices.empty()) {
        std::cerr << "Error reading file " << a.filename << ". Aborting." << std::endl;
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
        std::cerr << "Error embedding. Aborting." << std::endl;
        return;
    }

    // Create output directory if it doesn't exist
    fs::create_directories(a.outputDir);

    // Write skeleton.out
    fs::path skeletonPath = fs::path(a.outputDir) / "skeleton.out";
    {
        std::ofstream os(skeletonPath.string());
        if (!os.is_open()) {
            std::cerr << "Error: Cannot open " << skeletonPath.string() << " for writing." << std::endl;
            return;
        }
        for (size_t i = 0; i < o.embedding.size(); ++i) {
            auto p = (o.embedding[i] - m.toAdd) / m.scale;
            os << i << " "
               << p[0] << " "
               << p[1] << " "
               << p[2] << " "
               << given.fPrev()[i] << "\n";
        }
    }

    // Write attachment.out
    fs::path attachmentPath = fs::path(a.outputDir) / "attachment.out";
    {
        std::ofstream astrm(attachmentPath.string());
        if (!astrm.is_open()) {
            std::cerr << "Error: Cannot open " << attachmentPath.string() << " for writing." << std::endl;
            return;
        }
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
    std::vector<std::string> args;
    for (int i = 0; i < argc; ++i)
        args.push_back(argv[i]);

    if (argc < 2) {
        std::cerr << args[0] << ": Usage: export_only filename.obj [options]" << std::endl;
        return 1;
    }
    process(args);
    ArgData a = processArgs(args);
    std::cout << "Export completed: " 
              << (fs::path(a.outputDir) / "skeleton.out").string() << ", "
              << (fs::path(a.outputDir) / "attachment.out").string() << std::endl;
    return 0;
}