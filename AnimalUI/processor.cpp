/*
Copyright (c) 2007 Ilya Baran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

// #include <FL/fl_ask.H>
#include <fstream>

#include "processor.h"
#include "../Libs/Pinocchio/skeleton.h"
#include "../Libs/Pinocchio/utils.h"
#include "../Libs/Pinocchio/debugging.h"
#include "../Libs/Pinocchio/attachment.h"
#include "../Libs/Pinocchio/pinocchioApi.h"
#include "defmesh.h"
#include "motion.h"
#include "animalSkeleton.h"
#include "../Eigen.h"

struct ArgData
{
    ArgData() : stopAtMesh(false), stopAfterCircles(false), skelScale(1.), noFit(false),
                skeleton(HumanSkeleton())
    {
    }

    bool stopAtMesh;
    bool stopAfterCircles;
    string filename;
    string motionname;
    Quaternion<> meshTransform;
    double skelScale;
    bool noFit;
    Skeleton skeleton;
    string skeletonfilename;
};

void printUsageAndExit()
{
    cout << "Usage: DemoUI filename.{obj | ply | off | gts | stl}" << endl;
    cout << "              [-skel skelname]" << endl;
    cout << "              [-motion motionname]" << endl;

    exit(0);
}

ArgData processArgs(const vector<string> &args)
{
    ArgData out;
    int cur = 2;
    int num = args.size();
    if (num < 2)
        printUsageAndExit();

    out.filename = args[1];

    while (cur < num)
    {
        string curStr = args[cur++];
        std::cout << curStr << std::endl;
        if (curStr == string("-skel"))
        {
            if (cur == num)
            {
                cout << "No skeleton specified; ignoring." << endl;
                continue;
            }
            curStr = args[cur++];
            out.skeletonfilename = curStr;
            out.skeleton = AnimalSkeleton(out.skeletonfilename);
            // out.skeleton = FileSkeleton(out.skeletonfilename);
            // out.skeleton = HorseSkeleton();
            continue;
        }
        if (curStr == string("-rot"))
        {
            if (cur + 3 >= num)
            {
                cout << "Too few rotation arguments; exiting." << endl;
                printUsageAndExit();
            }
            double x, y, z, deg;
            sscanf(args[cur++].c_str(), "%lf", &x);
            sscanf(args[cur++].c_str(), "%lf", &y);
            sscanf(args[cur++].c_str(), "%lf", &z);
            sscanf(args[cur++].c_str(), "%lf", &deg);

            out.meshTransform = Quaternion<>(Vector3(x, y, z), deg * M_PI / 180.) * out.meshTransform;
            continue;
        }
        if (curStr == string("-motion"))
        {
            if (cur == num)
            {
                cout << "No motion filename specified; ignoring." << endl;
                continue;
            }
            out.motionname = args[cur++];
            continue;
        }
        cout << "Unrecognized option: " << curStr << endl;
        printUsageAndExit();
    }

    return out;
}

Eigen::MatrixXd process(const std::vector<std::string> &args)
{
    int i;
    ArgData a = processArgs(args);

    Debugging::setOutStream(cout);

    Mesh m(a.filename);
    if (m.vertices.size() == 0)
    {
        cout << "Error reading file.  Aborting." << endl;
        exit(0);
        return;
    }

    for (i = 0; i < (int)m.vertices.size(); ++i)
        m.vertices[i].pos = a.meshTransform * m.vertices[i].pos;
    m.normalizeBoundingBox();
    m.computeVertexNormals();

    Skeleton given = a.skeleton;
    given.scale(a.skelScale * 1.0);

    a.stopAtMesh = true;
    if (a.stopAtMesh)
    { // if early bailout
        // w->addMesh(new StaticDisplayMesh(m));
    }

    for (int i = 0; i < given.fGraph().verts.size(); i++)
    {
        vector<int> verts = given.fGraph().edges[i];
        for (int j = 0; j < verts.size(); j++)
        {
            if (verts[j] > i)
            {
                std::cout << i << ":" << given.fGraph().verts[i] << " " << verts[j] << ":" << given.fGraph().verts[verts[j]] << std::endl;
                // w->addLine(LineSegment(given.fGraph().verts[i], given.fGraph().verts[verts[j]], Vector3(.5, .5, 0), 4.));
            }
        }
        // w->addPoint(given.fGraph().verts[i]);
    }

    PinocchioOutput o;
    o = autorig(given, m);

    if (o.embedding.size() == 0)
    {
        cout << "Error embedding" << endl;
        exit(0);
    }

    if (a.motionname.size() > 0)
    {
        // w->addMesh(new DefMesh(m, given, o.embedding, *(o.attachment), new Motion(a.motionname)));
    }
    else
    {
        // w->addMesh(new StaticDisplayMesh(m));

        for (i = 1; i < (int)o.embedding.size(); ++i)
        {
            // w->addLine(LineSegment(o.embedding[i], o.embedding[given.fPrev()[i]], Vector3(.5, .5, 0), 4.));
        }
    }

    // // output skeleton embedding
    // for (i = 0; i < (int)o.embedding.size(); ++i)
    //     o.embedding[i] = (o.embedding[i] - m.toAdd) / m.scale;
    // ofstream os("./data/skeleton.out");
    // for (i = 0; i < (int)o.embedding.size(); ++i)
    // {
    //     os << i << " " << o.embedding[i][0] << " " << o.embedding[i][1] << " " << o.embedding[i][2] << " " << a.skeleton.fPrev()[i] << endl;
    // }

    // output attachment
    // std::ofstream astrm("./data/attachment.out");

    std::Vector<double, -1> v = o.attachment->getWeights(0);
    Eigen::MatrixXd c((int)m.vertices.size(),v.size());

    for (i = 0; i < (int)m.vertices.size(); ++i)
    {
        std::Vector<double, -1> v = o.attachment->getWeights(i);
        for (int j = 0; j < v.size(); ++j)
        {
            double d = floor(0.5 + v[j] * 10000.) / 10000.;
            // astrm << d << " ";
            c(i,j) = d;
        }
        // astrm << endl;
    }
    return c;
    // delete o.attachment;
}
