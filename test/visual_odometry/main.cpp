#include <iostream>
#include <stdio.h>
#include <fstream>

#include "KLTFlow.h"
#include "libpngFileReader.h"

#include "core/rectification/ransacEstimator.h"
#include <core/buffers/bufferFactory.h>

#include "core/tinyxml2/tinyxml2.h"

using namespace tinyxml2;
using namespace std;
using namespace corecvs;

enum Method {
    GET_ESSENTIAL_RANSAC,
    GET_FUNDAMENTAL_RANSAC
};

/* ctrl c ctrl v */
double readDouble(XMLElement *node, char *name) {
    XMLElement *e = node->FirstChildElement(name);
    if (e == NULL) {
        SYNC_PRINT(("Document is malformed"));
        return 0;
    } else {
        cout << "Text <" << e->GetText() << ">" << endl;
    }
    return std::stod(e->GetText());
}

int readInt (XMLElement *node, char *name) {
    XMLElement *e = node->FirstChildElement(name);
    if (e == NULL) {
        SYNC_PRINT(("Document is malformed"));
        return 0;
    } else {
        cout << "Text <" << e->GetText() << ">" << endl;
    }
    return stoi(e->GetText());
}

string readString (XMLElement *node, char *name) {
    XMLElement *e = node->FirstChildElement(name);
    if (e == NULL) {
        SYNC_PRINT(("Document is malformed"));
        return 0;
    } else {
        cout << "Text <" << e->GetText() << ">" << endl;
    }
    return e->GetText();
}

int main(int argc, char *argv[])
{
#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    LibjpegFileSaver::registerMyself();
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    LibpngFileSaver::registerMyself();
#endif

    /* camera */
    double focal = 1;
    double x0 = 0;
    double y0 = 0;

    /* ransac */
    int maxIterations = 1;
    int trySize = 1;
    double threshold = 1.0;
    int algorithm = 0;

    /* optical flow proc */

    /* other */
    bool trace = false;
    string sequence = "sequence.txt";
    string outname = "trajectry.png";

    string filename = argc > 1 ? argv[1] : "default.xml";

    XMLDocument* doc = new XMLDocument();
    doc->LoadFile(filename.c_str());

    XMLElement *node = doc->FirstChildElement("params");
    focal = readDouble(node, "focal");
    x0 = readDouble(node, "x0");
    y0 = readDouble(node, "y0");
    maxIterations = readInt(node, "maxIterations");
    trySize = readInt(node, "trySize");
    algorithm = readInt(node, "algorithm");
    threshold = readDouble(node, "threshold");
    sequence = readString(node, "sequence");
    outname = readString(node, "outname");

    /* do st with it */
    Method method = GET_FUNDAMENTAL_RANSAC;

    RansacEstimator *ransacEstimator = new RansacEstimator(trySize, maxIterations, threshold);
    ransacEstimator->algorithm = algorithm;

    Processor6D *proc = new OpenCVFlowProcessor();

    ifstream fin(sequence);
    PinholeCameraIntrinsics intrinsics(focal, focal, x0, y0);
    Matrix33 K = intrinsics.getKMatrix33();
    ProjectiveTransform Kinv(intrinsics.getInvKMatrix33());

    cout << "\nRuns with:: \nfocal:: " << focal << "\t x0:: " << x0 << "\t y0:: " << y0;
    cout << "\nmaxIterations:: " << maxIterations << "\t trySize:: " << trySize << "\t Threshold:: " << threshold;
    cout << "\nFrom:: " << sequence << "\t To:: " << outname;
    SYNC_PRINT((""));

    RGB24Buffer *trajectory = new RGB24Buffer(1000, 1000);
    Matrix33 R_f = Matrix33::Identity();
    Vector3dd t_f = Vector3dd::Zero();

    string nextFrameName;
    fin >> nextFrameName;
    RGB24Buffer *nextFrame = BufferFactory::getInstance()->loadRGB24Bitmap(nextFrameName);
    proc->beginFrame();
    proc->setFrameRGB24(Processor6D::FRAME_LEFT_ID, nextFrame);
    proc->endFrame();

    while (!fin.eof()) {
        fin >> nextFrameName;
        nextFrame = BufferFactory::getInstance()->loadRGB24Bitmap(nextFrameName);
        if (nextFrame == NULL)
            continue;

        proc->beginFrame();
        proc->setFrameRGB24(Processor6D::FRAME_LEFT_ID, nextFrame);
        proc->endFrame();

        FlowBuffer *flow = proc->getFlow();
        CorrespondenceList list(flow);
        list.transform(Kinv, Kinv);
        vector<Correspondence *> data;
        for (int i = 0; i < list.size(); i++) {
            data.push_back(&list.operator [](i));
        }

        EssentialMatrix E;
        if (method == Method::GET_FUNDAMENTAL_RANSAC) {
            Matrix33 F = ransacEstimator->getFundamentalRansac(&data);
            E = EssentialMatrix(F);
        }
        else {
            E = EssentialMatrix(ransacEstimator->getEssentialRansac(&data));
        }


        EssentialDecomposition ed[4];
        EssentialDecomposition final = E.decompose(&data, ed);

        if (trace)
            cout << "\nRotate by:: \n" << final.rotation << "\nand shift by:: \n" << final.direction;
        R_f *= final.rotation;
        t_f += (R_f * final.direction);

        Vector2dd current = t_f.xz();
        trajectory->drawCrosshare3(current + Vector2dd(500, 500), RGBColor::Red());
    }

    BufferFactory::getInstance()->saveRGB24Bitmap(trajectory, outname);

    return 0;
}
