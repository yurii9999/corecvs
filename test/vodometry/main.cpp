#include <iostream>

#include <core/buffers/bufferFactory.h>

#include "core/utils/global.h"

#include "KLTFlow.h"

#include <stdio.h>


#include "libpngFileReader.h"

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"

#include "openCvFileReader.h"
#include "openCvKeyPointsWrapper.h"
#include "openCVTools.h"

#include <exception>
#include <sstream>

#include <opencv2/core/core.hpp>        // Mat
#include <opencv2/highgui/highgui.hpp>  // imread

#include "ertfinder.h"

using namespace std;
using namespace corecvs;
using namespace cv;

int main()
{
    Processor6D *proc = new OpenCVFlowProcessor;
    ertFinder finder(550, 320, 240);

    LibpngFileReader::registerMyself();
    LibpngFileSaver::registerMyself();

    char filename[40];
    sprintf(filename,  "votest/%d.png", 1);
    RGB24Buffer *fstFrame = BufferFactory::getInstance()->loadRGB24Bitmap(filename);
    proc->beginFrame();
    proc->setFrameRGB24(Processor6D::FRAME_LEFT_ID, fstFrame);
    proc->endFrame();

    Matrix33 R_f = Matrix33::Identity();
    Vector3dd t_f(0, 0, 0);

    int i = 2;

    RGB24Buffer traj(700, 1000);

    clock_t begin = clock();

    while (true) {
        RGB24Buffer *in = BufferFactory::getInstance()->loadRGB24Bitmap(filename);
        if (in == NULL) {
            break;
        }


        proc->beginFrame();
        proc->setFrameRGB24(Processor6D::FRAME_LEFT_ID, in);
        proc->endFrame();
        FlowBuffer *flow = proc->getFlow();

        Matrix33 E, R;
        Vector3dd t;

        finder.findEssentialMatrix(&E, flow);
        finder.findRt(&R, &t, E, flow);

        R_f *= R;
        t_f += 2 * (R_f * t);

//        cout << "\nFrame " << i << "has read\n" << "E matrix: \n" << E << "\nR matrix: \n" << R << "\nt vector: \n" << t;


//        delete_safe(in1);
        char outname[24];
        sprintf(outname,  "result/%d.png", i);

        i++;
        sprintf(filename,  "votest/%d.png", i);

        int x = int(t_f.x()) + 500;
        int y = int(t_f.y()) + 350;
        traj.drawPixel(x, y, RGBColor::Red());


        RGB24Buffer c(in);
        c.drawFlowBuffer3(flow);
        BufferFactory::getInstance()->saveRGB24Bitmap(&c, outname);

    }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Total time taken: " << elapsed_secs << "s" << endl;

    BufferFactory::getInstance()->saveRGB24Bitmap(&traj, "result/result.png");

//    cout << "\nR_f: \n" << R_f << "\n t_f: \n" << t_f;
    return 0;
}
