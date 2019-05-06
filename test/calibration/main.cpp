
#include <iostream>



#include <core/buffers/bufferFactory.h>

#include "core/stereointerface/processor6D.h"
#include "core/stereointerface/dummyFlowProcessor.h"

#include "core/utils/global.h"

#ifdef WITH_OPENCV
#include "KLTFlow.h"
#endif



#ifdef WITH_LIBJPEG
#include "libjpegFileReader.h"
#endif
#ifdef WITH_LIBPNG
#include "libpngFileReader.h"
#endif


#include "core/kltflow/kltGenerator.h"

#include "core/math/mathUtils.h"
#include "core/buffers/g12Buffer.h"
#include "core/buffers/kernels/spatialGradient.h"
#include "core/buffers/bufferFactory.h"
#include "core/fileformats/bmpLoader.h"
#include "core/segmentation/segmentator.h"
#include "core/buffers/rgb24/rgb24Buffer.h"

#include "core/buffers/convolver/convolver.h"

#include "openCvCheckerboardDetector.h"
#include "core/buffers/g8Buffer.h"
#include "openCVTools.h"
#include <opencv2/calib3d/calib3d.hpp>  // findChessboardCorners
#include "core/cameracalibration/flatPatternCalibrator.h"

using namespace std;
using namespace corecvs;

RGB24Buffer *drawCorners(G8Buffer *image, ObservationList corners) {
    RGB24Buffer *result = new RGB24Buffer(image);
    for (int i = 0; i < corners.size(); i++) {
        PointObservation p = corners.at(i);
        result->drawCrosshare3(p.u(), p.v(), RGBColor::Blue());
    }

    return result;
}

int main(int argc, char *argv[])
{
#ifdef WITH_LIBJPEG
    LibjpegFileReader::registerMyself();
    LibjpegFileSaver::registerMyself();
    SYNC_PRINT(("Libjpeg support on\n"));
#endif
#ifdef WITH_LIBPNG
    LibpngFileReader::registerMyself();
    LibpngFileSaver::registerMyself();
    SYNC_PRINT(("Libpng support on\n"));
#endif

    /* convertor from jpg
    for (int i = 1; i < 13; i++) {
        char inname[40];
        sprintf(inname, "chessboard/%d.jpg", i);
        char outname[40];
        sprintf(outname, "chessboard/%d.bmp", i);
        RGB24Buffer *inn = BufferFactory::getInstance()->loadRGB24Bitmap(inname);
        BufferFactory::getInstance()->saveRGB24Bitmap(inn, outname);
    }

    return -1;
    */

    BoardAlignerParams bap;
    bap.idealHeight = 9;
    bap.idealWidth = 6;
    bap.type = AlignmentType::FIT_ALL;

    CheckerboardDetectionParameters cbdp;
    cbdp.setPreciseDiameter(11);
//    cbdp.print();
//    return 0;
    bool saveDistortedImagesWithChessboard = false;


    OpenCvCheckerboardDetector detector(cbdp, bap);
    PinholeCameraIntrinsics intrinsics(300.0, 300.0, 320.0, 240.0, 0.0, Vector2dd(640, 480), Vector2dd(640, 480));
    LineDistortionEstimatorParameters distortion(LineDistortionEstimatorCost::LINE_DEVIATION_COST, 1000, 3, false/* ??? */, false, false, false);
//    distortion.print();
//    return 0;
    CameraConstraints constraints = CameraConstraints::DEFAULT | CameraConstraints::UNLOCK_DISTORTION;
    FlatPatternCalibrator calibrator(constraints, intrinsics, distortion, 1.0);

    char filename[40];
    sprintf(filename,  "chessboard/%d.bmp", 1);
    G12Buffer *imageG12 = BufferFactory::getInstance()->loadG12Bitmap(filename);

    if (imageG12 == NULL) {
        return -1;
    }


    int i = 2;
    while (imageG12 != NULL) {
        G8Buffer *imageG8 = G8Buffer::FromG12Buffer(imageG12);
        bool found = detector.detectPattern(*imageG8);

        if (found) {
            ObservationList currentFramePoints;
            detector.getPointData(currentFramePoints);
            calibrator.addPattern(currentFramePoints);

            if (saveDistortedImagesWithChessboard) {
                RGB24Buffer *toSave = drawCorners(imageG8, currentFramePoints);
                char outname[40];
                int j = i - 1;
                sprintf(outname, "chessboard/out%d.bmp", j);
                BufferFactory::getInstance()->saveRGB24Bitmap(toSave, outname);
            }
        }
        sprintf(filename,  "chessboard/%d.bmp", i);

        delete_safe(imageG12);
        delete_safe(imageG8);
        imageG12 = BufferFactory::getInstance()->loadG12Bitmap(filename);

        i++;
    }

    calibrator.solve();
//    calibrator.solve();
    cout << "\nIntrinsics: \n";
    calibrator.getIntrinsics().print();
    cout << "\n Distortion: \n";
    calibrator.getDistortion().print();
    cout << endl << "error: " << endl;
    cout << calibrator.getRmseReprojectionError();
    vector<double> koefs = calibrator.getDistortion().koeff();

    cout << "\n" << calibrator.getIntrinsics().cx()
         << " " << calibrator.getIntrinsics().cy()
         << " " << calibrator.getIntrinsics().fx()
         << " " << koefs[0] << " " << koefs[1] << " 0.0 0.0 " << koefs[2] << endl;

    return 0;
}
