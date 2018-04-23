#include <iostream>
#include <string>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "alpr.h"

// compares binary images of license plates
// returns a distance score between 0 and 1
float compPltBinImg(cv::Mat& oImgPltPrb, cv::Mat& oImgPltGall, int nAdpThldTyp, int nBlkSz, double fSubConst, int nRandNum, int nRandRng, cv::Size oPltSz)
{
    int nOfst, nNonZero, nNonZeroMin = oPltSz.area();
    cv::Mat oImgPltGallRand, oImgPltCmb, oImgPltDisp, oPersTrans;
    cv::Point2f oPtCoord[] = { cv::Point2f(0, 0), cv::Point2f((oPltSz.width - 1), 0), cv::Point2f((oPltSz.width - 1), (oPltSz.height - 1)), cv::Point2f(0, (oPltSz.height - 1)) };
    cv::Point2f oPtCoordRand[4];

    // get binary image of the probe
    cv::cvtColor(oImgPltPrb, oImgPltPrb, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(oImgPltPrb, oImgPltPrb, 255, nAdpThldTyp, cv::THRESH_BINARY_INV, nBlkSz, fSubConst);
    //cv::threshold(oImgPltPrb, oImgPltPrb, nBinPltThldPrb, 255, cv::THRESH_BINARY_INV);
    //cv::cvtColor(oImgPltPrb, oImgPltDisp, cv::COLOR_GRAY2BGR);
    //cv::imwrite("./data/lp_prb.jpg", oImgPltDisp);

    // get binary image of the gallery
    cv::cvtColor(oImgPltGall, oImgPltGall, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(oImgPltGall, oImgPltGall, 255, nAdpThldTyp, cv::THRESH_BINARY_INV, nBlkSz, fSubConst);
    //cv::threshold(oImgPltGall, oImgPltGall, nBinPltThldGall, 255, cv::THRESH_BINARY_INV);
    //cv::cvtColor(oImgPltGall, oImgPltDisp, cv::COLOR_GRAY2BGR);
    //cv::imwrite("./data/lp_gall.jpg", oImgPltDisp);

    // generate random perspective transform of the gallery
    for (int i = 0; i < nRandNum; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            nOfst = (rand() % (nRandRng * 2)) - nRandRng;
            oPtCoordRand[j].x = oPtCoord[j].x + nOfst;
            nOfst = (rand() % (nRandRng * 2)) - nRandRng;
            oPtCoordRand[j].y = oPtCoord[j].y + nOfst;
        }

        oPersTrans = cv::getPerspectiveTransform(oPtCoord, oPtCoordRand);
        cv::warpPerspective(oImgPltGall, oImgPltGallRand, oPersTrans, oImgPltGall.size());
        //cv::cvtColor(oImgPltGallRand, oImgPltDisp, cv::COLOR_GRAY2BGR);
        //char acOutImgPltRandPth[128] = {};
        //std::sprintf(acOutImgPltRandPth, "./data/lp_gall_rand%d.jpg", i);
        //cv::imwrite(acOutImgPltRandPth, oImgPltDisp);

        // find minimum overlap between the probe image and the warped gallery image
        cv::bitwise_or(oImgPltPrb, oImgPltGallRand, oImgPltCmb);
        //cv::cvtColor(oImgPltCmb, oImgPltDisp, cv::COLOR_GRAY2BGR);
        //char acOutImgPltCmbPth[128] = {};
        //std::sprintf(acOutImgPltCmbPth, "./data/lp_cmb%d.jpg", i);
        //cv::imwrite(acOutImgPltCmbPth, oImgPltDisp);
        nNonZero = cv::countNonZero(oImgPltCmb);

        if (nNonZeroMin > nNonZero)
        {
            nNonZeroMin = nNonZero;
            //std::printf("current random index: %d\n", i);
        }
    }

    return ((float)nNonZeroMin / (float)oPltSz.area());
}

int main(int argc, char *argv[])
{
    // adaptive thresholding algorithm to use, see cv::AdaptiveThresholdTypes
    int nAdpThldTyp = cv::ADAPTIVE_THRESH_MEAN_C;
    //int nAdpThldTyp = cv::ADAPTIVE_THRESH_GAUSSIAN_C;
    // size of a pixel neighborhood that is used to calculate a threshold value for the pixel: 3, 5, 7, and so on
    int nBlkSz = 31;
    // constant subtracted from the mean or weighted mean . Normally, it is positive but may be zero or negative as well.
    double fSubConst = 3.0;
    // number of random perspective transforms for comparison of binary images: higher -> higher probability to find the optimum match
    int nRandNum = 200;
    // range in pixels for the four corner points of license plate to generate random perspectives: higher -> larger variance
    int nRandRng = 30;
    // predefined size of the license plate
    cv::Size oPltSz(304, 152);

    cv::Mat oImgPltPrb = cv::imread("./data/015032_lp.jpg");
    cv::resize(oImgPltPrb, oImgPltPrb, oPltSz);
    cv::Mat oImgPltGall = cv::imread("./data/8742_lp.jpg");
    cv::resize(oImgPltGall, oImgPltGall, oPltSz);

    float fDistScr = compPltBinImg(oImgPltPrb, oImgPltGall, nAdpThldTyp, nBlkSz, fSubConst, nRandNum, nRandRng, oPltSz);

    std::cout << "distance score: " << fDistScr << std::endl;
    return 0;
}
