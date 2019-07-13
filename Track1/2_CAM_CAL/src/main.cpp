//-----------------------------------------------------------------------------------
//
//  Copyright (c) 2018 Zheng Tang <zhtang@uw.edu>.  All rights reserved.
//
//  Description:
//      Implementation of camera calibration
//
//-----------------------------------------------------------------------------------

#include "Cfg.h"
#include "CamCal.h"

int main(int argc, char *argv[])
{
	cv::Mat oImgFrm;
	cv::Size oFrmSz;
	CCamCal oCamCal;
	CCfg oCfg;
	oCfg.ldCfgFl();

	// read input frame image
    char acInFrmPth[128] = {};
    std::strcpy(acInFrmPth, oCfg.getInFrmPth());
	oImgFrm = cv::imread(acInFrmPth, CV_LOAD_IMAGE_COLOR);

	// set frame size
	oFrmSz.width = oImgFrm.cols;
	oFrmSz.height = oImgFrm.rows;

	// resize frame if necessary
	if (0 >= oCfg.getRszFrmHei())
		oCfg.setFrmSz(oFrmSz);
	else
	{
		oFrmSz = cv::Size((((float)oFrmSz.width / (float)oFrmSz.height) * oCfg.getRszFrmHei()), oCfg.getRszFrmHei());
		oCfg.setFrmSz(oFrmSz);
	}

	// camera calibration
	oCamCal.initialize(oCfg, oImgFrm);
	oVanLnSel.initialize(oCfg, oImgFrm);
	std::vector<cv::Point> oVanPt = oVanLnSel.process();
	oCamCal.process(oVanPt);

	return 0;
}
