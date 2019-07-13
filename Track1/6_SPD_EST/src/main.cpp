//-----------------------------------------------------------------------------------
//
//  Copyright (c) 2018 Zheng Tang <zhtang@uw.edu>.  All rights reserved.
//
//  Description:
//      Implementation of 3D speed estimation
//
//-----------------------------------------------------------------------------------

#include <fstream>
//#include <direct.h>	// in Windows
#include <sys/stat.h>	// in Linux
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CTrkNd
{
public:
	CTrkNd(void);
	CTrkNd(int nFrmCnt, int nId, cv::Rect oBBox, float fDetScr, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, char* acDetCls, float fDep);
	~CTrkNd(void);

	inline int getFrmCnt(void) { return m_nFrmCnt; }
	inline void setFrmCnt(int nFrmCnt) { m_nFrmCnt = nFrmCnt; }
	inline int getId(void) { return m_nId; }
	inline void setId(int nId) { m_nId = nId; }
	inline cv::Rect getBBox(void) { return m_oBBox; }
	inline void setBBox(cv::Rect oBBox) { m_oBBox = oBBox; }
	inline float getDetScr(void) { return m_fDetScr; }
	inline void setDetScr(float fDetScr) { m_fDetScr = fDetScr; }
	inline cv::Point2f get2dFtPt(void) { return m_o2dFtPt; }
	inline void set2dFtPt(cv::Point2f o2dFtPt) { m_o2dFtPt = o2dFtPt; }
	inline cv::Point3f get3dFtPt(void) { return m_o3dFtPt; }
	inline void set3dFtPt(cv::Point3f o3dFtPt) { m_o3dFtPt = o3dFtPt; }
	inline char* getDetCls(void) { return m_acDetCls; }
	inline void setDetCls(char* acDetCls) { std::strcpy(m_acDetCls, acDetCls); }
	inline float getDep(void) { return m_fDep; }
	inline void setDep(float fDep) { m_fDep = fDep; }
	inline float getSpd(void) { return m_fSpd; }
	inline void setSpd(float fSpd) { m_fSpd = fSpd; }

private:
	//! frame count
	int m_nFrmCnt;
	//! object ID
	int m_nId;
	//! object bounding box
	cv::Rect m_oBBox;
	//! detection score
	float m_fDetScr;
	//! 2D foot point
	cv::Point2f m_o2dFtPt;
	//! 3D foot point (in meter)
	cv::Point3f m_o3dFtPt;
	//! class of object detection
	char m_acDetCls[32];
	//! distance to the camera (in meter)
	float m_fDep;
	//! speed (in mi/h)
	float m_fSpd;
};

CTrkNd::CTrkNd(void)
{

}

CTrkNd::CTrkNd(int nFrmCnt, int nId, cv::Rect oBBox, float fDetScr, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, char* acDetCls, float fDep)
{
	setFrmCnt(nFrmCnt);
	setId(nId);
	setBBox(oBBox);
	setDetScr(fDetScr);
	set2dFtPt(o2dFtPt);
	set3dFtPt(o3dFtPt);
	setDetCls(acDetCls);
	setDep(fDep);
}

CTrkNd::~CTrkNd(void)
{

}

bool cmpDep(CTrkNd oTrkNd1, CTrkNd oTrkNd2)	// used to sort tracking nodes in a list
{
	return oTrkNd1.getDep() < oTrkNd2.getDep();
}

bool cmpFrmCnt(CTrkNd oTrkNd1, CTrkNd oTrkNd2)	// used to sort tracking nodes in a list
{
	return oTrkNd1.getFrmCnt() < oTrkNd2.getFrmCnt();
}

// backprojects 2D point to 3D ground position
static cv::Point3f bkproj2d23d(cv::Point2f o2dPt, float afProjMat[12], int nLenUnit = 1000)
{
	cv::Point3f o3dPt;

	cv::Mat oMatA(3, 3, CV_64F);
	oMatA.at<double>(0, 0) = afProjMat[0];
	oMatA.at<double>(0, 1) = afProjMat[1];
	oMatA.at<double>(0, 2) = -o2dPt.x;
	oMatA.at<double>(1, 0) = afProjMat[4];
	oMatA.at<double>(1, 1) = afProjMat[5];
	oMatA.at<double>(1, 2) = -o2dPt.y;
	oMatA.at<double>(2, 0) = afProjMat[8];
	oMatA.at<double>(2, 1) = afProjMat[9];
	oMatA.at<double>(2, 2) = -1.0;

	cv::Mat oMatAInv(3, 3, CV_64F);
	cv::invert(oMatA, oMatAInv, cv::DECOMP_SVD);

	cv::Mat oMatB(3, 1, CV_64F);
	oMatB.at<double>(0, 0) = -afProjMat[3];
	oMatB.at<double>(1, 0) = -afProjMat[7];
	oMatB.at<double>(2, 0) = -afProjMat[11];

	cv::Mat oMatM(3, 1, CV_64F);
	oMatM = oMatAInv * oMatB;

	o3dPt = cv::Point3f(oMatM.at<double>(0, 0), oMatM.at<double>(1, 0), 0.0f) / nLenUnit;

	return o3dPt;
}

int main(int argc, char *argv[])
{
	// video ID
	int aiVdo[] = { 1, 2, 3, 4, 5, 6, 7, 8,
		9, 10, 11, 12, 13, 14, 15, 16,
		17, 18, 19, 20, 21, 22,
		23, 24, 25, 26, 27 };
	std::vector<int> viVdo (aiVdo, aiVdo + sizeof(aiVdo) / sizeof(int) );
	// camera ID (LocX_Y)
	std::string astrCam[] = { "Loc1_1", "Loc1_2", "Loc1_3", "Loc1_4", "Loc1_5", "Loc1_6", "Loc1_7", "Loc1_8",
		"Loc2_1", "Loc2_2", "Loc2_3", "Loc2_4", "Loc2_5", "Loc2_6", "Loc2_7", "Loc2_8",
		"Loc3_1", "Loc3_2", "Loc3_3", "Loc3_4", "Loc3_5", "Loc3_6",
		"Loc4_1", "Loc4_2", "Loc4_3", "Loc4_4", "Loc4_5" };
	std::vector<std::string> vstrCam (astrCam, astrCam + sizeof(astrCam) / sizeof(std::string) );
	// path to Track1 main folder
	char acTrk1FlrPth[256] = {};
	std::strcpy(acTrk1FlrPth, "./data/");
	// window (no. of frames) for the computation of average speed
	int anSpdWinSz[] = { 15, 15, 15, 15, 15, 15, 15, 15,
		15, 15, 15, 15, 15, 15, 15, 15,
		31, 31, 31, 31, 31, 31,
		31, 31, 31, 31, 31 };
	std::vector<int> vnSpdWinSz (anSpdWinSz, anSpdWinSz + sizeof(anSpdWinSz) / sizeof(int) );
	// scale for speed
	float afSpdScl[] = { 1.25f, 1.25f, 1.25f, 1.25f, 1.25f, 1.25f, 1.25f, 1.25f,
		1.05f, 1.05f, 1.05f, 1.05f, 1.05f, 1.05f, 1.05f, 1.05f,
		0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f,
		0.89f, 0.89f, 0.89f, 0.89f, 0.89f };
	std::vector<float> vfSpdScl (afSpdScl, afSpdScl + sizeof(afSpdScl) / sizeof(float) );
	// maximum threshold for standard deviation of speed to assume constant speed
	float afSpdStdThld[] = { 70.0f, 70.0f, 70.0f, 70.0f, 70.0f, 70.0f, 70.0f, 70.0f,
		70.0f, 70.0f, 70.0f, 70.0f, 70.0f, 70.0f, 70.0f, 70.0f,
		15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f,
		5.0f, 5.0f, 5.0f, 5.0f, 5.0f };
	std::vector<float> vfSpdStdThld (afSpdStdThld, afSpdStdThld + sizeof(afSpdStdThld) / sizeof(float) );
	// minimum threshold for speed to apply constant speed strategy 
	float afSpdLowThld[] = { 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
		10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
		28.0f, 28.0f, 28.0f, 28.0f, 28.0f, 28.0f,
		18.0f, 18.0f, 18.0f, 18.0f, 18.0f };
	std::vector<float> vfSpdLowThld (afSpdLowThld, afSpdLowThld + sizeof(afSpdLowThld) / sizeof(float) );
	// maximum threshold for speed to assume the car is stopping
	float afSpdStpThld[] = { 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
		2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
		5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f,
		5.0f, 5.0f, 5.0f, 5.0f, 5.0f };
	std::vector<float> vfSpdStpThld (afSpdStpThld, afSpdStpThld + sizeof(afSpdStpThld) / sizeof(float) );
	// maximum threshold for propagation of speed for false negatives
	float afSpdPropFNThld [] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	std::vector<float> vfSpdPropFNThld  (afSpdPropFNThld , afSpdPropFNThld  + sizeof(afSpdPropFNThld) / sizeof(float) );
	// flag to output a complete video sequence
	bool bOutVdoFlg = false;
	cv::VideoWriter oVdoWrt;
	// flag to output plotted images (disable will accelerate the processing speed a lot)
	bool bOutTrk3dImgFlg = false;
	// threshold of detection score
	float fDetScrThld = 10.0f;
	// frame rate
	float fFrmRt = 30.0f;
	// frame size
	cv::Size oFrmSz(1920, 1080);
	// no. of frames to plot past trajectory of each vehicle
	int nPltTrajLenMax = 60;
	// path to the submission txt file
	char acOutSubmPth[256] = {};
	char acOutSubmNm[256] = {};
	std::strcpy(acOutSubmPth, acTrk1FlrPth);
	std::strcpy(acOutSubmNm, "track1.txt");
	std::strcat(acOutSubmPth, acOutSubmNm);
	FILE* pfOutSubmTxt = std::fopen(acOutSubmPth, "w");
	// path to output video file
	if (bOutVdoFlg)
	{
		char acOutVdoPth[256] = {};
		char acOutVdoNm[256] = {};
		std::strcpy(acOutVdoPth, acTrk1FlrPth);
		std::strcpy(acOutVdoNm, "track1.avi");
		std::strcat(acOutVdoPth, acOutVdoNm);
		oVdoWrt = cv::VideoWriter(acOutVdoPth, CV_FOURCC('M', 'P', '4', '2'), fFrmRt, oFrmSz);
	}

	for (int v = 0; v < viVdo.size(); v++)
	{
		// path to the camera folders
		char acCamFlrPth[256] = {};
		std::strcpy(acCamFlrPth, acTrk1FlrPth);
		std::strcat(acCamFlrPth, vstrCam[v].c_str());
		// input camera parameters for 2D-to-3D back projection
		char acInCamParamPth[256] = {};
		char acInCamParamNm[256] = {};
		std::strcpy(acInCamParamPth, acCamFlrPth);
		std::strcpy(acInCamParamNm, "/camCal/camParam.txt");
		std::strcat(acInCamParamPth, acInCamParamNm);
		// input 2D tracking results
		char acInTrk2dPth[256] = {};
		char acInTrk2dNm[256] = {};
		std::strcpy(acInTrk2dPth, acCamFlrPth);
		std::strcpy(acInTrk2dNm, "/trk2d.txt");
		std::strcat(acInTrk2dPth, acInTrk2dNm);
		// input folder of frame images
		char acInFrmFlrPth[256] = {};
		char acInFrmFlrNm[256] = {};
		std::strcpy(acInFrmFlrPth, acCamFlrPth);
		std::strcpy(acInFrmFlrNm, "/img1/");
		std::strcat(acInFrmFlrPth, acInFrmFlrNm);
		// output folder of each version
		char acOutTrkFlrPth[256] = {};
		char acOutTrkFlrNm[256] = {};
		std::strcpy(acOutTrkFlrPth, acCamFlrPth);
		std::strcpy(acOutTrkFlrNm, "/trk3d/");
		std::strcat(acOutTrkFlrPth, acOutTrkFlrNm);
		// output tracking results in NVIDIA AI City Challenge format
		char acOutTrkPth[256] = {};
		char acOutTrkNm[256] = {};
		std::strcpy(acOutTrkPth, acOutTrkFlrPth);
		std::strcpy(acOutTrkNm, "track1.txt");
		std::strcat(acOutTrkPth, acOutTrkNm);
		// output folder of plotted images of 3D tracking
		char acOutTrk3dImgFlrPth[256] = {};
		char acOutTrk3dImgFlrNm[256] = {};
		std::strcpy(acOutTrk3dImgFlrPth, acOutTrkFlrPth);
		std::strcpy(acOutTrk3dImgFlrNm, "trk3dimg1/");
		std::strcat(acOutTrk3dImgFlrPth, acOutTrk3dImgFlrNm);

		//_mkdir(acOutTrkFlrPth);	// in Windows
		mkdir(acOutTrkFlrPth, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

		std::vector<std::vector<CTrkNd> > vvoTrkNd;
		std::vector<std::vector<CTrkNd> > vvoTrkNdFN;

		// read projection matrix (camera parameters)
		FILE * poCamParamFl = std::fopen(acInCamParamPth, "r");
		if (poCamParamFl == NULL) { std::fputs("Error: camera parameters not loaded\n", stderr); exit(1); }
		std::ifstream ifsCamParam;
		ifsCamParam.open(acInCamParamPth);
		float afProjMat[12];
		int nLnCnt = 0;
		while (!ifsCamParam.eof())
		{
			char acBuf[256] = { 0 };
			ifsCamParam.getline(acBuf, 256);

			if (nLnCnt == 3)
				std::sscanf(acBuf, "%f %f %f %f %f %f %f %f %f %f %f %f",
					&afProjMat[0], &afProjMat[1], &afProjMat[2], &afProjMat[3], &afProjMat[4],
					&afProjMat[5], &afProjMat[6], &afProjMat[7], &afProjMat[8], &afProjMat[9],
					&afProjMat[10], &afProjMat[11]);

			nLnCnt++;
		}
		ifsCamParam.close();

		// read 2D tracking results
		char acInTrkBuf[256] = { 0 };
		int nFrmCnt, nId, nFrmCntMax = -1;
		float fDetScr = 0.0f, fDep, fSpd;
		cv::Rect oBBox;
		cv::Point2f o2dFtPt;
		cv::Point3f o3dFtPt;
		char acDetCls[32];
		CTrkNd oTrkNd;
		std::ifstream ifsInTrkTxt;
		ifsInTrkTxt.open(acInTrk2dPth);
		ifsInTrkTxt.getline(acInTrkBuf, 256);
		while (!ifsInTrkTxt.eof())
		{
			// read from the input txt file
			std::sscanf(acInTrkBuf, "%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%s", &nFrmCnt, &nId,
				&oBBox.x, &oBBox.y, &oBBox.width, &oBBox.height,
				&fDetScr, &o3dFtPt.x, &o3dFtPt.y, &o3dFtPt.z, acDetCls);

			// change from 1-based counting to 0-based counting
			//o2dFtPt = cv::Point2f((oBBox.x + (oBBox.width / 2.0f)), (oBBox.y + (oBBox.height / 2.0f)));
			o2dFtPt = cv::Point2f((oBBox.x + (oBBox.width / 2.0f)), (oBBox.y + oBBox.height));
			o3dFtPt = bkproj2d23d(o2dFtPt, afProjMat);
			fDep = cv::norm(o3dFtPt);
			oTrkNd = CTrkNd(nFrmCnt, nId, oBBox, fDetScr, o2dFtPt, o3dFtPt, acDetCls, fDep);

			while (vvoTrkNd.size() < (nId + 1))
				vvoTrkNd.push_back(std::vector<CTrkNd>());

			while (vvoTrkNdFN.size() < (nId + 1))
				vvoTrkNdFN.push_back(std::vector<CTrkNd>());

			if (fDetScrThld < fDetScr)
				vvoTrkNd[nId].push_back(oTrkNd);
			else
				vvoTrkNdFN[nId].push_back(oTrkNd);

			if (nFrmCntMax < nFrmCnt)
				nFrmCntMax = nFrmCnt;

			ifsInTrkTxt.getline(acInTrkBuf, 256);
		}
		ifsInTrkTxt.close();

		// compute speed
		int nTrajLen, iSt, iNd, nSpdWinSzUpd;
		float fDist;
		for (int i = 0; i < vvoTrkNd.size(); i++)
		{
			nTrajLen = vvoTrkNd[i].size();
			nSpdWinSzUpd = (nTrajLen < vnSpdWinSz[v]) ? nTrajLen : vnSpdWinSz[v];
			nSpdWinSzUpd = (0 == (nSpdWinSzUpd % 2)) ? (nSpdWinSzUpd - 1) : nSpdWinSzUpd;

			for (int j = 0; j < nTrajLen; j++)
			{
				fDist = 0.0f;
				iSt = j - ((nSpdWinSzUpd - 1) / 2);
				iSt = (0 <= iSt) ? iSt : 0;
				iNd = j + ((nSpdWinSzUpd - 1) / 2);
				iNd = (nTrajLen > iNd) ? iNd : (nTrajLen - 1);

				// accumulate the distance that the object travels between every two frames
				for (int k = iSt; k < iNd; k++)
					fDist += cv::norm(vvoTrkNd[i][k].get3dFtPt() - vvoTrkNd[i][k + 1].get3dFtPt());

				vvoTrkNd[i][j].setSpd(fDist * fFrmRt * vfSpdScl[v] * 2.23694f / (iNd - iSt));
			}
		}

		// compute speed for false negatives
		for (int i = 0; i < vvoTrkNdFN.size(); i++)
		{
			nTrajLen = vvoTrkNdFN[i].size();
			nSpdWinSzUpd = (nTrajLen < vnSpdWinSz[v]) ? nTrajLen : vnSpdWinSz[v];
			nSpdWinSzUpd = (0 == (nSpdWinSzUpd % 2)) ? (nSpdWinSzUpd - 1) : nSpdWinSzUpd;

			for (int j = 0; j < nTrajLen; j++)
			{
				fDist = 0.0f;
				iSt = j - ((nSpdWinSzUpd - 1) / 2);
				iSt = (0 <= iSt) ? iSt : 0;
				iNd = j + ((nSpdWinSzUpd - 1) / 2);
				iNd = (nTrajLen > iNd) ? iNd : (nTrajLen - 1);

				// accumulate the distance that the object travels between every two frames
				for (int k = iSt; k < iNd; k++)
					fDist += cv::norm(vvoTrkNdFN[i][k].get3dFtPt() - vvoTrkNdFN[i][k + 1].get3dFtPt());

				vvoTrkNdFN[i][j].setSpd(fDist * fFrmRt * vfSpdScl[v] * 2.23694f / (iNd - iSt));
			}
		}

		// fine-tune speed
		// compute the mean of speed and the mean of speed for instances that are close to the camera
		int nTrajLenFN, iTrkNdTPDistMin, nSpdConstAvgNum = 0;
		float fTrkNdTPDist, fTrkNdTPDistMin;
		double fSpdConstAvg = 0.0;
		std::vector<double> vfSpdMean, vfSpdMeanFN, vfSpdStd, vfSpdMeanCls;
		for (int i = 0; i < vvoTrkNd.size(); i++)
		{
			nTrajLen = vvoTrkNd[i].size();

			vfSpdMean.push_back(0.0);
			vfSpdMeanCls.push_back(0.0);

			if (vnSpdWinSz[v] < nTrajLen)
			{
				// sort all the objects according to their depths
				// the closer to the camera, the more accurate the calibration should be
				if (nTrajLen > 1)
					std::sort(vvoTrkNd[i].begin(), vvoTrkNd[i].end(), cmpDep);

				for (int j = 0; j < nTrajLen; j++)
				{
					vfSpdMean[i] += vvoTrkNd[i][j].getSpd();
					// the 1/3 of vehicle trajectory are considered to be close to the camera
					if ((((nTrajLen / 3) + (vnSpdWinSz[v] / 2)) > j) && ((vnSpdWinSz[v] / 2) <= j))
						vfSpdMeanCls[i] += vvoTrkNd[i][j].getSpd();
				}

				vfSpdMean[i] /= nTrajLen;
				// the 1/3 of vehicle trajectory are considered to be close to the camera
				vfSpdMeanCls[i] /= (nTrajLen / 3);
			}
		}

		for (int i = 0; i < vvoTrkNd.size(); i++)
		{
			vfSpdStd.push_back(0.0);
			nTrajLen = vvoTrkNd[i].size();

			if (vnSpdWinSz[v] < nTrajLen)
			{
				// apply constant speed strategy
				if (vfSpdLowThld[v] < vfSpdMeanCls[i])
				{
					// compute the standard deviation for each object identity
					for (int j = 0; j < nTrajLen; j++)
						vfSpdStd[i] += (vvoTrkNd[i][j].getSpd() - vfSpdMean[i]) * (vvoTrkNd[i][j].getSpd() - vfSpdMean[i]);
					vfSpdStd[i] /= nTrajLen;
					vfSpdStd[i] = std::sqrt(vfSpdStd[i]);

					// small variance of speed -> assume constant speed
					if (vfSpdStdThld[v] > vfSpdStd[i])
					{
						for (int j = 0; j < nTrajLen; j++)
						{
							vvoTrkNd[i][j].setSpd(vfSpdMeanCls[i]);
							fSpdConstAvg += vfSpdMeanCls[i];
							nSpdConstAvgNum++;
						}
					}
				}

				// small value of speed -> assume stopping
				for (int j = 0; j < nTrajLen; j++)
				{
					if (vfSpdStpThld[v] > vvoTrkNd[i][j].getSpd())
						vvoTrkNd[i][j].setSpd(0);
				}

				// find the speed for the false negatives
				nTrajLenFN = vvoTrkNdFN[i].size();
				for (int j = 0; j < nTrajLenFN; j++)
				{
					iTrkNdTPDistMin = -1;
					fTrkNdTPDistMin = FLT_MAX;

					for (int k = 0; k < nTrajLen; k++)
					{
						fTrkNdTPDist = cv::norm(vvoTrkNd[i][k].get3dFtPt() - vvoTrkNdFN[i][j].get3dFtPt());
						if (fTrkNdTPDistMin > fTrkNdTPDist)
						{
							iTrkNdTPDistMin = k;
							fTrkNdTPDistMin = fTrkNdTPDist;
						}
					}

					if ((0 <= iTrkNdTPDistMin) && (vfSpdPropFNThld[v] < vvoTrkNdFN[i][j].getSpd()))
						vvoTrkNdFN[i][j].setSpd(vvoTrkNd[i][iTrkNdTPDistMin].getSpd());
					else
						vvoTrkNdFN[i][j].setSpd(0.0f);
				}

				for (int j = 0; j < nTrajLenFN; j++)
					vvoTrkNd[i].push_back(vvoTrkNdFN[i][j]);

				if (nTrajLen > 1)
					std::sort(vvoTrkNd[i].begin(), vvoTrkNd[i].end(), cmpFrmCnt);
			}
		}

		//std::printf("video #%02d: average speed: %.3f\n", viVdo[v], (fSpdConstAvg / nSpdConstAvgNum));

		// output
		cv::Mat oImgFrm;
		int nPltTrajLen;
		char acVdo[8];
		char acId[8];
		char acSpd[16];
		char acInFrmNm[128] = { 0 };
		char acInFrmPth[128] = { 0 };
		char acOutFrmNm[128] = { 0 };
		char acOutFrmPth[128] = { 0 };
		std::vector<cv::Scalar> voBBoxClr;
		voBBoxClr.push_back(cv::Scalar(255, 0, 0)); voBBoxClr.push_back(cv::Scalar(0, 255, 0)); voBBoxClr.push_back(cv::Scalar(0, 0, 255));
		voBBoxClr.push_back(cv::Scalar(255, 255, 0)); voBBoxClr.push_back(cv::Scalar(255, 0, 255)); voBBoxClr.push_back(cv::Scalar(0, 255, 255));
		voBBoxClr.push_back(cv::Scalar(63, 127, 255)); voBBoxClr.push_back(cv::Scalar(255, 63, 127)); voBBoxClr.push_back(cv::Scalar(127, 255, 63));
		voBBoxClr.push_back(cv::Scalar(63, 255, 127)); voBBoxClr.push_back(cv::Scalar(255, 127, 63)); voBBoxClr.push_back(cv::Scalar(127, 63, 255));
		voBBoxClr.push_back(cv::Scalar(255, 255, 127)); voBBoxClr.push_back(cv::Scalar(255, 127, 255)); voBBoxClr.push_back(cv::Scalar(127, 255, 255));
		voBBoxClr.push_back(cv::Scalar(0, 255, 127)); voBBoxClr.push_back(cv::Scalar(127, 0, 255)); voBBoxClr.push_back(cv::Scalar(255, 127, 0));
		voBBoxClr.push_back(cv::Scalar(0, 127, 255)); voBBoxClr.push_back(cv::Scalar(127, 255, 0)); voBBoxClr.push_back(cv::Scalar(255, 0, 127));
		voBBoxClr.push_back(cv::Scalar(0, 0, 127)); voBBoxClr.push_back(cv::Scalar(0, 127, 0)); voBBoxClr.push_back(cv::Scalar(127, 0, 0));
		voBBoxClr.push_back(cv::Scalar(63, 127, 0)); voBBoxClr.push_back(cv::Scalar(0, 63, 127)); voBBoxClr.push_back(cv::Scalar(127, 0, 63));
		voBBoxClr.push_back(cv::Scalar(63, 0, 127)); voBBoxClr.push_back(cv::Scalar(0, 127, 63)); voBBoxClr.push_back(cv::Scalar(127, 63, 0));
		voBBoxClr.push_back(cv::Scalar(127, 0, 127)); voBBoxClr.push_back(cv::Scalar(0, 127, 127)); voBBoxClr.push_back(cv::Scalar(127, 127, 0));
		voBBoxClr.push_back(cv::Scalar(255, 127, 191)); voBBoxClr.push_back(cv::Scalar(255, 191, 127)); voBBoxClr.push_back(cv::Scalar(191, 127, 255));
		voBBoxClr.push_back(cv::Scalar(191, 255, 127)); voBBoxClr.push_back(cv::Scalar(127, 255, 191)); voBBoxClr.push_back(cv::Scalar(127, 191, 255));
		voBBoxClr.push_back(cv::Scalar(63, 63, 255)); voBBoxClr.push_back(cv::Scalar(255, 63, 63)); voBBoxClr.push_back(cv::Scalar(63, 255, 63));
		voBBoxClr.push_back(cv::Scalar(95, 159, 31)); voBBoxClr.push_back(cv::Scalar(31, 159, 95)); voBBoxClr.push_back(cv::Scalar(159, 31, 95));
		voBBoxClr.push_back(cv::Scalar(95, 31, 159)); voBBoxClr.push_back(cv::Scalar(31, 95, 159));  voBBoxClr.push_back(cv::Scalar(159, 95, 31));
		voBBoxClr.push_back(cv::Scalar(223, 31, 223)); voBBoxClr.push_back(cv::Scalar(223, 223, 31)); voBBoxClr.push_back(cv::Scalar(31, 223, 223));
		voBBoxClr.push_back(cv::Scalar(31, 63, 223)); voBBoxClr.push_back(cv::Scalar(63, 31, 223)); voBBoxClr.push_back(cv::Scalar(223, 31, 63));
		voBBoxClr.push_back(cv::Scalar(31, 223, 63)); voBBoxClr.push_back(cv::Scalar(63, 223, 31)); voBBoxClr.push_back(cv::Scalar(223, 63, 31));
		voBBoxClr.push_back(cv::Scalar(63, 159, 63)); voBBoxClr.push_back(cv::Scalar(159, 63, 63)); voBBoxClr.push_back(cv::Scalar(63, 63, 159));
		voBBoxClr.push_back(cv::Scalar(31, 0, 127)); voBBoxClr.push_back(cv::Scalar(0, 127, 31)); voBBoxClr.push_back(cv::Scalar(127, 31, 0));
		voBBoxClr.push_back(cv::Scalar(31, 127, 0)); voBBoxClr.push_back(cv::Scalar(0, 31, 127)); voBBoxClr.push_back(cv::Scalar(127, 0, 31));
		voBBoxClr.push_back(cv::Scalar(63, 191, 191)); voBBoxClr.push_back(cv::Scalar(63, 191, 191)); voBBoxClr.push_back(cv::Scalar(191, 191, 63));
		voBBoxClr.push_back(cv::Scalar(0, 63, 191)); voBBoxClr.push_back(cv::Scalar(0, 191, 63)); voBBoxClr.push_back(cv::Scalar(191, 63, 0));
		voBBoxClr.push_back(cv::Scalar(191, 0, 63, 0)); voBBoxClr.push_back(cv::Scalar(63, 0, 191)); voBBoxClr.push_back(cv::Scalar(63, 191, 0));

		// create folder for output images
		if (bOutTrk3dImgFlg)
			//_mkdir(acOutTrk3dImgFlrPth);	// in Windows
			mkdir(acOutTrk3dImgFlrPth, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

		FILE* pfOutTrkTxt = std::fopen(acOutTrkPth, "w");
		FILE* pfOutTrk3dTxt = std::fopen(acOutTrkPth, "w");

		for (int f = 0; f <= nFrmCntMax; f++)
		{
			std::printf("video #%02d: frame #%06d\n", viVdo[v], (f + 1));

			// read input frame image
			if (bOutTrk3dImgFlg || bOutVdoFlg)
			{
				std::sprintf(acInFrmNm, "%06d.jpg", f);
				std::strcpy(acInFrmPth, acInFrmFlrPth);
				std::strcat(acInFrmPth, acInFrmNm);
				oImgFrm = cv::imread(acInFrmPth, CV_LOAD_IMAGE_COLOR);
			}

			for (int i = 0; i < vvoTrkNd.size(); i++)
			{
				nTrajLen = vvoTrkNd[i].size();
				if ((f >= vvoTrkNd[i][0].getFrmCnt()) && (f <= vvoTrkNd[i][nTrajLen - 1].getFrmCnt()))
				{
					for (int j = 0; j < nTrajLen; j++)
					{
						if (f == vvoTrkNd[i][j].getFrmCnt())
						{
							// output tracking results in NVIDIA AI City Challenge format
							std::fprintf(pfOutTrkTxt, "%d %d %d %d %d %d %d %.3f %.5f\n",
								viVdo[v], (f + 1), -1, vvoTrkNd[i][j].getBBox().x, vvoTrkNd[i][j].getBBox().y,
								(vvoTrkNd[i][j].getBBox().x + vvoTrkNd[i][j].getBBox().width - 1),
								(vvoTrkNd[i][j].getBBox().y + vvoTrkNd[i][j].getBBox().height - 1),
								vvoTrkNd[i][j].getSpd(), (vvoTrkNd[i][j].getDetScr() / 100));

							// output submission results
							std::fprintf(pfOutSubmTxt, "%d %d %d %d %d %d %d %.3f %.5f\n",
								viVdo[v], (f + 1), -1, vvoTrkNd[i][j].getBBox().x, vvoTrkNd[i][j].getBBox().y,
								(vvoTrkNd[i][j].getBBox().x + vvoTrkNd[i][j].getBBox().width - 1),
								(vvoTrkNd[i][j].getBBox().y + vvoTrkNd[i][j].getBBox().height - 1),
								vvoTrkNd[i][j].getSpd(), (vvoTrkNd[i][j].getDetScr() / 100));

							if (bOutTrk3dImgFlg || bOutVdoFlg)
							{
								// plot bounding box
								cv::rectangle(oImgFrm, vvoTrkNd[i][j].getBBox(), voBBoxClr[i % voBBoxClr.size()], 2);
								// plot vehicle ID
								std::sprintf(acId, "%d", (i + 1));
								cv::putText(oImgFrm, acId, vvoTrkNd[i][j].get2dFtPt(), cv::FONT_HERSHEY_SIMPLEX, 1, voBBoxClr[i % voBBoxClr.size()], 2);
								// plot speed 
								std::sprintf(acSpd, "%.3f", vvoTrkNd[i][j].getSpd());
								cv::putText(oImgFrm, acSpd, cv::Point(vvoTrkNd[i][j].getBBox().x, (vvoTrkNd[i][j].getBBox().y - 20)),
									cv::FONT_HERSHEY_SIMPLEX, 1, voBBoxClr[i % voBBoxClr.size()], 2);
								// plot past trajectory
								nPltTrajLen = std::min(nPltTrajLenMax, (j + 1));
								for (int k = j; k > (j - nPltTrajLen + 1); k--)
									cv::line(oImgFrm, vvoTrkNd[i][k].get2dFtPt(), vvoTrkNd[i][k - 1].get2dFtPt(), voBBoxClr[i % voBBoxClr.size()], 2);
							}

							break;
						}
					}

					// plot video ID
					if (bOutVdoFlg)
						cv::putText(oImgFrm, vstrCam[v].c_str(), cv::Point(100, 100),
							cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
				}
			}

			// output plotted frames
			if (bOutTrk3dImgFlg)
			{
				std::sprintf(acOutFrmNm, "%06d.jpg", (f + 1));
				std::strcpy(acOutFrmPth, acOutTrk3dImgFlrPth);
				std::strcat(acOutFrmPth, acOutFrmNm);
				cv::imwrite(acOutFrmPth, oImgFrm);
			}

			// output video
			if (bOutVdoFlg)
				oVdoWrt.write(oImgFrm);
		}

		std::fclose(pfOutTrkTxt);
	}

	std::fclose(pfOutSubmTxt);

	cv::namedWindow("empty");
	cv::waitKey(0);

	return 0;
}
