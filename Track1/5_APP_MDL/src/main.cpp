//-----------------------------------------------------------------------------------
//
//  Copyright (c) 2018 Zheng Tang <zhtang@uw.edu>.  All rights reserved.
//
//  Description:
//      Implementation of adaptive appearance feature extraction and comparison
//
//-----------------------------------------------------------------------------------

#include <fstream>
#include <math.h>
//#include <direct.h>	// in Windows
#include <sys/stat.h>	// in Linux
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

//! define the breadth in pixels to determine frame/ROI border case (default: 10)
#define FRM_BRDR_BDTH (10)
//! define the normalized size for the appearance model (default: cv::Size(128, 128))
#define APP_MDL_NORM_SZ (cv::Size(128, 128))
//! define the threshold of visibility for the appearance model (default: 0.3)
#define APP_MDL_VIS_THLD (0.3)
//! define the LBP operator for the appearance model: 0: Extended LBP; 1: Fixed Sampling LBP; 2: Variance-based LBP (default: 1)
#define APP_MDL_LBP_OP (1)
//! define the number of samples in the appearance modeL (default: 30)
#define APP_MDL_SMP_NUM (30)

class CTrkNd
{
public:
	CTrkNd(void);
	CTrkNd(int nFrmCnt, int nId, cv::Rect oBBox, float fDetScr, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, char* acDetCls, float fDep, bool bFrmBrdrFlg);
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
	inline bool getFrmBrdrFlg(void) { return m_bFrmBrdrFlg; }
	inline void setFrmBrdrFlg(bool bFrmBrdrFlg) { m_bFrmBrdrFlg = bFrmBrdrFlg; }

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
	//! whether the object is attached to a frame border
	bool m_bFrmBrdrFlg;
};

CTrkNd::CTrkNd(void)
{
	setFrmCnt(-1);
}

CTrkNd::CTrkNd(int nFrmCnt, int nId, cv::Rect oBBox, float fDetScr, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, char* acDetCls, float fDep, bool bFrmBrdrFlg)
{
	setFrmCnt(nFrmCnt);
	setId(nId);
	setBBox(oBBox);
	setDetScr(fDetScr);
	set2dFtPt(o2dFtPt);
	set3dFtPt(o3dFtPt);
	setDetCls(acDetCls);
	setDep(fDep);
	setFrmBrdrFlg(bFrmBrdrFlg);
}

CTrkNd::~CTrkNd(void)
{

}

bool cmpDep(CTrkNd oTrkNd0, CTrkNd oTrkNd1)	// used to sort tracking nodes in a list
{
	return oTrkNd0.getDep() > oTrkNd1.getDep();
}

cv::Mat genDepMap(std::vector<CTrkNd> voTrkNd, cv::Size oFrmSz)
{
	cv::Mat oDepMap;
	oDepMap = cv::Mat::zeros(oFrmSz, cv::DataType<int>::type);
	int iDepBtm = INT_MAX;

	for (std::vector<CTrkNd>::iterator it = voTrkNd.begin(); it != voTrkNd.end(); ++it)
	{
		int iDep = it - voTrkNd.begin() + 1;
		cv::Rect oBBox = it->getBBox();

		if ((oFrmSz.height - FRM_BRDR_BDTH - 1) <= (oBBox.y + oBBox.height))
			iDepBtm = iDep;

		for (int x = it->getBBox().x; x < (it->getBBox().x + it->getBBox().width); x++)
		{
			for (int y = it->getBBox().y; y < (it->getBBox().y + it->getBBox().height); y++)
			{
				if (iDep > oDepMap.at<int>(cv::Point(x, y)))
				{
					if (iDep > iDepBtm)
						oDepMap.at<int>(cv::Point(x, y)) = iDepBtm;
					else
						oDepMap.at<int>(cv::Point(x, y)) = iDep;
				}
			}
		}
	}

	return oDepMap;
}

// local binary pattern
namespace lbp
{
	template <typename _Tp>
	void OLBP_(const cv::Mat& oImgSrc, cv::Mat& oImgDst)
	{
		oImgDst = cv::Mat::zeros(oImgSrc.rows - 2, oImgSrc.cols - 2, CV_8UC1);
		for (int i = 1; i < oImgSrc.rows - 1; i++)
		{
			for (int j = 1; j < oImgSrc.cols - 1; j++)
			{
				_Tp tCent = oImgSrc.at<_Tp>(i, j);
				unsigned char ucCd = 0;
				ucCd |= (oImgSrc.at<_Tp>(i - 1, j - 1) > tCent) << 7;
				ucCd |= (oImgSrc.at<_Tp>(i - 1, j) > tCent) << 6;
				ucCd |= (oImgSrc.at<_Tp>(i - 1, j + 1) > tCent) << 5;
				ucCd |= (oImgSrc.at<_Tp>(i, j + 1) > tCent) << 4;
				ucCd |= (oImgSrc.at<_Tp>(i + 1, j + 1) > tCent) << 3;
				ucCd |= (oImgSrc.at<_Tp>(i + 1, j) > tCent) << 2;
				ucCd |= (oImgSrc.at<_Tp>(i + 1, j - 1) > tCent) << 1;
				ucCd |= (oImgSrc.at<_Tp>(i, j - 1) > tCent) << 0;
				oImgDst.at<unsigned char>(i - 1, j - 1) = ucCd;
			}
		}
	}

	template <typename _Tp>
	void ELBP_(const cv::Mat& oImgSrc, cv::Mat& oImgDst, int nRad = 1, int nNbrNum = 8)
	{
		nNbrNum = std::max(std::min(nNbrNum, 31), 1); // set bounds
		oImgDst = cv::Mat::zeros(oImgSrc.rows - 2 * nRad, oImgSrc.cols - 2 * nRad, CV_32SC1);
		for (int n = 0; n < nNbrNum; n++)
		{
			// sample points
			float x = static_cast<float>(nRad) * cos(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
			float y = static_cast<float>(nRad) * -sin(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
			// relative indices
			int fx = static_cast<int>(floor(x));
			int fy = static_cast<int>(floor(y));
			int cx = static_cast<int>(ceil(x));
			int cy = static_cast<int>(ceil(y));
			// fractional part
			float ty = y - fy;
			float tx = x - fx;
			// set interpolation weights
			float w1 = (1 - tx) * (1 - ty);
			float w2 = tx * (1 - ty);
			float w3 = (1 - tx) * ty;
			float w4 = tx * ty;
			// iterate through your data
			for (int i = nRad; i < oImgSrc.rows - nRad; i++)
			{
				for (int j = nRad; j < oImgSrc.cols - nRad; j++)
				{
					float t = w1 * oImgSrc.at<_Tp>(i + fy, j + fx) + w2 * oImgSrc.at<_Tp>(i + fy, j + cx) +
						w3 * oImgSrc.at<_Tp>(i + cy, j + fx) + w4 * oImgSrc.at<_Tp>(i + cy, j + cx);
					// we are dealing with floating point precision, so add some little tolerance
					oImgDst.at<unsigned char>(i - nRad, j - nRad) += ((t > oImgSrc.at<_Tp>(i, j)) && (abs(t - oImgSrc.at<_Tp>(i, j)) >
						std::numeric_limits<float>::epsilon())) << n;
				}
			}
		}
	}

	template <typename _Tp>
	void VARLBP_(const cv::Mat& oImgSrc, cv::Mat& oImgDst, int nRad = 1, int nNbrNum = 8)
	{
		std::max(std::min(nNbrNum, 31), 1); // set bounds
		oImgDst = cv::Mat::zeros(oImgSrc.rows - 2 * nRad, oImgSrc.cols - 2 * nRad, CV_32FC1);
		cv::Mat _mean = cv::Mat::zeros(oImgSrc.rows, oImgSrc.cols, CV_32FC1);
		cv::Mat _delta = cv::Mat::zeros(oImgSrc.rows, oImgSrc.cols, CV_32FC1);
		cv::Mat _m2 = cv::Mat::zeros(oImgSrc.rows, oImgSrc.cols, CV_32FC1);
		for (int n = 0; n<nNbrNum; n++)
		{
			// sample points
			float x = static_cast<float>(nRad) * cos(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
			float y = static_cast<float>(nRad) * -sin(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
			// relative indices
			int fx = static_cast<int>(floor(x));
			int fy = static_cast<int>(floor(y));
			int cx = static_cast<int>(ceil(x));
			int cy = static_cast<int>(ceil(y));
			// fractional part
			float ty = y - fy;
			float tx = x - fx;
			// set interpolation weights
			float w1 = (1 - tx) * (1 - ty);
			float w2 = tx * (1 - ty);
			float w3 = (1 - tx) *      ty;
			float w4 = tx * ty;
			// iterate through your data
			for (int i = nRad; i < oImgSrc.rows - nRad; i++)
			{
				for (int j = nRad; j < oImgSrc.cols - nRad; j++)
				{
					float t = w1 * oImgSrc.at<_Tp>(i + fy, j + fx) + w2 * oImgSrc.at<_Tp>(i + fy, j + cx) +
						w3 * oImgSrc.at<_Tp>(i + cy, j + fx) + w4 * oImgSrc.at<_Tp>(i + cy, j + cx);
					_delta.at<float>(i, j) = t - _mean.at<float>(i, j);
					_mean.at<float>(i, j) = (_mean.at<float>(i, j) + (_delta.at<float>(i, j) / (1.0*(n + 1))));
					_m2.at<float>(i, j) = _m2.at<float>(i, j) + _delta.at<float>(i, j) * (t - _mean.at<float>(i, j));
				}
			}
		}
		// calculate result
		for (int i = nRad; i < oImgSrc.rows - nRad; i++)
		{
			for (int j = nRad; j < oImgSrc.cols - nRad; j++)
			{
				oImgDst.at<float>(i - nRad, j - nRad) = _m2.at<float>(i, j) / (1.0*(nNbrNum - 1));
			}
		}
	}
}

// generate LBP image: 0: Extended LBP; 1: Fixed Sampling LBP; 2: Variance-based LBP
cv::Mat genLbp(const cv::Mat& oImgSrc, int nLbpOp = 1, int nRad = 1, int nNbrNum = 8)
{
	cv::Mat oImgDst; // image after preprocessing
	cv::Mat oImgLbp; // LBP image to return

	cv::cvtColor(oImgSrc, oImgDst, CV_BGR2GRAY);
	cv::GaussianBlur(oImgDst, oImgDst, cv::Size(7, 7), 5, 3, cv::BORDER_CONSTANT); // tiny bit of smoothing

	switch (nLbpOp)
	{
	case 0:
		switch (oImgDst.type()) // use the extended operator
		{
		case CV_8SC1: lbp::ELBP_<char>(oImgDst, oImgLbp); break;
		case CV_8UC1: lbp::ELBP_<unsigned char>(oImgDst, oImgLbp); break;
		case CV_16SC1: lbp::ELBP_<short>(oImgDst, oImgLbp); break;
		case CV_16UC1: lbp::ELBP_<unsigned short>(oImgDst, oImgLbp); break;
		case CV_32SC1: lbp::ELBP_<int>(oImgDst, oImgLbp); break;
		case CV_32FC1: lbp::ELBP_<float>(oImgDst, oImgLbp); break;
		case CV_64FC1: lbp::ELBP_<double>(oImgDst, oImgLbp); break;
		}
		break;
	case 1:
		switch (oImgDst.type()) // use the original operator
		{
		case CV_8SC1: lbp::OLBP_<char>(oImgDst, oImgLbp); break;
		case CV_8UC1: lbp::OLBP_<unsigned char>(oImgDst, oImgLbp); break;
		case CV_16SC1: lbp::OLBP_<short>(oImgDst, oImgLbp); break;
		case CV_16UC1: lbp::OLBP_<unsigned short>(oImgDst, oImgLbp); break;
		case CV_32SC1: lbp::OLBP_<int>(oImgDst, oImgLbp); break;
		case CV_32FC1: lbp::OLBP_<float>(oImgDst, oImgLbp); break;
		case CV_64FC1: lbp::OLBP_<double>(oImgDst, oImgLbp); break;
		}
		break;
	case 2:
		switch (oImgDst.type())
		{
		case CV_8SC1: lbp::VARLBP_<char>(oImgDst, oImgLbp); break;
		case CV_8UC1: lbp::VARLBP_<unsigned char>(oImgDst, oImgLbp); break;
		case CV_16SC1: lbp::VARLBP_<short>(oImgDst, oImgLbp); break;
		case CV_16UC1: lbp::VARLBP_<unsigned short>(oImgDst, oImgLbp); break;
		case CV_32SC1: lbp::VARLBP_<int>(oImgDst, oImgLbp); break;
		case CV_32FC1: lbp::VARLBP_<float>(oImgDst, oImgLbp); break;
		case CV_64FC1: lbp::VARLBP_<double>(oImgDst, oImgLbp); break;
		}
		break;
	}

	//// now to return the patterns a normalization is necessary
	//cv::normalize(oImgLbp, oImgLbp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	return oImgLbp;
}

void calcHist(cv::Mat oImg, cv::Mat oMsk, std::vector<float>& vfAppMdlHist, int nHistSz)
{
    int nImgVal, nMskVal, nHistBin = 256 / nHistSz;
    double fVecMag = 0.0;

    for (int b = 0; b < nHistSz; b++)
        vfAppMdlHist[b] = 0.0f;

    for (int x = 0; x < oImg.cols; x++)
    {
        for (int y = 0; y < oImg.rows; y++)
        {
            nImgVal = oImg.at<uchar>(y, x);
            nMskVal = oMsk.at<uchar>(y, x);
            vfAppMdlHist[nImgVal / nHistBin] += (float)nMskVal / 255.0f;
        }
    }

    // normalize the histogram by L2 norm
    for (int b = 0; b < nHistSz; b++)
        fVecMag += vfAppMdlHist[b] * vfAppMdlHist[b];
    fVecMag = std::sqrt(fVecMag);
    for (int b = 0; b < nHistSz; b++)
        vfAppMdlHist[b] /= fVecMag;
}

// compare adaptive appearance features of a probe and a gallery
// return the distance between the probe and the gallery (between 0 and 1)
// nHistCmpTyp: -1: EMD; 0: Correlation; 1: Chi-Square; 2: Intersection; 3: Bhattacharyya distance
float compAppMdl(char *acInFeatFlrPthPrb, int nIdPrb, char *acInFeatFlrPthGall, int nIdGall, double fDistThld, int nHistCmpTyp, int nEmdDistTyp = CV_DIST_L1)
{
    int nUnmtchCnt = 0;
    double fDist;
    char acInFeatNm[128] = {};
	char acInFeatPth[128] = {};
	char acInFeatBuf[2048] = {};
	std::ifstream ifsFeat;
	cv::Mat oAppMdlHistPrb(88, 1, CV_32FC1);
	cv::Mat oAppMdlHistGall(88, 1, CV_32FC1);
    cv::Mat oAppMdlSigPrb(88, 2, CV_32FC1);
	cv::Mat oAppMdlSigGall(88, 2, CV_32FC1);
	std::vector<float> vfAppMdlSmp(88);
	std::vector<std::vector<float> > vvfAppMdlPrb;
	std::vector<std::vector<float> > vvfAppMdlGall;

	// read model of the probe
	std::sprintf(acInFeatNm, "%06d.txt", nIdPrb);
	std::strcpy(acInFeatPth, acInFeatFlrPthPrb);
	std::strcat(acInFeatPth, acInFeatNm);
	ifsFeat.open(acInFeatPth);
	ifsFeat.getline(acInFeatBuf, 2048);
	while (!ifsFeat.eof())
	{
		std::sscanf(acInFeatBuf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            &vfAppMdlSmp[0], &vfAppMdlSmp[1], &vfAppMdlSmp[2], &vfAppMdlSmp[3],
            &vfAppMdlSmp[4], &vfAppMdlSmp[5], &vfAppMdlSmp[6], &vfAppMdlSmp[7],
            &vfAppMdlSmp[8], &vfAppMdlSmp[9], &vfAppMdlSmp[10], &vfAppMdlSmp[11],
            &vfAppMdlSmp[12], &vfAppMdlSmp[13], &vfAppMdlSmp[14], &vfAppMdlSmp[15],
            &vfAppMdlSmp[16], &vfAppMdlSmp[17], &vfAppMdlSmp[18], &vfAppMdlSmp[19],
            &vfAppMdlSmp[20], &vfAppMdlSmp[21], &vfAppMdlSmp[22], &vfAppMdlSmp[23],
            &vfAppMdlSmp[24], &vfAppMdlSmp[25], &vfAppMdlSmp[26], &vfAppMdlSmp[27],
            &vfAppMdlSmp[28], &vfAppMdlSmp[29], &vfAppMdlSmp[30], &vfAppMdlSmp[31],
            &vfAppMdlSmp[32], &vfAppMdlSmp[33], &vfAppMdlSmp[34], &vfAppMdlSmp[35],
            &vfAppMdlSmp[36], &vfAppMdlSmp[37], &vfAppMdlSmp[38], &vfAppMdlSmp[39],
            &vfAppMdlSmp[40], &vfAppMdlSmp[41], &vfAppMdlSmp[42], &vfAppMdlSmp[43],
            &vfAppMdlSmp[44], &vfAppMdlSmp[45], &vfAppMdlSmp[46], &vfAppMdlSmp[47],
            &vfAppMdlSmp[48], &vfAppMdlSmp[49], &vfAppMdlSmp[50], &vfAppMdlSmp[51],
            &vfAppMdlSmp[52], &vfAppMdlSmp[53], &vfAppMdlSmp[54], &vfAppMdlSmp[55],
            &vfAppMdlSmp[56], &vfAppMdlSmp[57], &vfAppMdlSmp[58], &vfAppMdlSmp[59],
            &vfAppMdlSmp[60], &vfAppMdlSmp[61], &vfAppMdlSmp[62], &vfAppMdlSmp[63],
            &vfAppMdlSmp[64], &vfAppMdlSmp[65], &vfAppMdlSmp[66], &vfAppMdlSmp[67],
            &vfAppMdlSmp[68], &vfAppMdlSmp[69], &vfAppMdlSmp[70], &vfAppMdlSmp[71],
            &vfAppMdlSmp[72], &vfAppMdlSmp[73], &vfAppMdlSmp[74], &vfAppMdlSmp[75],
            &vfAppMdlSmp[76], &vfAppMdlSmp[77], &vfAppMdlSmp[78], &vfAppMdlSmp[79],
            &vfAppMdlSmp[70], &vfAppMdlSmp[81], &vfAppMdlSmp[82], &vfAppMdlSmp[83],
            &vfAppMdlSmp[84], &vfAppMdlSmp[85], &vfAppMdlSmp[86], &vfAppMdlSmp[87]);
		vvfAppMdlPrb.push_back(vfAppMdlSmp);
		ifsFeat.getline(acInFeatBuf, 2048);
	}
	ifsFeat.close();

	// read model of the gallery
	std::sprintf(acInFeatNm, "%06d.txt", nIdGall);
	std::strcpy(acInFeatPth, acInFeatFlrPthGall);
	std::strcat(acInFeatPth, acInFeatNm);
	ifsFeat.open(acInFeatPth);
	ifsFeat.getline(acInFeatBuf, 2048);
	while (!ifsFeat.eof())
	{
		std::sscanf(acInFeatBuf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            &vfAppMdlSmp[0], &vfAppMdlSmp[1], &vfAppMdlSmp[2], &vfAppMdlSmp[3],
            &vfAppMdlSmp[4], &vfAppMdlSmp[5], &vfAppMdlSmp[6], &vfAppMdlSmp[7],
            &vfAppMdlSmp[8], &vfAppMdlSmp[9], &vfAppMdlSmp[10], &vfAppMdlSmp[11],
            &vfAppMdlSmp[12], &vfAppMdlSmp[13], &vfAppMdlSmp[14], &vfAppMdlSmp[15],
            &vfAppMdlSmp[16], &vfAppMdlSmp[17], &vfAppMdlSmp[18], &vfAppMdlSmp[19],
            &vfAppMdlSmp[20], &vfAppMdlSmp[21], &vfAppMdlSmp[22], &vfAppMdlSmp[23],
            &vfAppMdlSmp[24], &vfAppMdlSmp[25], &vfAppMdlSmp[26], &vfAppMdlSmp[27],
            &vfAppMdlSmp[28], &vfAppMdlSmp[29], &vfAppMdlSmp[30], &vfAppMdlSmp[31],
            &vfAppMdlSmp[32], &vfAppMdlSmp[33], &vfAppMdlSmp[34], &vfAppMdlSmp[35],
            &vfAppMdlSmp[36], &vfAppMdlSmp[37], &vfAppMdlSmp[38], &vfAppMdlSmp[39],
            &vfAppMdlSmp[40], &vfAppMdlSmp[41], &vfAppMdlSmp[42], &vfAppMdlSmp[43],
            &vfAppMdlSmp[44], &vfAppMdlSmp[45], &vfAppMdlSmp[46], &vfAppMdlSmp[47],
            &vfAppMdlSmp[48], &vfAppMdlSmp[49], &vfAppMdlSmp[50], &vfAppMdlSmp[51],
            &vfAppMdlSmp[52], &vfAppMdlSmp[53], &vfAppMdlSmp[54], &vfAppMdlSmp[55],
            &vfAppMdlSmp[56], &vfAppMdlSmp[57], &vfAppMdlSmp[58], &vfAppMdlSmp[59],
            &vfAppMdlSmp[60], &vfAppMdlSmp[61], &vfAppMdlSmp[62], &vfAppMdlSmp[63],
            &vfAppMdlSmp[64], &vfAppMdlSmp[65], &vfAppMdlSmp[66], &vfAppMdlSmp[67],
            &vfAppMdlSmp[68], &vfAppMdlSmp[69], &vfAppMdlSmp[70], &vfAppMdlSmp[71],
            &vfAppMdlSmp[72], &vfAppMdlSmp[73], &vfAppMdlSmp[74], &vfAppMdlSmp[75],
            &vfAppMdlSmp[76], &vfAppMdlSmp[77], &vfAppMdlSmp[78], &vfAppMdlSmp[79],
            &vfAppMdlSmp[70], &vfAppMdlSmp[81], &vfAppMdlSmp[82], &vfAppMdlSmp[83],
            &vfAppMdlSmp[84], &vfAppMdlSmp[85], &vfAppMdlSmp[86], &vfAppMdlSmp[87]);
		vvfAppMdlGall.push_back(vfAppMdlSmp);
		ifsFeat.getline(acInFeatBuf, 2048);
	}
	ifsFeat.close();

	for (int i = 0; i < vvfAppMdlPrb.size(); i++)
	{
		for (int j = 0; j < vvfAppMdlGall.size(); j++)
		{
            for (int b = 0; b < 88; b++)
            {
                oAppMdlHistPrb.at<float>(b, 0) = vvfAppMdlPrb[i][b];
                oAppMdlHistGall.at<float>(b, 0) = vvfAppMdlGall[j][b];
            }

            // compute distance between probe histograms and gallery histograms
            if (0 <= nHistCmpTyp)
                fDist = cv::compareHist(oAppMdlHistPrb, oAppMdlHistGall, nHistCmpTyp);
            else
            {
                // create signatures as required by EMD
                for (int b = 0; b < 88; b++)
                {
                    oAppMdlSigPrb.at<float>(b, 0) = oAppMdlHistPrb.at<float>(b, 0);
                    oAppMdlSigPrb.at<float>(b, 1) = b;
                    oAppMdlSigGall.at<float>(b, 0) = oAppMdlHistGall.at<float>(b, 0);
                    oAppMdlSigGall.at<float>(b, 1) = b;
                }

                fDist = cv::EMD(oAppMdlSigPrb, oAppMdlSigGall, nEmdDistTyp);
            }

            if (fDistThld < fDist)
                nUnmtchCnt++;
		}
	}

    return ((float)nUnmtchCnt / (float)(vvfAppMdlPrb.size() * vvfAppMdlGall.size()));
}

// inTrkRes inFrmFlr outFeatFlr fWgtBGR fWgtHSV fWgtLab fWgtLbp fWgtGrad
int main(int argc, char *argv[])
{
	char acInTrkTxt[256] = {};
	std::strcpy(acInTrkTxt, argv[1]);
	char acInFrmFlrPth[256] = {};
	std::strcpy(acInFrmFlrPth, argv[2]);
	char acOutFlrPth[256] = {};
	std::strcpy(acOutFlrPth, argv[3]);
	float fWgtBGR = atof(argv[4]), fWgtHSV = atof(argv[5]), fWgtLab = atof(argv[6]), fWgtLbp = atof(argv[7]), fWgtGrad = atof(argv[8]);

	char acInTrkBuf[256] = {};
	char acInFrmNm[128] = {};
	char acInFrmPth[128] = {};
	char acOutFeatNm[128] = {};
	char acOutFeatPth[128] = {};
	char acOutFeatBuf[2048] = {};
	int nFrmCntCurr = 0, nFrmCnt, nId, nMskArea, nMskInitArea, nSmpCnt, iDep, iXAppMdl, iYAppMdl;
	float fDetScr, fDep;
	double fDistSq, fRoofRiSq = (double)((APP_MDL_NORM_SZ.width * APP_MDL_NORM_SZ.width) + (APP_MDL_NORM_SZ.height * APP_MDL_NORM_SZ.height)) / 4.0;
	double fProb;
	bool bFrmBrdrFlg, bNxtFrmFlg;
	FILE * poFeatFl;
	std::ifstream ifsFeat;
	cv::Mat oImgFrm, oDepMap, oAppMdlFrm, oAppMdlMsk, oAppMdlMskInit, oAppMdlHsv, oAppMdlLab, oAppMdlLbp, oAppMdlGrad, oAppMdlGradMag, oAppMdlGradAng;
	cv::Size oFrmSz;
	cv::Rect oBBox;
	cv::Rect2f oBBoxf;
	cv::Point2f o2dFtPt;
	cv::Point3f o3dFtPt;
    cv::Vec2f ovGradMagSmp;
    cv::Vec2b ovGradAngSmp;
    cv::HOGDescriptor oHogDesc;
    int anChGrad[3];
	char acDetCls[32];
	CTrkNd oTrkNd;
	std::vector<float> vfIdMinDep;
	std::vector<float> vfAppMdlSmp(88);
	std::vector<float> vfAppMdlClrBHist(8);
	std::vector<float> vfAppMdlClrGHist(8);
	std::vector<float> vfAppMdlClrRHist(8);
	std::vector<float> vfAppMdlClrHHist(8);
	std::vector<float> vfAppMdlClrSHist(8);
	std::vector<float> vfAppMdlClrVHist(8);
	std::vector<float> vfAppMdlClrLHist(8);
	std::vector<float> vfAppMdlClraHist(8);
	std::vector<float> vfAppMdlClrbHist(8);
	std::vector<float> vfAppMdlLbpHist(8);
	std::vector<float> vfAppMdlGradHist(8);
	std::vector<std::vector<float> > vvfAppMdl;
	std::vector<cv::Mat> voClrPln;
	std::vector<CTrkNd> voTrkNd;

	// create output folder
	//_mkdir(acOutFlrPth);	// in Windows
	mkdir(acOutFlrPth, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

	// create input text results of tracking
	std::ifstream ifsInTrkTxt;
	ifsInTrkTxt.open(acInTrkTxt);

	// initialize foreground mask
	oAppMdlMskInit = cv::Mat::zeros(APP_MDL_NORM_SZ, CV_8UC1);
    for (int x = 0; x < APP_MDL_NORM_SZ.width; x++)
    {
        for (int y = 0; y < APP_MDL_NORM_SZ.height; y++)
        {
            double fDistSq = ((x - (APP_MDL_NORM_SZ.width / 2)) * (x - (APP_MDL_NORM_SZ.width / 2))) +
                ((y - (APP_MDL_NORM_SZ.height / 2)) * (y - (APP_MDL_NORM_SZ.height / 2)));
            oAppMdlMskInit.at<uchar>(y, x) = 255 * std::exp((-2.0) * fDistSq / fRoofRiSq);
        }
    }

	cv::ellipse(oAppMdlMskInit, cv::RotatedRect(cv::Point(0, 0), cv::Point((APP_MDL_NORM_SZ.width - 1), 0), cv::Point((APP_MDL_NORM_SZ.width - 1), (APP_MDL_NORM_SZ.height - 1))), cv::Scalar(255), -1);
	nMskInitArea = cv::countNonZero(oAppMdlMskInit);

	while (true)
	{
		std::printf("frame %06d\n", nFrmCntCurr);

		// read input frame image
		std::sprintf(acInFrmNm, "%06d.jpg", nFrmCntCurr);
		std::strcpy(acInFrmPth, acInFrmFlrPth);
		std::strcat(acInFrmPth, acInFrmNm);
		oImgFrm = cv::imread(acInFrmPth, CV_LOAD_IMAGE_COLOR);
		oFrmSz = oImgFrm.size();
		if (oImgFrm.empty())
			break;

		// read tracking results
		bNxtFrmFlg = true;
		std::vector<CTrkNd>().swap(voTrkNd);
		while ((!ifsInTrkTxt.eof()) && ((nFrmCntCurr == oTrkNd.getFrmCnt()) || (-1 == oTrkNd.getFrmCnt())))
		{
			// at the first frame with objects
			if (-1 == oTrkNd.getFrmCnt())
				bNxtFrmFlg = false;

			// push back the extra node read from the last iteration
			// not at the first frame with objects
			if (bNxtFrmFlg && (oTrkNd.getFrmCnt() == nFrmCnt))
			{
				voTrkNd.push_back(oTrkNd);
				bNxtFrmFlg = false;
			}

			// read from the input txt file
			ifsInTrkTxt.getline(acInTrkBuf, 256);
			std::sscanf(acInTrkBuf, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%s", &nFrmCnt, &nId,
				&oBBoxf.x, &oBBoxf.y, &oBBoxf.width, &oBBoxf.height,
				&fDetScr, &o3dFtPt.x, &o3dFtPt.y, &o3dFtPt.z, acDetCls);

			if (nFrmCntCurr <= nFrmCnt)
			{
				oBBox = oBBoxf;
				//o2dFtPt = cv::Point2f((oBBox.x + (oBBox.width / 2.0f)), (oBBox.y + (oBBox.height / 2.0f)));
				o2dFtPt = cv::Point2f((oBBox.x + (oBBox.width / 2.0f)), (oBBox.y + oBBox.height));
				//fDep = cv::norm(o3dFtPt);
				fDep = 1.0f / o2dFtPt.y;

				// at the frame border
				if ((FRM_BRDR_BDTH > oBBox.x) || (FRM_BRDR_BDTH > oBBox.y) ||
					((oFrmSz.width - FRM_BRDR_BDTH - 1) <= (oBBox.x + oBBox.width)) ||
					((oFrmSz.height - FRM_BRDR_BDTH - 1) <= (oBBox.y + oBBox.height)))
					bFrmBrdrFlg = true;
				else
					bFrmBrdrFlg = false;

				oTrkNd = CTrkNd(nFrmCnt, nId, oBBox, fDetScr, o2dFtPt, o3dFtPt, acDetCls, fDep, bFrmBrdrFlg);

				// only objects in the current frame are pushed back
				if (nFrmCntCurr == nFrmCnt)
					voTrkNd.push_back(oTrkNd);
			}
		}

		if (voTrkNd.size())
		{
			// create depth map
			std::sort(voTrkNd.begin(), voTrkNd.end(), cmpDep);
			oDepMap = genDepMap(voTrkNd, oFrmSz);

			for (std::vector<CTrkNd>::iterator it = voTrkNd.begin(); it != voTrkNd.end(); ++it)
			{
				// not attached to any frame border
				if (!it->getFrmBrdrFlg())
				{
					nId = it->getId();
					oBBox = it->getBBox();
					iDep = it - voTrkNd.begin() + 1;

					// apply the depth map to remove occluded area
					oAppMdlMsk = oAppMdlMskInit.clone();
					for (int x = oBBox.x; x < (oBBox.x + oBBox.width - 1); x++)
					{
						for (int y = oBBox.y; y < (oBBox.y + oBBox.height - 1); y++)
						{
							if (iDep < oDepMap.at<int>(y, x))
							{
								iXAppMdl = (float)APP_MDL_NORM_SZ.width * (float)(x - oBBox.x) / (float)oBBox.width;
								iYAppMdl = (float)APP_MDL_NORM_SZ.height * (float)(y - oBBox.y) / (float)oBBox.height;
								oAppMdlMsk.at<uchar>(iYAppMdl, iXAppMdl) = 0;
							}
						}
					}

					// the visibility is larger than a threshold for the construction of appearance model
					nMskArea = cv::countNonZero(oAppMdlMsk);
					if (APP_MDL_VIS_THLD < ((double)nMskArea / (double)nMskInitArea))
					{
						// read existing model
						std::vector<std::vector<float> >().swap(vvfAppMdl);
						std::sprintf(acOutFeatNm, "%06d.txt", nId);
						std::strcpy(acOutFeatPth, acOutFlrPth);
						std::strcat(acOutFeatPth, acOutFeatNm);

						poFeatFl = std::fopen(acOutFeatPth, "r");
						if (poFeatFl == NULL)
						{
							poFeatFl = std::fopen(acOutFeatPth, "w");
							std::fclose(poFeatFl);
						}
						else
							std::fclose(poFeatFl);

						nSmpCnt = 0;
						ifsFeat.open(acOutFeatPth);
						ifsFeat.getline(acOutFeatBuf, 2048);
						while (!ifsFeat.eof())
						{
							std::sscanf(acOutFeatBuf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
								&vfAppMdlSmp[0], &vfAppMdlSmp[1], &vfAppMdlSmp[2], &vfAppMdlSmp[3],
								&vfAppMdlSmp[4], &vfAppMdlSmp[5], &vfAppMdlSmp[6], &vfAppMdlSmp[7],
								&vfAppMdlSmp[8], &vfAppMdlSmp[9], &vfAppMdlSmp[10], &vfAppMdlSmp[11],
								&vfAppMdlSmp[12], &vfAppMdlSmp[13], &vfAppMdlSmp[14], &vfAppMdlSmp[15],
								&vfAppMdlSmp[16], &vfAppMdlSmp[17], &vfAppMdlSmp[18], &vfAppMdlSmp[19],
								&vfAppMdlSmp[20], &vfAppMdlSmp[21], &vfAppMdlSmp[22], &vfAppMdlSmp[23],
								&vfAppMdlSmp[24], &vfAppMdlSmp[25], &vfAppMdlSmp[26], &vfAppMdlSmp[27],
								&vfAppMdlSmp[28], &vfAppMdlSmp[29], &vfAppMdlSmp[30], &vfAppMdlSmp[31],
								&vfAppMdlSmp[32], &vfAppMdlSmp[33], &vfAppMdlSmp[34], &vfAppMdlSmp[35],
								&vfAppMdlSmp[36], &vfAppMdlSmp[37], &vfAppMdlSmp[38], &vfAppMdlSmp[39],
								&vfAppMdlSmp[40], &vfAppMdlSmp[41], &vfAppMdlSmp[42], &vfAppMdlSmp[43],
								&vfAppMdlSmp[44], &vfAppMdlSmp[45], &vfAppMdlSmp[46], &vfAppMdlSmp[47],
								&vfAppMdlSmp[48], &vfAppMdlSmp[49], &vfAppMdlSmp[50], &vfAppMdlSmp[51],
								&vfAppMdlSmp[52], &vfAppMdlSmp[53], &vfAppMdlSmp[54], &vfAppMdlSmp[55],
								&vfAppMdlSmp[56], &vfAppMdlSmp[57], &vfAppMdlSmp[58], &vfAppMdlSmp[59],
								&vfAppMdlSmp[60], &vfAppMdlSmp[61], &vfAppMdlSmp[62], &vfAppMdlSmp[63],
								&vfAppMdlSmp[64], &vfAppMdlSmp[65], &vfAppMdlSmp[66], &vfAppMdlSmp[67],
								&vfAppMdlSmp[68], &vfAppMdlSmp[69], &vfAppMdlSmp[70], &vfAppMdlSmp[71],
								&vfAppMdlSmp[72], &vfAppMdlSmp[73], &vfAppMdlSmp[74], &vfAppMdlSmp[75],
								&vfAppMdlSmp[76], &vfAppMdlSmp[77], &vfAppMdlSmp[78], &vfAppMdlSmp[79],
								&vfAppMdlSmp[70], &vfAppMdlSmp[81], &vfAppMdlSmp[82], &vfAppMdlSmp[83],
								&vfAppMdlSmp[84], &vfAppMdlSmp[85], &vfAppMdlSmp[86], &vfAppMdlSmp[87]);
							vvfAppMdl.push_back(vfAppMdlSmp);
							ifsFeat.getline(acOutFeatBuf, 2048);
							nSmpCnt++;
						}
						ifsFeat.close();

						// find the minimum depth for each object ID
						while (vfIdMinDep.size() <= nId)
							vfIdMinDep.push_back(it->getDep());
						if (vfIdMinDep[nId] > it->getDep())
							vfIdMinDep[nId] = it->getDep();

						// generate a random float number between 0.0 and 1.0
						if (APP_MDL_SMP_NUM <= nSmpCnt)
							fProb = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

						// for distant objects, there is small probability for model update
						if ((APP_MDL_SMP_NUM > nSmpCnt) || ((APP_MDL_SMP_NUM <= nSmpCnt) && (fProb <= (vfIdMinDep[nId] / it->getDep()))))
						{
							// extract and normalize object image
							oAppMdlFrm = oImgFrm(oBBox);
							cv::resize(oAppMdlFrm, oAppMdlFrm, APP_MDL_NORM_SZ);

							// calculate RGB color histograms
							cv::split(oAppMdlFrm, voClrPln);
							calcHist(voClrPln[0], oAppMdlMsk, vfAppMdlClrBHist, 8);
							calcHist(voClrPln[1], oAppMdlMsk, vfAppMdlClrGHist, 8);
							calcHist(voClrPln[2], oAppMdlMsk, vfAppMdlClrRHist, 8);

                            // calculate HSV color histograms
                            cv::cvtColor(oAppMdlFrm, oAppMdlHsv, CV_BGR2HSV);
							cv::split(oAppMdlHsv, voClrPln);
							calcHist(voClrPln[0], oAppMdlMsk, vfAppMdlClrHHist, 8);
							calcHist(voClrPln[1], oAppMdlMsk, vfAppMdlClrSHist, 8);
							calcHist(voClrPln[2], oAppMdlMsk, vfAppMdlClrVHist, 8);

                            // calculate Lab color histograms
                            cv::cvtColor(oAppMdlFrm, oAppMdlLab, CV_BGR2Lab);
							cv::split(oAppMdlLab, voClrPln);
							calcHist(voClrPln[0], oAppMdlMsk, vfAppMdlClrLHist, 8);
							calcHist(voClrPln[1], oAppMdlMsk, vfAppMdlClraHist, 8);
							calcHist(voClrPln[2], oAppMdlMsk, vfAppMdlClrbHist, 8);

							// calculate LBP histogram
							oAppMdlLbp = genLbp(oAppMdlFrm, APP_MDL_LBP_OP);
							calcHist(oAppMdlLbp, oAppMdlMsk(cv::Rect(1, 1, (APP_MDL_NORM_SZ.width - 2), (APP_MDL_NORM_SZ.height - 2))), vfAppMdlLbpHist, 8);

                            // calculate gradient histogram
                            oHogDesc.computeGradient(oAppMdlFrm, oAppMdlGradMag, oAppMdlGradAng);
                            oAppMdlGrad = cv::Mat(APP_MDL_NORM_SZ, CV_8UC3);
                            for (int x = 0; x < APP_MDL_NORM_SZ.width; x++)
                            {
                                for (int y = 0; y < APP_MDL_NORM_SZ.height; y++)
                                {
                                    ovGradMagSmp = oAppMdlGradMag.at<cv::Vec2f>(y, x);
                                    anChGrad[0] = (0 < (ovGradMagSmp[0] * 128)) ? ((ovGradMagSmp[0] * 128) - 1) : 0;
                                    anChGrad[0] += (0 < (ovGradMagSmp[1] * 128)) ? ((ovGradMagSmp[1] * 128) - 1) : 0;
                                    anChGrad[0] /= 2;
                                    ovGradAngSmp = oAppMdlGradAng.at<cv::Vec2b>(y, x);
                                    anChGrad[1] = (0 < ovGradAngSmp[0]) ? ((ovGradAngSmp[0] * 32) - 1) : 0;
                                    anChGrad[2] = (0 < ovGradAngSmp[1]) ? ((ovGradAngSmp[1] * 32) - 1) : 0;
                                    oAppMdlGrad.at<cv::Vec3b>(y, x) = cv::Vec3b(anChGrad[0], anChGrad[1], anChGrad[2]);
                                }
                            }
                            cv::cvtColor(oAppMdlGrad, oAppMdlGrad, cv::COLOR_Lab2BGR);
                            cv::cvtColor(oAppMdlGrad, oAppMdlGrad, cv::COLOR_BGR2GRAY);
                            calcHist(oAppMdlGrad, oAppMdlMsk, vfAppMdlGradHist, 8);

							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i] = vfAppMdlClrBHist[i] * fWgtBGR;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+8] = vfAppMdlClrGHist[i] * fWgtBGR;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+16] = vfAppMdlClrRHist[i] * fWgtBGR;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+24] = vfAppMdlClrHHist[i] * fWgtHSV;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+32] = vfAppMdlClrSHist[i] * fWgtHSV;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+40] = vfAppMdlClrVHist[i] * fWgtHSV;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+48] = vfAppMdlClrLHist[i] * fWgtLab;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+56] = vfAppMdlClraHist[i] * fWgtLab;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+64] = vfAppMdlClrbHist[i] * fWgtLab;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+72] = vfAppMdlLbpHist[i] * fWgtLbp;
							for (int i = 0; i < 8; i++)
								vfAppMdlSmp[i+80] = vfAppMdlGradHist[i] * fWgtGrad;

							if (APP_MDL_SMP_NUM > nSmpCnt)
								vvfAppMdl.push_back(vfAppMdlSmp);
							else
								vvfAppMdl[rand() % APP_MDL_SMP_NUM] = vfAppMdlSmp;

							poFeatFl = std::fopen(acOutFeatPth, "w");
							for (int i = 0; i < vvfAppMdl.size(); i++)
							{
								std::fprintf(poFeatFl, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
									vvfAppMdl[i][0], vvfAppMdl[i][1], vvfAppMdl[i][2], vvfAppMdl[i][3],
									vvfAppMdl[i][4], vvfAppMdl[i][5], vvfAppMdl[i][6], vvfAppMdl[i][7],
									vvfAppMdl[i][8], vvfAppMdl[i][9], vvfAppMdl[i][10], vvfAppMdl[i][11],
									vvfAppMdl[i][12], vvfAppMdl[i][13], vvfAppMdl[i][14], vvfAppMdl[i][15],
									vvfAppMdl[i][16], vvfAppMdl[i][17], vvfAppMdl[i][18], vvfAppMdl[i][19],
									vvfAppMdl[i][20], vvfAppMdl[i][21], vvfAppMdl[i][22], vvfAppMdl[i][23],
									vvfAppMdl[i][24], vvfAppMdl[i][25], vvfAppMdl[i][26], vvfAppMdl[i][27],
									vvfAppMdl[i][28], vvfAppMdl[i][29], vvfAppMdl[i][30], vvfAppMdl[i][31],
									vvfAppMdl[i][32], vvfAppMdl[i][33], vvfAppMdl[i][34], vvfAppMdl[i][35],
									vvfAppMdl[i][36], vvfAppMdl[i][37], vvfAppMdl[i][38], vvfAppMdl[i][39],
									vvfAppMdl[i][40], vvfAppMdl[i][41], vvfAppMdl[i][42], vvfAppMdl[i][43],
									vvfAppMdl[i][44], vvfAppMdl[i][45], vvfAppMdl[i][46], vvfAppMdl[i][47],
									vvfAppMdl[i][48], vvfAppMdl[i][49], vvfAppMdl[i][50], vvfAppMdl[i][51],
									vvfAppMdl[i][52], vvfAppMdl[i][53], vvfAppMdl[i][54], vvfAppMdl[i][55],
									vvfAppMdl[i][56], vvfAppMdl[i][57], vvfAppMdl[i][58], vvfAppMdl[i][59],
									vvfAppMdl[i][60], vvfAppMdl[i][61], vvfAppMdl[i][62], vvfAppMdl[i][63],
									vvfAppMdl[i][64], vvfAppMdl[i][65], vvfAppMdl[i][66], vvfAppMdl[i][67],
									vvfAppMdl[i][68], vvfAppMdl[i][69], vvfAppMdl[i][70], vvfAppMdl[i][71],
									vvfAppMdl[i][72], vvfAppMdl[i][73], vvfAppMdl[i][74], vvfAppMdl[i][75],
									vvfAppMdl[i][76], vvfAppMdl[i][77], vvfAppMdl[i][78], vvfAppMdl[i][79],
									vvfAppMdl[i][80], vvfAppMdl[i][81], vvfAppMdl[i][82], vvfAppMdl[i][83],
									vvfAppMdl[i][84], vvfAppMdl[i][85], vvfAppMdl[i][86], vvfAppMdl[i][87]);
							}
							std::fclose(poFeatFl);
						}
					}
				}
			}
		}

		nFrmCntCurr++;
	}

	ifsInTrkTxt.close();

	return 0;
}
