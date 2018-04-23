#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define IM1 2147483563
#define IM2 2147483399
#define AM (1.0/IM1)
#define IMM1 (IM1-1)
#define IA1 40014
#define IA2 40692
#define IQ1 53668
#define IQ2 52774
#define IR1 12211
#define IR2 3791
#define NTAB 32
#define NDIV (1+IMM1/NTAB)
#define EPS1 1.2e-7
#define RNMX (1.0-EPS1)

//! generates expanded image
static cv::Mat genExpnImg(float fExpnRat, cv::Size oFrmSz)
{
	cv::Mat oImgExpn;

	if (1.0f >= fExpnRat)
	{
		oImgExpn = cv::Mat::zeros(cv::Size(oFrmSz.width, oFrmSz.height), CV_8UC3);
		return oImgExpn;
	}
	else
	{
		oImgExpn = cv::Mat::zeros(cv::Size((oFrmSz.width * fExpnRat), (oFrmSz.height * fExpnRat)), CV_8UC3);
		return oImgExpn;
	}
}

//! projects a point in the original image to the expanded image
static cv::Point2f projPtOrig2Expn(cv::Point2f oPtOrig, float fExpnRat, cv::Size oFrmSz)
{
	return cv::Point2f(oPtOrig.x + (oFrmSz.width * (fExpnRat - 1.0f) / 2.0f),
		oPtOrig.y + (oFrmSz.height * (fExpnRat - 1.0f) / 2.0f));
}

//! projects a point in the expanded image to the original image
static cv::Point2f projPtExpn2Orig(cv::Point2f oPtExpn, float fExpnRat, cv::Size oFrmSz)
{
	return cv::Point2f(oPtExpn.x - (oFrmSz.width * (fExpnRat - 1.0f) / 2.0f),
		oPtExpn.y - (oFrmSz.height * (fExpnRat - 1.0f) / 2.0f));
}

//! rotates a point by a given angle
static cv::Point2f rotPt(cv::Point2f oPt, float fAng)
{
	return cv::Point2f(((oPt.x * std::cos(fAng)) - (oPt.y * std::sin(fAng))),
		((oPt.x * std::sin(fAng)) + (oPt.y * std::cos(fAng))));
}

//! projects 3D point to 2D pixel location
static cv::Point2f proj3d22d(cv::Point3f o3dPt, float afProjMat[12], int nLenUnit = 1)
{
	cv::Mat oMatP(3, 4, CV_32FC1, afProjMat);

	cv::Mat oMatM3d(4, 1, CV_32FC1);
	oMatM3d.at<float>(0, 0) = o3dPt.x * nLenUnit;
	oMatM3d.at<float>(1, 0) = o3dPt.y * nLenUnit;
	oMatM3d.at<float>(2, 0) = o3dPt.z * nLenUnit;
	oMatM3d.at<float>(3, 0) = 1.0f;

	cv::Mat oMatM2d(3, 1, CV_32FC1);
	oMatM2d = oMatP * oMatM3d;

	cv::Point2f o2dPt = cv::Point2f(oMatM2d.at<float>(0, 0) / oMatM2d.at<float>(2, 0),
		oMatM2d.at<float>(1, 0) / oMatM2d.at<float>(2, 0));

	return o2dPt;
}

//! backprojects 2D point to 3D ground position
static cv::Point3f bkproj2d23d(cv::Point2f o2dPt, float afProjMat[12], int nLenUnit = 1, int nCoordSysTyp = 1)
{
	cv::Point3f o3dPt;

	cv::Mat oMatA(3, 3, CV_64F);

	if (0 == nCoordSysTyp)
	{
		oMatA.at<double>(0, 0) = afProjMat[0];
		oMatA.at<double>(0, 1) = -o2dPt.x;
		oMatA.at<double>(0, 2) = afProjMat[2];
		oMatA.at<double>(1, 0) = afProjMat[4];
		oMatA.at<double>(1, 1) = -o2dPt.y;
		oMatA.at<double>(1, 2) = afProjMat[6];
		oMatA.at<double>(2, 0) = afProjMat[8];
		oMatA.at<double>(2, 1) = -1.0;
		oMatA.at<double>(2, 2) = afProjMat[10];
	}
	else if (1 == nCoordSysTyp)
	{
		oMatA.at<double>(0, 0) = afProjMat[0];
		oMatA.at<double>(0, 1) = afProjMat[1];
		oMatA.at<double>(0, 2) = -o2dPt.x;
		oMatA.at<double>(1, 0) = afProjMat[4];
		oMatA.at<double>(1, 1) = afProjMat[5];
		oMatA.at<double>(1, 2) = -o2dPt.y;
		oMatA.at<double>(2, 0) = afProjMat[8];
		oMatA.at<double>(2, 1) = afProjMat[9];
		oMatA.at<double>(2, 2) = -1.0;
	}

	cv::Mat oMatAInv(3, 3, CV_64F);
	cv::invert(oMatA, oMatAInv, cv::DECOMP_SVD);

	cv::Mat oMatB(3, 1, CV_64F);
	oMatB.at<double>(0, 0) = -afProjMat[3];
	oMatB.at<double>(1, 0) = -afProjMat[7];
	oMatB.at<double>(2, 0) = -afProjMat[11];

	cv::Mat oMatM(3, 1, CV_64F);
	oMatM = oMatAInv * oMatB;

	if (0 == nCoordSysTyp)
		o3dPt = cv::Point3f(oMatM.at<double>(0, 0), 0.0f, oMatM.at<double>(2, 0)) / nLenUnit;
	else if(1 == nCoordSysTyp)
		o3dPt = cv::Point3f(oMatM.at<double>(0, 0), oMatM.at<double>(1, 0), 0.0f) / nLenUnit;

	return o3dPt;
}

//! generates a random double variable
static double rand2(long *idum)
{
	int j;
	long k;
	static long idum2 = 123456789;
	static long iy = 0;
	static long iv[NTAB];
	double temp;
	if (*idum <= 0) {
		if (-(*idum) < 1) *idum = 1;
		else *idum = -(*idum);
		idum2 = (*idum);
		for (j = NTAB + 7; j >= 0; j--) {
			k = (*idum) / IQ1;
			*idum = IA1*(*idum - k*IQ1) - k*IR1;
			if (*idum < 0) *idum += IM1;
			if (j < NTAB) iv[j] = *idum;
		}
		iy = iv[0];
	}
	k = (*idum) / IQ1;
	*idum = IA1*(*idum - k*IQ1) - k*IR1;
	if (*idum < 0) *idum += IM1;
	k = idum2 / IQ2;
	idum2 = IA2*(idum2 - k*IQ2) - k*IR2;
	if (idum2 < 0) idum2 += IM2;
	j = iy / NDIV;
	iy = iv[j] - idum2;
	iv[j] = *idum;
	if (iy < 1) iy += IMM1;
	if ((temp = AM*iy) > RNMX) return RNMX;
	else return temp;
}

//! generates a random number
static double get_rand_num(double max, double min, long seed)
{
	double rand = rand2(&seed);
	double duration = max - min;
	return min + rand*duration;
}