#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>

class CCfg
{
public:
	//! full constructor
	CCfg();
	//! default destructor
	~CCfg();

	//! loads configuration file from directory
	void ldCfgFl(void);

	inline cv::Size getFrmSz(void) { return m_oFrmSz; }
	void setFrmSz(cv::Size oFrmSz) { m_oFrmSz = oFrmSz; };

	inline char* getInFrmPth(void) { return m_acInFrmPth; }
	inline char* getOutCamParamPth(void) { return m_acOutCamParamPth; }
	inline int getRszFrmHei(void) { return m_nRszFrmHei; }
	inline int getLenUnit(void) { return m_nLenUnit; }
	inline bool getCalSelVanLnFlg(void) { return m_bCalSelVanLnFlg; }
	inline cv::Point getCalVr(void) { return m_oCalVr; }
	inline cv::Point getCalVl(void) { return m_oCalVl; }
	inline float getCalCamHeiMax(void) { return m_fCalCamHeiMax; }
	inline float getCalCamHeiMin(void) { return m_fCalCamHeiMin; }
	inline int getCalGrdSzR(void) { return m_nCalGrdSzR; }
	inline int getCalGrdSzL(void) { return m_nCalGrdSzL; }
	inline bool getCalEdaOptFlg(void) { return m_bCalEdaOptFlg; }
	inline std::vector<cv::Point> getCalMeasLnSegNdPt(void) { return m_voCalMeasLnSegNdPt; }
	inline std::vector<float> getCalMeasLnSegDist(void) { return m_vfCalMeasLnSegDist; }

private:
	//! reads char array
	std::string rdCharArr(std::string strCfg, int nParamPos);
	//! reads integer number
	int rdInt(std::string strCfg, int nParamPos);
	//! reads float number
	float rdFlt(std::string strCfg, int nParamPos);
	//! reads bool value
	bool rdBool(std::string strCfg, int nParamPos);
	//! reads 2D point
	cv::Point rd2dPt(std::string strCfg, int nParamPos);
	//! reads vector of float numbers
	std::vector<float> rdVecFlt(std::string strCfg, int nParamPos);
	//! reads vector of 2D points
	std::vector<cv::Point> rdVec2dPt(std::string strCfg, int nParamPos);

	//! video frame size
	cv::Size m_oFrmSz;
	//! path of input frame image
	char m_acInFrmPth[256];
	//! path of output text file of camera parameters
	char m_acOutCamParamPth[256];
	//! resized video frame height (-1: original size)
	int m_nRszFrmHei;
	//! the length unit, 10 or 1000 (1 cm = 10 mm, 1 m = 1000 mm)
	int m_nLenUnit;
	//! flag of selecting vanishing lines on the ground plane, necessary when m_nInCalTyp == 1
	bool m_bCalSelVanLnFlg;
	//! given vanishing point Vr, necessary when m_bCalSelVanLnFlg == false
	cv::Point m_oCalVr;
	//! given vanishing point Vl, necessary when m_bCalSelVanLnFlg == false
	cv::Point m_oCalVl;
	//! the maximum height of camera in m_nLenUnit, necessary when m_nInCalTyp > 1
	float m_fCalCamHeiMax;
	//! the minimum height of camera in m_nLenUnit, necessary when m_nInCalTyp > 1
	float m_fCalCamHeiMin;
	//! size of the 3D grid (in m_nLenUnit) on ground plane along R axis, necessary when m_nInCalTyp > 0
	int m_nCalGrdSzR;
	//! size of the 3D grid (in m_nLenUnit) on ground plane along L axis, necessary when m_nInCalTyp > 0
	int m_nCalGrdSzL;
	//! flag of EDA optimization for camera calibration, necessary when m_nInCalTyp > 0
	bool m_bCalEdaOptFlg;
	//! pair(s) of end points of measuring line segments, necessary when m_nCalReprojErrTyp == 1 and total number must be even
	std::vector<cv::Point> m_voCalMeasLnSegNdPt;
	//! ground truth distance of measuring line segments , necessary when m_nCalReprojErrTyp == 1 and total number must be half of m_voCalMeasLnSegNdPt
	std::vector<float> m_vfCalMeasLnSegDist;
};
