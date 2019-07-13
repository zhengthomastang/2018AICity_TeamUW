#pragma once

#include <fstream>
#include "utils.h"
#include "Cfg.h"

//! define the type of coordinate system for camera calibration: 0: X-Z ground plane; 1: X-Y ground plane (default: 0)
#define COORD_SYS_TYP (1)
//! define the type of Vy estimation: 0: computing center of mass; 1: RANSAC (default: 1)
#define VY_EST_TYP (1)
//! define the type of Linf estimation: 0: linear regression; 1: RANSAC (default: 1)
#define LINF_EST_TYP (1)
//! define the type of principal point estimation: 0: assuming as the image center; 1: the point with minimum distance to the perpendicular line to Linf (default: 0)
#define PRIN_PT_EST_TYP (0)
//! define the number of iterations in RANSAC for Vy estimation, necessary when  VY_EST_TYP = 1 (default: 100)
#define VY_RS_ITER_NUM (100)
//! define the threshold for the distance (divided by the frame width) of RANSAC inliers to Vy, necessary when  VY_EST_TYP = 1 (default: 2.0)
#define VY_RS_DIST_THLD (2.0)
//! define the number of iterations in RANSAC for Linf estimation, necessary when  LINF_EST_TYP = 1 (default: 100)
#define LINF_RS_ITER_NUM (100)
//! define the threshold for the distance (divided by the frame height) of RANSAC inliers to Linf, necessary when  LINF_EST_TYP = 1 (default: 0.15)
#define LINF_RS_DIST_THLD (0.20)
//! define the threshold for the distance (divided by the frame height) of RANSAC inliers to Linf, necessary when  LINF_EST_TYP = 1 (default: 0.15)
#define LINF_RS_DIST_THLD (0.20)
//! define the range for focal length in ratio in EDA optimization (default: 0.2f)
#define EDA_RNG_F (0.2f)
//! define the range for principal point coordinates in pixels in EDA optimization (default: 100)
#define EDA_RNG_PRIN_PT (100)
//! define the range for rotation angles in degrees in EDA optimization (default: 45.0)
#define EDA_RNG_ROT_ANG (45.0)
//! define the initial population of EDA (default: 20000)
#define EDA_INIT_POP (20000)
//! define the selected population of EDA (default: 20)
#define EDA_SEL_POP (20)
//! define the number of iterations of EDA (default: 100)
#define EDA_ITER_NUM (100)
//! define the threshold of ratio of reprojection errors between iterations (default: 0.10)
#define EDA_REPROJ_ERR_THLD (0.01)
//! define image expansion ratio for plotting vanishing points and horizon line (default: 2.0f)
#define IMG_EXPN_RAT (2.0f)

template <typename T> T deg2rad(T deg) { return deg * (CV_PI / 180.0); }
template <typename T> T rad2deg(T rad) { return rad * (180.0 / CV_PI); }

// camera parameters and their matrices
class CCamParam
{
public:
	struct SParamRng
	{
		// X-Z ground plane
		// s m' = K [R|t] M' (P = K [R|t])
		//   [u]   [fx 0  cx] [r11 r12 r13] [1 0 0 t1] [X]
		// s [v] = [0  fy cy] [r21 r22 r23] [0 1 0 t2] [Y]
		//   [1]   [0  0  1 ] [r31 r32 r33] [0 0 1 t3] [Z]
		//                                             [1]
		// skew = 0
		// X&Z on the ground, Y straight downwards
		// (0) originally CCS parallel with WCS
		// (1) translate upwards by t
		// (2) rotate yaw(pan) degrees around Y axis
		// (3) rotate pitch(tilt) degrees around X axis
		// (4) rotate roll degrees around Z axis
		// R = RZ * RX * RY
		//      [cos(roll) -sin(roll) 0          ]
		// RZ = [sin(roll) cos(roll)  0          ]
		//      [0         0          1          ]
		//      [1         0          0          ]
		// RX = [0         cos(pitch) -sin(pitch)]
		//      [0         sin(pitch) cos(pitch) ]
		//      [cos(yaw)  0          sin(yaw)   ]
		// RY = [0         1          0          ]
		//      [-sin(yaw) 0          cos(yaw)   ]
		// r11 = cos(roll)cos(yaw) - sin(roll)sin(pitch)sin(yaw)
		// r12 = -sin(roll)cos(pitch)
		// r13 = cos(roll)sin(yaw) + sin(roll)sin(pitch)cos(yaw)
		// r21 = sin(roll)cos(yaw) + cos(roll)sin(pitch)sin(yaw)
		// r22 = cos(roll)cos(pitch)
		// r23 = sin(roll)sin(yaw) - cos(roll)sin(pitch)cos(yaw)
		// r31 = -cos(pitch)sin(yaw)
		// r32 = sin(pitch)
		// r33 = cos(pitch)cos(yaw)
		// t1 = tx(0)
		// t2 = ty(-Hc)
		// t3 = tz(0)

		SParamRng()
		{
			fFxMax = 5000, fFxMin = 0;
			fFyMax = 5000, fFyMin = 0;
			fCxMax = 5000, fCxMin = 0;
			fCyMax = 5000, fCyMin = 0;
			fRollMax = deg2rad(90), fRollMin = deg2rad(-90);
			fPitchMax = deg2rad(90), fPitchMin = deg2rad(-90);
			fYawMax = deg2rad(90), fYawMin = deg2rad(-90);
			fTxMax = 10, fTxMin = -10;
			fTyMax = 10, fTyMin = -10;
			fTzMax = 10, fTzMin = -10;
		}

		float fFxMax, fFxMin;	// camera focal length
		float fFyMax, fFyMin;	// camera focal length fy = fx * a (aspect ratio, close to 1)
		float fCxMax, fCxMin;	// optical center/principal point
		float fCyMax, fCyMin;	// optical center/principal point
		float fRollMax, fRollMin;	// roll angle
		float fPitchMax, fPitchMin;	// pitch(tilt) angle
		float fYawMax, fYawMin;	// yaw(pan) angle
		float fTxMax, fTxMin;
		float fTyMax, fTyMin;
		float fTzMax, fTzMin;
	};

	CCamParam(void);
	~CCamParam(void);

	inline float* getInParamMat() { return m_afK; }
	inline float* getRotMat() { return m_afR; }
	inline float* getTntMat() { return m_afT; }
	inline float* getProjMat() { return m_afP; }
	inline float getFx() { return m_fFx; }
	inline void setFx(float fFx) { m_fFx = fFx; }
	inline float getFy() { return m_fFy; }
	inline void setFy(float fFy) { m_fFy = fFy; }
	inline float getCx() { return m_fCx; }
	inline void setCx(float fCx) { m_fCx = fCx; }
	inline float getCy() { return m_fCy; }
	inline void setCy(float fCy) { m_fCy = fCy; }
	inline float getRoll() { return m_fRoll; }
	inline void setRoll(float fRoll) { m_fRoll = fRoll; }
	inline float getPitch() { return m_fPitch; }
	inline void setPitch(float fPitch) { m_fPitch = fPitch; }
	inline float getYaw() { return m_fYaw; }
	inline void setYaw(float fYaw) { m_fYaw = fYaw; }
	inline float getTx() { return m_fTx; }
	inline void setTx(float fTx) { m_fTx = fTx; }
	inline float getTy() { return m_fTy; }
	inline void setTy(float fTy) { m_fTy = fTy; }
	inline float getTz() { return m_fTz; }
	inline void setTz(float fTz) { m_fTz = fTz; }
	inline float getReprojErr() { return m_fReprojErr; }
	inline void setReprojErr(float fReprojErr) { m_fReprojErr = fReprojErr; }

	//! sets the matrix of intrinsic parameters
	void setInParamMat(float fFx, float fFy, float fCx, float fCy);
	//! sets the matrix of rotation
	void setRotMat(float fRoll, float fPitch, float fYaw);
	//! sets the matrix of translation
	void setTntMat(float fTx, float fTy, float fTz);
	//! calculates the projective matrix
	void calcProjMat();

	//! reads the file of camera intrinsic parameters
	void rdCamInParamTxt(const char* acCamInParamPth);
	//! initializes camera model
	void initCamMdl(SParamRng sParamRng);

private:
	float m_afK[9];
	float m_afR[9];
	float m_afT[3];
	float m_afP[12];

	float m_fFx;
	float m_fFy;
	float m_fCx;
	float m_fCy;
	float m_fRoll;
	float m_fPitch;
	float m_fYaw;
	float m_fTx;
	float m_fTy;
	float m_fTz;

	float m_fReprojErr;
};

// camera calibrator
class CCamCal
{
public:
	CCamCal(void);
	~CCamCal(void);

	//! initializes the self-calibrator
	void initialize(CCfg oCfg, cv::Mat oImgBg);
	//! perform self-calibration
	bool process(std::vector<cv::Point> voVanPt);

	inline cv::Point2f getVyCand(int i) { return m_voVyCand[i]; }
	inline cv::Point2f getLinfCand(int i) { return m_voLinfCand[i]; }
	inline cv::Point2f getPrinPt(void) { return m_oPrinPt; }
	inline cv::Point2f getVr(void) { return m_oVr; }
	inline cv::Point2f getVl(void) { return m_oVl; }

private:
	//! estimates vertical vanishing point Vy by computing center of mass
	void estVyByCM(void);
	//! estimates vertical vanishing point Vy by RANSAC algorithm
	void estVyByRS(void);
	//! estimates horizon line Linf by linear regression
	void estLinfByLR(void);
	//! estimates horizon line Linf by RANSAC algorithm
	void estLinfByRS(void);
	//! estimates principal point by assuming it as the image center
	void estPrinPtByAC(void);
	//! estimates principal point by minimum distance
	void estPrinPtByMD(void);
	//! estimates vanishing points Vr and Vl on the horizon line
	void estVrVl(void);
	//! calibrates camera by direct computation
	void calCamDctComp(void);
	//! calibrates camera by EDA optimization
	void calCamEdaOpt(void);
	//! computes camera parameters
	void compCamParam(cv::Point2f oVr, cv::Point2f oVl, cv::Point2f oPrinPt, float fCamHei, CCamParam* poCamParam);
	//! calculates reprojection error based on measurement error compared to ground truth
	double calcReprojErr(CCamParam* poCamParam);
	//! calculates starting grid point
	cv::Point calcStGrdPt(CCamParam* poCamParam);
	//! tests starting grid point (all corner points of the grip are within the expanded frame image)
	bool tstStGrdPt(cv::Point oStGrdPt, CCamParam* poCamParam);
	//! initializes the ranges of camera parameters in EDA optimization
	CCamParam::SParamRng initEdaParamRng(cv::Point2f oVr, cv::Point2f oVl, cv::Point2f oPrinPt, CCamParam* poCamParam = NULL);
	//! estimates the ranges of camera parameters in EDA optimization
	CCamParam::SParamRng estEdaParamRng(std::vector<CCamParam>* pvoCamParam);
	//! plots a 3D grid on the ground plane
	void plt3dGrd(CCamParam* poCamParam, cv::Point2f oVr, cv::Point2f oVl, cv::Point oStGrdPt);

	//! configuration parameters
	CCfg m_oCfg;
	//! background image for plotting results
	cv::Mat m_oImgBg;
	//! 25% progress flag for collecting 2D tracking data
	bool m_bCol25ProgFlg;
	//! 50% progress flag for collecting 2D tracking data
	bool m_bCol50ProgFlg;
	//! 75% progress flag for collecting 2D tracking data
	bool m_bCol75ProgFlg;
	//! candidate points for vertical vanishing point (Vy) estimation
	std::vector<cv::Point2f> m_voVyCand;
	//! candidate points for horizon line (Linf) estimation
	std::vector<cv::Point2f> m_voLinfCand;
	//! estimated vertical vanishing point
	cv::Point2f m_oVy;
	//! the slope of estimated horizon line
	float m_fLinfSlp;
	//! the intercept of estimated horizon line with y axis
	float m_fLinfItcp;
	//! the sample correlation coefficient of estimated horizon line
	float m_fLinfCorr;
	//! estimated vanishing point Vr
	cv::Point2f m_oVr;
	//! estimated vanishing point Vl
	cv::Point2f m_oVl;
	//! (initial) principal point (optical center) of the camera
	cv::Point2f m_oPrinPt;
};

// selector of vanishing lines
class CVanLnSel
{
public:
	CVanLnSel(void);
	~CVanLnSel(void);

	//! initializes the vanishing lines selector
	void initialize(CCfg oCfg, cv::Mat oImgBg);
	//! calibrates the camera manually
	std::vector<cv::Point> process(void);
	//! pushes a node to the list and draw the line & circle
	void addNd(int nX, int nY);
	//! returns the flag of vanishing points selection
	inline bool getSelVanLnFlg(void) { return m_bSelVanLnFlg; }
	//! checks if the background image is loaded
	inline bool chkImgLd(void)
	{
		if (!m_oImgBg.empty())
			return true;
		else
			return false;
	}

private:
	//! computes to vanishing points
	std::vector<cv::Point> compVanPts(void);

	//! configuration parameters
	CCfg m_oCfg;
	//! background image for plotting results
	cv::Mat m_oImgBg;
	//! flag of completing the selection of vanishing lines
	bool m_bSelVanLnFlg;
	//! list of nodes
	std::vector<cv::Point> m_voNd;
};

extern CVanLnSel oVanLnSel;
