#include "CamCal.h"

CVanLnSel oVanLnSel;

void on_mouse3(int event, int x, int y, int flags, void*)  // mouse event
{
	if (oVanLnSel.getSelVanLnFlg())
		return;

	if (!oVanLnSel.chkImgLd())
	{
		std::cout << "Error: on_mouse3(): background image is unloaded" << std::endl;
		return;
	}

	if (event == CV_EVENT_FLAG_LBUTTON)
		oVanLnSel.addNd(x, y);

	return;
}

bool compReprojErr(CCamParam oCamParam1, CCamParam oCamParam2)
{
	return (oCamParam1.getReprojErr() < oCamParam2.getReprojErr());
}

CCamParam::CCamParam(void)
{

}

CCamParam::~CCamParam(void)
{

}

void CCamParam::setInParamMat(float fFx, float fFy, float fCx, float fCy)
{
	m_afK[0] = fFx;
	m_afK[1] = 0.0f;
	m_afK[2] = fCx;
	m_afK[3] = 0.0f;
	m_afK[4] = fFy;
	m_afK[5] = fCy;
	m_afK[6] = 0.0f;
	m_afK[7] = 0.0f;
	m_afK[8] = 1.0f;
}

void CCamParam::setRotMat(float fRoll, float fPitch, float fYaw)
{
	if (0 == COORD_SYS_TYP)
	{
		m_afR[0] = (std::cos(fRoll) * std::cos(fYaw)) - (std::sin(fRoll) * std::sin(fPitch) * std::sin(fYaw));
		m_afR[1] = -std::sin(fRoll) * std::cos(fPitch);
		m_afR[2] = (std::cos(fRoll) * std::sin(fYaw)) + (std::sin(fRoll) * std::sin(fPitch) * std::cos(fYaw));
		m_afR[3] = (std::sin(fRoll) * std::cos(fYaw)) + (std::cos(fRoll) * std::sin(fPitch) * std::sin(fYaw));
		m_afR[4] = std::cos(fRoll) * std::cos(fPitch);
		m_afR[5] = (std::sin(fRoll) * std::sin(fYaw)) - (std::cos(fRoll) * std::sin(fPitch) * std::cos(fYaw));
		m_afR[6] = -std::cos(fPitch) * std::sin(fYaw);
		m_afR[7] = std::sin(fPitch);
		m_afR[8] = std::cos(fPitch) * std::cos(fYaw);
	}
	else if (1 == COORD_SYS_TYP)
	{
		m_afR[0] = (-std::cos(fRoll) * std::sin(fYaw)) - (std::sin(fRoll) * std::sin(fPitch) * std::cos(fYaw));
		m_afR[1] = (-std::cos(fRoll) * std::cos(fYaw)) - (std::sin(fRoll) * std::sin(fPitch) * std::cos(fYaw));
		m_afR[2] = std::sin(fRoll) * std::cos(fPitch);
		m_afR[3] = (-std::sin(fRoll) * std::sin(fYaw)) + (std::cos(fRoll) * std::sin(fPitch) * std::cos(fYaw));
		m_afR[4] = (-std::sin(fRoll) * std::cos(fYaw)) - (std::cos(fRoll) * std::sin(fPitch) * std::sin(fYaw));
		m_afR[5] = -std::cos(fRoll) * std::cos(fPitch);
		m_afR[6] = std::cos(fPitch) * std::cos(fYaw);
		m_afR[7] = -std::cos(fPitch) * std::sin(fYaw);
		m_afR[8] = std::sin(fPitch);
	}
}

void CCamParam::setTntMat(float fTx, float fTy, float fTz)
{
	m_afT[0] = fTx;
	m_afT[1] = fTy;
	m_afT[2] = fTz;
}

void CCamParam::calcProjMat()
{
	cv::Mat oMatK(3, 3, CV_32FC1, m_afK);
	cv::Mat oMatR(3, 3, CV_32FC1, m_afR);

	// T = -Rt
	cv::Mat oMatT(3, 1, CV_32FC1);
	oMatT.ptr<float>(0)[0] = -m_afT[0];
	oMatT.ptr<float>(1)[0] = -m_afT[1];
	oMatT.ptr<float>(2)[0] = -m_afT[2];
	oMatT = oMatR * oMatT;

	// P = K [R|T]
	cv::Mat oMatP(3, 4, CV_32FC1);
	oMatP.ptr<float>(0)[0] = m_afR[0];
	oMatP.ptr<float>(0)[1] = m_afR[1];
	oMatP.ptr<float>(0)[2] = m_afR[2];
	oMatP.ptr<float>(1)[0] = m_afR[3];
	oMatP.ptr<float>(1)[1] = m_afR[4];
	oMatP.ptr<float>(1)[2] = m_afR[5];
	oMatP.ptr<float>(2)[0] = m_afR[6];
	oMatP.ptr<float>(2)[1] = m_afR[7];
	oMatP.ptr<float>(2)[2] = m_afR[8];
	oMatP.ptr<float>(0)[3] = oMatT.ptr<float>(0)[0];
	oMatP.ptr<float>(1)[3] = oMatT.ptr<float>(1)[0];
	oMatP.ptr<float>(2)[3] = oMatT.ptr<float>(2)[0];
	oMatP = oMatK * oMatP;

	m_afP[0] = oMatP.ptr<float>(0)[0];
	m_afP[1] = oMatP.ptr<float>(0)[1];
	m_afP[2] = oMatP.ptr<float>(0)[2];
	m_afP[3] = oMatP.ptr<float>(0)[3];
	m_afP[4] = oMatP.ptr<float>(1)[0];
	m_afP[5] = oMatP.ptr<float>(1)[1];
	m_afP[6] = oMatP.ptr<float>(1)[2];
	m_afP[7] = oMatP.ptr<float>(1)[3];
	m_afP[8] = oMatP.ptr<float>(2)[0];
	m_afP[9] = oMatP.ptr<float>(2)[1];
	m_afP[10] = oMatP.ptr<float>(2)[2];
	m_afP[11] = oMatP.ptr<float>(2)[3];
}

void CCamParam::rdCamInParamTxt(const char* acCamInParamPth)
{
	std::ifstream ifsCamInParam;
	ifsCamInParam.open(acCamInParamPth);

	char acBuf[256] = { 0 };
	ifsCamInParam.getline(acBuf, 256);
	std::sscanf(acBuf, "%f %f %f %f %f %f %f %f %f\n",
		&m_afK[0], &m_afK[1], &m_afK[2], &m_afK[3], &m_afK[4], &m_afK[5], &m_afK[6], &m_afK[7], &m_afK[8]);

	ifsCamInParam.close();
}

void CCamParam::initCamMdl(SParamRng sParamRng)
{
	m_fFx = (float)get_rand_num(sParamRng.fFxMin, sParamRng.fFxMax, rand());
	m_fFy = (float)get_rand_num(sParamRng.fFyMin, sParamRng.fFyMax, rand());
	m_fCx = (float)get_rand_num(sParamRng.fCxMin, sParamRng.fCxMax, rand());
	m_fCy = (float)get_rand_num(sParamRng.fCyMin, sParamRng.fCyMax, rand());
	m_fRoll = (float)get_rand_num(sParamRng.fRollMin, sParamRng.fRollMax, rand());
	m_fPitch = (float)get_rand_num(sParamRng.fPitchMin, sParamRng.fPitchMax, rand());
	m_fYaw = (float)get_rand_num(sParamRng.fYawMin, sParamRng.fYawMax, rand());
	m_fTx = (float)get_rand_num(sParamRng.fTxMin, sParamRng.fTxMax, rand());
	m_fTy = (float)get_rand_num(sParamRng.fTyMin, sParamRng.fTyMax, rand());
	m_fTz = (float)get_rand_num(sParamRng.fTzMin, sParamRng.fTzMax, rand());
}

CCamCal::CCamCal(void)
{
	//! candidate points for vertical vanishing point Vy
	std::vector<cv::Point2f>().swap(m_voVyCand);

	//! candidate points on horizon line Linf
	std::vector<cv::Point2f>().swap(m_voLinfCand);
}

CCamCal::~CCamCal(void)
{
	//! candidate points for vertical vanishing point Vy
	std::vector<cv::Point2f>().swap(m_voVyCand);

	//! candidate points on horizon line Linf
	std::vector<cv::Point2f>().swap(m_voLinfCand);
}

void CCamCal::initialize(CCfg oCfg, cv::Mat oImgBg)
{
	// configuration parameters
	m_oCfg = oCfg;

	// background image for plotting results
	m_oImgBg = oImgBg.clone();

	// 25% progress flag for collecting 2D tracking data
	m_bCol25ProgFlg = false;

	// 50% progress flag for collecting 2D tracking data
	m_bCol50ProgFlg = false;

	// 75% progress flag for collecting 2D tracking data
	m_bCol75ProgFlg = false;

	// candidate points for vertical vanishing point Vy
	std::vector<cv::Point2f>().swap(m_voVyCand);

	// candidate points on horizon line Linf
	std::vector<cv::Point2f>().swap(m_voLinfCand);

    // estimated vanishing point Vr
    m_oVr = cv::Point2f((m_oCfg.getFrmSz().width - 1), 0.0f);

    // estimated vanishing point Vl
    m_oVl = cv::Point2f(0.0f, 0.0f);
}

bool CCamCal::process(std::vector<cv::Point> voVanPt)
{
	// manual calibration with input vanishing points
	if (2 == voVanPt.size())
	{
		m_oVr = voVanPt[0];
		m_oVl = voVanPt[1];
		estPrinPtByAC();

		// w/o optimization
		if (!m_oCfg.getCalEdaOptFlg())
			calCamDctComp();
		// w/ EDA optimization
		else
			calCamEdaOpt();

		return true;
	}
	else
		return false;
}

void CCamCal::estVyByCM(void)
{
	double fSumX = 0.0;
	double fSumY = 0.0;
	int nCandNum = m_voVyCand.size();

	for (int i = 0; i < nCandNum; i++)
	{
		fSumX += (double)m_voVyCand[i].x;
		fSumY += (double)m_voVyCand[i].y;
	}

	m_oVy.x = fSumX / (double)nCandNum;
	m_oVy.y = fSumY / (double)nCandNum;
}

void CCamCal::estVyByRS(void)
{
	int iRand0 = 0, iRand1 = 0, iRand2 = 0, iRand3 = 0, iIter = 0, nInlCnt, nMaxInlNum = INT_MIN, nCandNum = m_voVyCand.size();
	double fDist2Vy = 0.0, fVyCandX = 0.0, fVyCandY = 0.0, fMinVyCandX = DBL_MAX, fMaxVyCandX = -DBL_MAX;
	std::vector<bool> vbInl;
	std::vector<bool> vbMaxInl;

	while (VY_RS_ITER_NUM > iIter)
	{
		std::vector<bool>().swap(vbInl);
		for (int i = 0; i < nCandNum; i++)
			vbInl.push_back(true);

		nInlCnt = 0;
		fMinVyCandX = DBL_MAX;
		fMaxVyCandX = -DBL_MAX;

		iRand0 = rand() % nCandNum;
		iRand1 = rand() % nCandNum;
		iRand2 = rand() % nCandNum;
		iRand3 = rand() % nCandNum;

		while (iRand0 == iRand1)
			iRand1 = rand() % nCandNum;
		while ((iRand0 == iRand2) || (iRand1 == iRand2))
			iRand2 = rand() % nCandNum;
		while ((iRand0 == iRand3) || (iRand1 == iRand3) || (iRand2 == iRand3))
			iRand3 = rand() % nCandNum;

		fVyCandX = (m_voVyCand[iRand0].x + m_voVyCand[iRand1].x + m_voVyCand[iRand2].x + m_voVyCand[iRand3].x) / 4.0;
		fVyCandY = (m_voVyCand[iRand0].y + m_voVyCand[iRand1].y + m_voVyCand[iRand2].y + m_voVyCand[iRand3].y) / 4.0;

		for (int i = 0; i < nCandNum; i++)
		{
			fDist2Vy = std::sqrt(((fVyCandY - m_voVyCand[i].y) * (fVyCandY - m_voVyCand[i].y))
				+ ((fVyCandX - m_voVyCand[i].x) * (fVyCandX - m_voVyCand[i].x)));

			if ((VY_RS_DIST_THLD * (double)m_oCfg.getFrmSz().width)  > fDist2Vy)
			{
				if (fMinVyCandX > m_voVyCand[i].x)
					fMinVyCandX = m_voVyCand[i].x;
				if (fMaxVyCandX < m_voVyCand[i].x)
					fMaxVyCandX = m_voVyCand[i].x;
				nInlCnt++;
			}
			else
				vbInl[i] = false;
		}

		// add weight on nInlCnt based on the width of inlier points
		nInlCnt = (double)nInlCnt * ((double)m_oCfg.getFrmSz().width / (fMaxVyCandX - fMinVyCandX));

		if (nMaxInlNum < nInlCnt)
		{
			nMaxInlNum = nInlCnt;
			vbMaxInl = vbInl;
		}

		iIter++;
	}

	for (int i = nCandNum - 1; i >= 0; i--)
	{
		if (!vbMaxInl[i])
			m_voVyCand.erase(m_voVyCand.begin() + i);
	}

	// calculate the center of mass of the inliers as the final Vy
	estVyByCM();

	std::vector<bool>().swap(vbInl);
	std::vector<bool>().swap(vbMaxInl);
}

void CCamCal::estLinfByLR(void)
{
	double fSumX = 0.0;
	double fSumY = 0.0;
	int nCandNum = m_voLinfCand.size();

	for (int i = 0; i < nCandNum; i++)
	{
		fSumX += (double)m_voLinfCand[i].x;
		fSumY += (double)m_voLinfCand[i].y;
	}

	double fAveX = fSumX / (double)nCandNum;
	double fAveY = fSumY / (double)nCandNum;

	double fCovXY = 0.0;
	double fVarX = 0.0;
	double fVarY = 0.0;

	for (int i = 0; i < nCandNum; i++)
	{
		fCovXY += (m_voLinfCand[i].x - fAveX) * (m_voLinfCand[i].y - fAveY);
		fVarX += (m_voLinfCand[i].x - fAveX) * (m_voLinfCand[i].x - fAveX);
		fVarY += (m_voLinfCand[i].y - fAveY) * (m_voLinfCand[i].y - fAveY);
	}

	m_fLinfSlp = fCovXY / fVarX;
	m_fLinfItcp = fAveY - (m_fLinfSlp * fAveX);
	m_fLinfCorr = fCovXY / (std::sqrt(fVarX) * std::sqrt(fVarY));
}

void CCamCal::estLinfByRS(void)
{
	int iRand0 = 0, iRand1 = 0, iIter = 0, nInlCnt, nMaxInlNum = INT_MIN, nCandNum = m_voLinfCand.size();
	double fDist2Linf = 0.0;
	std::vector<bool> vbInl;
	std::vector<bool> vbMaxInl;

	while (LINF_RS_ITER_NUM > iIter)
	{
		std::vector<bool>().swap(vbInl);
		for (int i = 0; i < nCandNum; i++)
			vbInl.push_back(true);

		nInlCnt = 0;

		iRand0 = rand() % nCandNum;
		iRand1 = rand() % nCandNum;

		while (iRand0 == iRand1)
			iRand1 = rand() % nCandNum;

		double fDenom = std::sqrt(((m_voLinfCand[iRand1].y - m_voLinfCand[iRand0].y) * (m_voLinfCand[iRand1].y - m_voLinfCand[iRand0].y))
			+ ((m_voLinfCand[iRand1].x - m_voLinfCand[iRand0].x) * (m_voLinfCand[iRand1].x - m_voLinfCand[iRand0].x)));

		for (int i = 0; i < nCandNum; i++)
		{
			fDist2Linf = abs(((m_voLinfCand[iRand1].y - m_voLinfCand[iRand0].y) * m_voLinfCand[i].x)
				- ((m_voLinfCand[iRand1].x - m_voLinfCand[iRand0].x) * m_voLinfCand[i].y)
				+ (m_voLinfCand[iRand1].x * m_voLinfCand[iRand0].y)
				- (m_voLinfCand[iRand0].x * m_voLinfCand[iRand1].y)) / fDenom;

			if ((LINF_RS_DIST_THLD * (double)m_oCfg.getFrmSz().height) > fDist2Linf)
				nInlCnt++;
			else
				vbInl[i] = false;
		}

		if (nMaxInlNum < nInlCnt)
		{
			nMaxInlNum = nInlCnt;
			vbMaxInl = vbInl;
		}

		iIter++;
	}

	for (int i = nCandNum - 1; i >= 0; i--)
	{
		if (!vbMaxInl[i])
			m_voLinfCand.erase(m_voLinfCand.begin() + i);
	}

	// perform linear regression on the inliers to determine the final Linf
	estLinfByLR();

	std::vector<bool>().swap(vbInl);
	std::vector<bool>().swap(vbMaxInl);
}

void CCamCal::estPrinPtByAC(void)
{
	m_oPrinPt.x = (float)m_oCfg.getFrmSz().width / 2.0f;
	m_oPrinPt.y = (float)m_oCfg.getFrmSz().height / 2.0f;
}

void CCamCal::estPrinPtByMD(void)
{
	// perpendicular line to Linf passing through Vy
	float fPerpVyLinfSlp = -(1.0f / m_fLinfSlp);
	float fPerpVyLinfItcp = m_oVy.y + (m_oVy.x / m_fLinfSlp);

	// central point of the frame image
	cv::Point2f oFrmCentPt;
	oFrmCentPt.x = (float)m_oCfg.getFrmSz().width / 2.0f;
	oFrmCentPt.y = (float)m_oCfg.getFrmSz().height / 2.0f;

	m_oPrinPt.x = (oFrmCentPt.x + (fPerpVyLinfSlp * oFrmCentPt.y) - (fPerpVyLinfSlp * fPerpVyLinfItcp)) /
		((fPerpVyLinfSlp * fPerpVyLinfSlp) + 1.0f);
	m_oPrinPt.y = ((fPerpVyLinfSlp * oFrmCentPt.x) + ((fPerpVyLinfSlp * fPerpVyLinfSlp) * oFrmCentPt.y) + fPerpVyLinfItcp) /
		((fPerpVyLinfSlp * fPerpVyLinfSlp) + 1.0f);
}

void CCamCal::estVrVl(void)
{
    // assume the line connecting Vr and Vy always goes through the bottom right corner of the frame image
    float fAVrVy = (m_oCfg.getFrmSz().height - 1) - m_oVy.y;
    float fBVrVy = m_oVy.x - (m_oCfg.getFrmSz().width - 1);
    float fCVrVy = ((m_oCfg.getFrmSz().width - 1) * m_oVy.y) - (m_oVy.x * (m_oCfg.getFrmSz().height - 1));
    float fDVrVy = (fAVrVy * (-1.0f)) - (m_fLinfSlp * fBVrVy);
    if (0.0f == fDVrVy)
    {
        int nBtmRgtXInc = 0;

        while (0.0f == fDVrVy)
        {
            nBtmRgtXInc += 1;
            fAVrVy = (m_oCfg.getFrmSz().height - 1) - m_oVy.y;
            fBVrVy = m_oVy.x - (m_oCfg.getFrmSz().width + nBtmRgtXInc - 1);
            fCVrVy = ((m_oCfg.getFrmSz().width - 1) * m_oVy.y) - (m_oVy.x * (m_oCfg.getFrmSz().height - 1));
            fDVrVy = (fAVrVy * (-1.0f)) - (m_fLinfSlp * fBVrVy);
        }
    }

    m_oVr.x = ((fBVrVy * m_fLinfItcp) - ((-1.0f) * fCVrVy)) / fDVrVy;
    m_oVr.y = ((m_fLinfSlp * fCVrVy) - (fAVrVy * m_fLinfItcp)) / fDVrVy;

    float fVlPrinPtSlp = -1.0f / (-fAVrVy / fBVrVy);

    float fDVlPrinPt = fVlPrinPtSlp  - m_fLinfSlp;
    m_oVl.x = ((fVlPrinPtSlp * m_oPrinPt.x) - m_oPrinPt.y + m_fLinfItcp) / fDVlPrinPt;
    m_oVl.y = ((fVlPrinPtSlp * m_fLinfItcp) + (m_fLinfSlp * fVlPrinPtSlp * m_oPrinPt.x) - (m_fLinfSlp * m_oPrinPt.y)) / fDVlPrinPt;

    while ((m_oCfg.getFrmSz().width > m_oVr.x) || (0 <= m_oVl.x))
    {
        m_oVr.x -= 1.0f;
        m_oVr.y = (m_oVr.x * m_fLinfSlp) + m_fLinfItcp;

        fAVrVy = m_oVr.y - m_oVy.y;
        fBVrVy = m_oVy.x - m_oVr.x;
        fVlPrinPtSlp = -1.0f / (-fAVrVy / fBVrVy);
        fDVlPrinPt = fVlPrinPtSlp  - m_fLinfSlp;
        m_oVl.x = ((fVlPrinPtSlp * m_oPrinPt.x) - m_oPrinPt.y + m_fLinfItcp) / fDVlPrinPt;
        m_oVl.y = ((fVlPrinPtSlp * m_fLinfItcp) + (m_fLinfSlp * fVlPrinPtSlp * m_oPrinPt.x) - (m_fLinfSlp * m_oPrinPt.y)) / fDVlPrinPt;

        if (m_oCfg.getFrmSz().width > m_oVr.x)
            break;
    }
}

void CCamCal::calCamDctComp(void)
{
	double fReprojErr;

	float fCamHei = (m_oCfg.getCalCamHeiMax() + m_oCfg.getCalCamHeiMin()) / 2.0f;

	CCamParam oCamParam;

	compCamParam(m_oVr, m_oVl, m_oPrinPt, fCamHei, &oCamParam);

	cv::Point oStGrdPt = calcStGrdPt(&oCamParam);

	fReprojErr = calcReprojErr(&oCamParam);

	printf("Reprojection error = %.7f\n", fReprojErr);

	plt3dGrd(&oCamParam, m_oVr, m_oVl, oStGrdPt);
}

void CCamCal::calCamEdaOpt(void)
{
	int nR = EDA_INIT_POP, nN = EDA_SEL_POP, nIterNum = EDA_ITER_NUM, iIter = 0, iProc;
    bool bProc25, bProc50, bProc75;
    float fFx, fFy, fCx, fCy, fRoll, fPitch, fYaw, fTx, fTy, fTz;
	double fReprojErr, fReprojErrMean, fReprojErrMeanPrev, fReprojErrStd;
	cv::Point oStGrdPt;
    std::vector<CCamParam>::iterator ivoCamParam;
	CCamParam oCamParam, oCamParamRand;
	CCamParam::SParamRng sParamRng;

	// set starting grid point
	compCamParam(m_oVr, m_oVl, m_oPrinPt, ((m_oCfg.getCalCamHeiMax() + m_oCfg.getCalCamHeiMin()) / 2.0f), &oCamParam);
	oStGrdPt = calcStGrdPt(&oCamParam);

	// initialize range of each parameter
	sParamRng = initEdaParamRng(m_oVr, m_oVl, m_oPrinPt);

	// EDA optimization
	// EMNA-global
	if (nN >= nR)
		std::printf("Error: Selected population should be less than initial population\n");

	std::vector<CCamParam> voCamParam;

	for (int iR = 0; iR < nR; iR++)
	{
		oCamParamRand.initCamMdl(sParamRng);
		voCamParam.push_back(oCamParamRand);
	}

	std::printf("Start EDA optimization for camera calibration\n");

	while (nIterNum > iIter)
	{
		printf("==== generation %d: ====\n", iIter);
		iProc = 0;
		bProc25 = false;
		bProc50 = false;
		bProc75 = false;
		fReprojErrMean = 0.0;
		fReprojErrStd = 0.0;

		for (ivoCamParam = voCamParam.begin(); ivoCamParam != voCamParam.end(); ivoCamParam++)
		{
			fFx = ivoCamParam->getFx();
            fFy = ivoCamParam->getFy();
            fCx = ivoCamParam->getCx();
            fCy = ivoCamParam->getCy();
            fRoll = ivoCamParam->getRoll();
            fPitch = ivoCamParam->getPitch();
            fYaw = ivoCamParam->getYaw();
            fTx = ivoCamParam->getTx();
            fTy = ivoCamParam->getTy();
            fTz = ivoCamParam->getTz();

			oCamParam.setInParamMat(fFx, fFy, fCx, fCy);
			oCamParam.setRotMat(fRoll, fPitch, fYaw);
			oCamParam.setTntMat(fTx, fTy, fTz);
			oCamParam.calcProjMat();

			fReprojErr = calcReprojErr(&oCamParam);

			ivoCamParam->setReprojErr(fReprojErr);
			fReprojErrMean += fReprojErr;
			iProc++;

			if ((((float)iProc / (float)nR) > 0.25) && (!bProc25)) { std::printf("25%%..."); bProc25 = true; }
			if ((((float)iProc / (float)nR) > 0.50) && (!bProc50)) { std::printf("50%%..."); bProc50 = true; }
			if ((((float)iProc / (float)nR) > 0.75) && (!bProc75)) { std::printf("75%%..."); bProc75 = true; }
		}

		fReprojErrMean /= nR;

		for (ivoCamParam = voCamParam.begin(); ivoCamParam != voCamParam.end(); ivoCamParam++)
		{
			double fReprojErr = ivoCamParam->getReprojErr();
			fReprojErrStd += (fReprojErr - fReprojErrMean) * (fReprojErr - fReprojErrMean);
		}

		fReprojErrStd = std::sqrt(fReprojErrStd / nR);

		std::printf("100%%!\n");
		std::printf("current error mean = %f\n", fReprojErrMean);
		std::printf("current error standard deviation = %f\n", fReprojErrStd);

		if (!fReprojErrMean)
		{
			std::printf("Camera calibration fails.\n");
			break;
		}

		// Check if generation needs to stop
		if ((0 < iIter) && ((fReprojErrMeanPrev * EDA_REPROJ_ERR_THLD) > std::abs(fReprojErrMean - fReprojErrMeanPrev)))
		{
			std::printf("Reprojection error is small enough. Stop generation.\n");
			break;
		}

		fReprojErrMeanPrev = fReprojErrMean;

		std::stable_sort(voCamParam.begin(), voCamParam.end(), compReprojErr);
		voCamParam.erase(voCamParam.begin() + nN, voCamParam.end());

		plt3dGrd(&oCamParam, m_oVr, m_oVl, oStGrdPt);
		sParamRng = estEdaParamRng(&voCamParam);

		for (int iR = 0; iR < nR; iR++)
		{
			CCamParam oCamParamRand;
			oCamParamRand.initCamMdl(sParamRng);
			voCamParam.push_back(oCamParamRand);
		}

		iIter++;

		printf("\n");
	}

	if (nIterNum <= iIter)
		printf("Exit: Results can not converge.\n");
}

void CCamCal::compCamParam(cv::Point2f oVr, cv::Point2f oVl, cv::Point2f oPrinPt, float fCamHei, CCamParam* poCamParam)
{
	cv::Point2f oVrC, oVlC;
	cv::Point2f oVrCRot, oVlCRot;
	float fF, fRoll, fPitch, fYaw;

	// calculate f, roll, pitch, and yaw using vanishing points
	oVrC.x = oVr.x - oPrinPt.x;
	oVrC.y = oPrinPt.y - oVr.y;
	oVlC.x = oVl.x - oPrinPt.x;
	oVlC.y = oPrinPt.y - oVl.y;

	fRoll = std::atan2((oVrC.y - oVlC.y), (oVrC.x - oVlC.x));
	fRoll = (fRoll > (CV_PI / 2.0)) ? (fRoll - CV_PI) : fRoll;
	fRoll = (fRoll < (-CV_PI / 2.0)) ? (fRoll + CV_PI) : fRoll;

	oVrCRot = rotPt(oVrC, -fRoll);
	oVlCRot = rotPt(oVlC, -fRoll);

	fF = std::sqrt(-((oVrCRot.y * oVrCRot.y) + (oVrCRot.x * oVlCRot.x)));

	if (0 == COORD_SYS_TYP)
	{
		fPitch = -std::atan2(oVrCRot.y, fF);
		fYaw = -std::atan2(fF, (oVrCRot.x * std::cos(fPitch)));
	}
	else if (1 == COORD_SYS_TYP)
	{
		fPitch = -std::atan2(oVrCRot.y, fF);
		fYaw = -std::atan2((oVrCRot.x * std::cos(fPitch)), fF);
	}

	poCamParam->setFx(fF);
	poCamParam->setFy(fF);
	poCamParam->setCx(oPrinPt.x);
	poCamParam->setCy(oPrinPt.y);
	poCamParam->setRoll(fRoll);
	poCamParam->setPitch(fPitch);
	poCamParam->setYaw(fYaw);
	poCamParam->setTx(0.0f);
	if (0 == COORD_SYS_TYP)
	{
		poCamParam->setTy(-fCamHei * m_oCfg.getLenUnit());
		poCamParam->setTz(0.0f);
	}
	else if (1 == COORD_SYS_TYP)
	{
		poCamParam->setTy(0.0f);
		poCamParam->setTz(fCamHei * m_oCfg.getLenUnit());
	}

	poCamParam->setInParamMat(fF, fF, oPrinPt.x, oPrinPt.y);
	poCamParam->setRotMat(fRoll, fPitch, fYaw);
	if (0 == COORD_SYS_TYP)
		poCamParam->setTntMat(0.0f, (-fCamHei * m_oCfg.getLenUnit()), 0.0f);
	else if (1 == COORD_SYS_TYP)
		poCamParam->setTntMat(0.0f, 0.0f, (fCamHei * m_oCfg.getLenUnit()));
	poCamParam->calcProjMat();
}

double CCamCal::calcReprojErr(CCamParam* poCamParam)
{
	std::vector<cv::Point> voMeasLnSegNdPt = m_oCfg.getCalMeasLnSegNdPt();
	std::vector<float> vfMeasLnSegDist = m_oCfg.getCalMeasLnSegDist();
	double fReprojErr = 0.0;
	cv::Point oSt2dPt, oNd2dPt;
	cv::Point3d oSt3dPt, oNd3dPt;

	for (int i = 0; i < (voMeasLnSegNdPt.size() / 2); i++)
	{
		oSt2dPt = voMeasLnSegNdPt[i*2];
		oNd2dPt = voMeasLnSegNdPt[i*2+1];
		oSt3dPt = bkproj2d23d(oSt2dPt, poCamParam->getProjMat(), m_oCfg.getLenUnit());
		oNd3dPt = bkproj2d23d(oNd2dPt, poCamParam->getProjMat(), m_oCfg.getLenUnit());
		fReprojErr += std::abs(cv::norm(oNd3dPt - oSt3dPt) - vfMeasLnSegDist[i]);
	}

	return fReprojErr;
}

cv::Point CCamCal::calcStGrdPt(CCamParam* poCamParam)
{
	// look for the starting grid location that will make all the grid points within the frame image
	bool bStGrdPtFlg = false;
	int nMaxDist, nMaxSumSqDist = 0;
	std::vector<cv::Point> voPt;
	cv::Point oStGrdPt;

	// iterate from smallest distance to largest distance to the original point
	while (true)
	{
		std::vector<cv::Point>().swap(voPt);
		nMaxDist = std::sqrt(nMaxSumSqDist);

		for (int i = 0; i <= nMaxDist; i++)
		{
			for (int j = i; j <= nMaxDist; j++)
			{
				if (nMaxSumSqDist == (i * i) + (j * j))
					voPt.push_back(cv::Point(i, j));
			}
		}

		if (voPt.size())
		{
			for (int i = 0; i < voPt.size(); i++)
			{
				oStGrdPt.x = voPt[i].x; oStGrdPt.y = voPt[i].y;
				if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }

				if (0 < voPt[i].x)
				{
					oStGrdPt.x = -voPt[i].x; oStGrdPt.y = voPt[i].y;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}

				if (0 < voPt[i].y)
				{
					oStGrdPt.x = voPt[i].x; oStGrdPt.y = -voPt[i].y;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}

				if (0 < voPt[i].x)
				{
					oStGrdPt.x = -voPt[i].x; oStGrdPt.y = -voPt[i].y;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}

				if (voPt[i].x < voPt[i].y)
				{
					oStGrdPt.x = voPt[i].y; oStGrdPt.y = voPt[i].x;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}

				if (voPt[i].x < voPt[i].y)
				{
					oStGrdPt.x = -voPt[i].y; oStGrdPt.y = voPt[i].x;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}

				if ((voPt[i].x < voPt[i].y) && (0 < voPt[i].x))
				{
					oStGrdPt.x = voPt[i].y; oStGrdPt.y = -voPt[i].x;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}

				if ((voPt[i].x < voPt[i].y) && (0 < voPt[i].x))
				{
					oStGrdPt.x = -voPt[i].y; oStGrdPt.y = -voPt[i].x;
					if (tstStGrdPt(oStGrdPt, poCamParam)) { bStGrdPtFlg = true; break; }
				}
			}
		}

		if (bStGrdPtFlg)
			break;

		nMaxSumSqDist++;
	}

	return oStGrdPt;
}

bool CCamCal::tstStGrdPt(cv::Point oStGrdPt, CCamParam* poCamParam)
{
	int nLftEdgX = (-(IMG_EXPN_RAT - 1.0f) / 2.0f) * (float)m_oCfg.getFrmSz().width;
	int nTopEdgY = (-(IMG_EXPN_RAT - 1.0f) / 2.0f) * (float)m_oCfg.getFrmSz().height;
	int nRgtEdgX = ((IMG_EXPN_RAT + 1.0f) / 2.0f) * (float)m_oCfg.getFrmSz().width;
	int nBtmEdgY = ((IMG_EXPN_RAT + 1.0f) / 2.0f) * (float)m_oCfg.getFrmSz().height;

	cv::Point2f o2dStPt(-1.0f, -1.0f), o2dRNdPt(-1.0f, -1.0f), o2dLNdPt(-1.0f, -1.0f), o2dNdPt(-1.0f, -1.0f);

	cv::Point oNdGrdPt;
	oNdGrdPt.x = oStGrdPt.x + m_oCfg.getCalGrdSzR();
	oNdGrdPt.y = oStGrdPt.y + m_oCfg.getCalGrdSzL();

	if (0 == COORD_SYS_TYP)
		o2dStPt = proj3d22d(cv::Point3f(oStGrdPt.x, 0.0f, oStGrdPt.y), poCamParam->getProjMat(), m_oCfg.getLenUnit());
	else if (1 == COORD_SYS_TYP)
		o2dStPt = proj3d22d(cv::Point3f(oStGrdPt.x, oStGrdPt.y, 0.0f), poCamParam->getProjMat(), m_oCfg.getLenUnit());

	if (0 == COORD_SYS_TYP)
		o2dRNdPt = proj3d22d(cv::Point3f(oNdGrdPt.x, 0.0f, oStGrdPt.y), poCamParam->getProjMat(), m_oCfg.getLenUnit());
	else if (1 == COORD_SYS_TYP)
		o2dRNdPt = proj3d22d(cv::Point3f(oNdGrdPt.x, oStGrdPt.y, 0.0f), poCamParam->getProjMat(), m_oCfg.getLenUnit());

	if (0 == COORD_SYS_TYP)
		o2dLNdPt = proj3d22d(cv::Point3f(oStGrdPt.x, 0.0f, oNdGrdPt.y), poCamParam->getProjMat(), m_oCfg.getLenUnit());
	else if (1 == COORD_SYS_TYP)
		o2dLNdPt = proj3d22d(cv::Point3f(oStGrdPt.x, oNdGrdPt.y, 0.0f), poCamParam->getProjMat(), m_oCfg.getLenUnit());

	if (0 == COORD_SYS_TYP)
		o2dNdPt = proj3d22d(cv::Point3f(oNdGrdPt.x, 0.0f, oNdGrdPt.y), poCamParam->getProjMat(), m_oCfg.getLenUnit());
	else if (1 == COORD_SYS_TYP)
		o2dNdPt = proj3d22d(cv::Point3f(oNdGrdPt.x, oNdGrdPt.y, 0.0f), poCamParam->getProjMat(), m_oCfg.getLenUnit());

	if (((o2dStPt.x >= nLftEdgX) && (o2dStPt.y >= nTopEdgY) && (o2dStPt.x < nRgtEdgX) && (o2dStPt.y < nBtmEdgY)) &&
		((o2dRNdPt.x >= nLftEdgX) && (o2dRNdPt.y >= nTopEdgY) && (o2dRNdPt.x < nRgtEdgX) && (o2dRNdPt.y < nBtmEdgY)) &&
		((o2dLNdPt.x >= nLftEdgX) && (o2dLNdPt.y >= nTopEdgY) && (o2dLNdPt.x < nRgtEdgX) && (o2dLNdPt.y < nBtmEdgY)) &&
		((o2dNdPt.x >= nLftEdgX) && (o2dNdPt.y >= nTopEdgY) && (o2dNdPt.x < nRgtEdgX) && (o2dNdPt.y < nBtmEdgY)))
		return true;
	else
		return false;
}

CCamParam::SParamRng CCamCal::initEdaParamRng(cv::Point2f oVr, cv::Point2f oVl, cv::Point2f oPrinPt, CCamParam* poCamParam)
{
	cv::Point2f oVrC, oVlC;
	cv::Point2f oVrCRot, oVlCRot;
	float fF, fRoll, fPitch, fYaw;
	CCamParam::SParamRng sParamRng;

	// calculate f, roll, pitch, and yaw by using vanishing points
	oVrC.x = oVr.x - oPrinPt.x;
	oVrC.y = oPrinPt.y - oVr.y;
	oVlC.x = oVl.x - oPrinPt.x;
	oVlC.y = oPrinPt.y - oVl.y;

	fRoll = std::atan2((oVrC.y - oVlC.y), (oVrC.x - oVlC.x));
	fRoll = (fRoll > (CV_PI / 2.0)) ? (fRoll - CV_PI) : fRoll;
	fRoll = (fRoll < (-CV_PI / 2.0)) ? (fRoll + CV_PI) : fRoll;

	oVrCRot = rotPt(oVrC, -fRoll);
	oVlCRot = rotPt(oVlC, -fRoll);

	if (poCamParam)
	{
		float* acK = poCamParam->getInParamMat();
		fF = (acK[0] + acK[4]) / 2.0f;
	}
	else
	{
		fF = std::sqrt(-((oVrCRot.y * oVrCRot.y) + (oVrCRot.x * oVlCRot.x)));
	}

	if (0 == COORD_SYS_TYP)
	{
		fPitch = std::atan2(oVrCRot.y, fF);
		fYaw = -std::atan2(fF, (oVrCRot.x * cos(fPitch)));
	}
	else if (1 == COORD_SYS_TYP)
	{
		fPitch = -std::atan2(oVrCRot.y, fF);
		fYaw = -std::atan2((oVrCRot.x * cos(fPitch)), fF);
	}

	// construct ranges of camera parameters
	sParamRng.fFxMax = (poCamParam) ? poCamParam->getInParamMat()[0] : fF * (1.0f + EDA_RNG_F);
	sParamRng.fFxMin = (poCamParam) ? poCamParam->getInParamMat()[0] : fF * (1.0f - EDA_RNG_F);
	sParamRng.fFyMax = (poCamParam) ? poCamParam->getInParamMat()[4] : fF * (1.0f + EDA_RNG_F);
	sParamRng.fFyMin = (poCamParam) ? poCamParam->getInParamMat()[4] : fF * (1.0f - EDA_RNG_F);
	sParamRng.fCxMax = (poCamParam) ? poCamParam->getInParamMat()[2] : oPrinPt.x + EDA_RNG_PRIN_PT;
	sParamRng.fCxMin = (poCamParam) ? poCamParam->getInParamMat()[2] : oPrinPt.x - EDA_RNG_PRIN_PT;
	sParamRng.fCyMax = (poCamParam) ? poCamParam->getInParamMat()[5] : oPrinPt.y + EDA_RNG_PRIN_PT;
	sParamRng.fCyMin = (poCamParam) ? poCamParam->getInParamMat()[5] : oPrinPt.y - EDA_RNG_PRIN_PT;
	sParamRng.fRollMax = fRoll + deg2rad(EDA_RNG_ROT_ANG);
	sParamRng.fRollMin = fRoll - deg2rad(EDA_RNG_ROT_ANG);
	sParamRng.fPitchMax = fPitch + deg2rad(EDA_RNG_ROT_ANG);
	sParamRng.fPitchMin = fPitch - deg2rad(EDA_RNG_ROT_ANG);
	sParamRng.fYawMax = fYaw + deg2rad(EDA_RNG_ROT_ANG);
	sParamRng.fYawMin = fYaw - deg2rad(EDA_RNG_ROT_ANG);
	sParamRng.fTxMax = 0.0f;
	sParamRng.fTxMin = 0.0f;
	if (0 == COORD_SYS_TYP)
	{
		sParamRng.fTyMax = -m_oCfg.getCalCamHeiMin() * m_oCfg.getLenUnit();
		sParamRng.fTyMin = -m_oCfg.getCalCamHeiMax() * m_oCfg.getLenUnit();
		sParamRng.fTzMax = 0.0f;
		sParamRng.fTzMin = 0.0f;
	}
	else if (1 == COORD_SYS_TYP)
	{
		sParamRng.fTyMax = 0.0f;
		sParamRng.fTyMin = 0.0f;
		sParamRng.fTzMax = m_oCfg.getCalCamHeiMax() * m_oCfg.getLenUnit();
		sParamRng.fTzMin = m_oCfg.getCalCamHeiMin() * m_oCfg.getLenUnit();
	}

	return sParamRng;
}

CCamParam::SParamRng CCamCal::estEdaParamRng(std::vector<CCamParam>* pvoCamParam)
{
	int nCamParamNum = pvoCamParam->size(), nParamNum = 10, iParam, iCamParam = 0;
	float fParamVar;
	float* afParamMean = (float*)calloc(nParamNum, sizeof(float));
	float* afParamData = (float*)calloc(nParamNum*nCamParamNum, sizeof(float));
	std::vector<CCamParam>::iterator ivoCamParam;
	CCamParam::SParamRng sParamRng;

	// calculate means of parameters
	for (ivoCamParam = pvoCamParam->begin(); ivoCamParam != pvoCamParam->end(); ivoCamParam++)
	{
        iParam = 0;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getFx();    afParamMean[0] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getFy();    afParamMean[1] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getCx();    afParamMean[2] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getCy();    afParamMean[3] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getRoll();  afParamMean[4] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getPitch(); afParamMean[5] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getYaw();   afParamMean[6] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getTx();    afParamMean[7] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getTy();    afParamMean[8] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		afParamData[iParam*nCamParamNum + iCamParam] = ivoCamParam->getTz();    afParamMean[9] += afParamData[iParam*nCamParamNum + iCamParam];  iParam++;
		iCamParam++;
	}

	for (iParam = 0; iParam < nParamNum; iParam++)
		afParamMean[iParam] /= nCamParamNum;

	// fx
	iParam = 0;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fFxMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fFxMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// fy
	iParam = 1;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fFyMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fFyMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// cx
	iParam = 2;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fCxMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fCxMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// cy
	iParam = 3;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fCyMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fCyMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// roll
	iParam = 4;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fRollMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fRollMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// pitch
	iParam = 5;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fPitchMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fPitchMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// yaw
	iParam = 6;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fYawMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fYawMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// tz
	iParam = 7;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fTxMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fTxMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// ty
	iParam = 8;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fTyMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fTyMin = afParamMean[iParam] - std::sqrt(fParamVar);

	// tz
	iParam = 9;
	fParamVar = 0.0f;
	for (iCamParam = 0; iCamParam < nCamParamNum; iCamParam++)
		fParamVar += (afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]) *
		(afParamData[(iParam * nCamParamNum) + iCamParam] - afParamMean[iParam]);
	fParamVar /= nCamParamNum;
	sParamRng.fTzMax = afParamMean[iParam] + std::sqrt(fParamVar);
	sParamRng.fTzMin = afParamMean[iParam] - std::sqrt(fParamVar);

	std::free(afParamMean);
	std::free(afParamData);

	return sParamRng;
}

void CCamCal::plt3dGrd(CCamParam* poCamParam, cv::Point2f oVr, cv::Point2f oVl, cv::Point oStGrdPt)
{
	cv::Point2f oVrExpn = projPtOrig2Expn(oVr, IMG_EXPN_RAT, m_oCfg.getFrmSz()),
		oVlExpn = projPtOrig2Expn(oVl, IMG_EXPN_RAT, m_oCfg.getFrmSz());

	cv::Mat oImgExpn = genExpnImg(IMG_EXPN_RAT, m_oCfg.getFrmSz());
	cv::Mat oImgCent(oImgExpn,
		cv::Rect(((IMG_EXPN_RAT - 1.0f) * m_oCfg.getFrmSz().width / 2.0f),
			((IMG_EXPN_RAT - 1.0f) * m_oCfg.getFrmSz().height / 2.0f),
			m_oCfg.getFrmSz().width, m_oCfg.getFrmSz().height));
	m_oImgBg.copyTo(oImgCent);

	FILE* pfCamParam;
	pfCamParam = std::fopen(m_oCfg.getOutCamParamPth(), "w");
	float *afK, *afR, *afT, *afP;
	afK = poCamParam->getInParamMat();
	afR = poCamParam->getRotMat();
	afT = poCamParam->getTntMat();
	afP = poCamParam->getProjMat();

	std::fprintf(pfCamParam, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f\n",
		afK[0], afK[1], afK[2], afK[3], afK[4], afK[5], afK[6], afK[7], afK[8]);
	std::fprintf(pfCamParam, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f\n",
		afR[0], afR[1], afR[2], afR[3], afR[4], afR[5], afR[6], afR[7], afR[8]);
	std::fprintf(pfCamParam, "%.7f %.7f %.7f\n", afT[0], afT[1], afT[2]);
	std::fprintf(pfCamParam, "%.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f\n",
		afP[0], afP[1], afP[2], afP[3], afP[4], afP[5], afP[6], afP[7], afP[8], afP[9], afP[10], afP[11]);
	std::fclose(pfCamParam);

	//FILE* pfIndCamParam = std::fopen(".\\data\\params.txt", "w");	// in Windows
	//FILE* pfIndCamParam = std::fopen("./data/params.txt", "w");	// in Linux
	//std::fprintf(pfIndCamParam, "fx: %.7f\n", poCamParam->getFx());
	//std::fprintf(pfIndCamParam, "fy: %.7f\n", poCamParam->getFy());
	//std::fprintf(pfIndCamParam, "cx: %.7f\n", poCamParam->getCx());
	//std::fprintf(pfIndCamParam, "cy: %.7f\n", poCamParam->getCy());
	//std::fprintf(pfIndCamParam, "roll: %.7f\n", rad2deg(poCamParam->getRoll()));
	//std::fprintf(pfIndCamParam, "pitch: %.7f\n", rad2deg(poCamParam->getPitch()));
	//std::fprintf(pfIndCamParam, "yaw: %.7f\n", rad2deg(poCamParam->getYaw()));
	//std::fclose(pfIndCamParam);

	cv::Point2f o2dMeasPt;
	cv::circle(oImgExpn, oVrExpn, 3, cv::Scalar(255, 128, 0, 0), 2);
	cv::circle(oImgExpn, oVlExpn, 3, cv::Scalar(255, 128, 0, 0), 2);

	cv::Point oNdGrdPt;
	oNdGrdPt.x = oStGrdPt.x + m_oCfg.getCalGrdSzR();
	oNdGrdPt.y = oStGrdPt.y + m_oCfg.getCalGrdSzL();

	std::vector<cv::Point> voMeasLnSegNdPt = m_oCfg.getCalMeasLnSegNdPt();
	cv::Point oSt2dPt, oNd2dPt;

	for (int i = 0; i < (voMeasLnSegNdPt.size() / 2); i++)
	{
		oSt2dPt = voMeasLnSegNdPt[i * 2];
		oNd2dPt = voMeasLnSegNdPt[i * 2 + 1];
		cv::line(oImgExpn, projPtOrig2Expn(oSt2dPt, IMG_EXPN_RAT, m_oCfg.getFrmSz()), 
			projPtOrig2Expn(oNd2dPt, IMG_EXPN_RAT, m_oCfg.getFrmSz()), cv::Scalar(0, 255, 0, 0), 3);
	}

	// draw points
	for (int iL = oStGrdPt.y; iL < oNdGrdPt.y; iL++)
	{
		for (int iR = oStGrdPt.x; iR < oNdGrdPt.x; iR++)
		{
			if (0 == COORD_SYS_TYP)
				o2dMeasPt = proj3d22d(cv::Point3f(iR, 0.0f, iL), poCamParam->getProjMat(), m_oCfg.getLenUnit());
			else if (1 == COORD_SYS_TYP)
				o2dMeasPt = proj3d22d(cv::Point3f(iR, iL, 0.0f), poCamParam->getProjMat(), m_oCfg.getLenUnit());
			o2dMeasPt = projPtOrig2Expn(o2dMeasPt, IMG_EXPN_RAT, m_oCfg.getFrmSz());

			if ((0 <= o2dMeasPt.x) && (oImgExpn.size().width > o2dMeasPt.x) &&
				(0 <= o2dMeasPt.y) && (oImgExpn.size().height > o2dMeasPt.y))
				cv::circle(oImgExpn, o2dMeasPt, 3, cv::Scalar(0, 0, 255, 0), 10);
		}
	}

	cv::Mat oImgDisp = cv::Mat(cv::Size(1920, (oImgExpn.size().height * 1920 / oImgExpn.size().width)), CV_8UC3);
	cv::resize(oImgExpn, oImgDisp, oImgDisp.size());
	cv::namedWindow("3D grid on ground plane", CV_WINDOW_NORMAL);
	cv::imshow("3D grid on ground plane", oImgDisp);
	cv::waitKey(1);
	//cv::imwrite(".\\data\\3dgrid.jpg", oImgDisp);	// for debug	// in Windows
	//cv::imwrite("./data/3dgrid.jpg", oImgDisp);	// for debug	// in Linux
}

CVanLnSel::CVanLnSel(void)
{

}

CVanLnSel::~CVanLnSel(void)
{

}

void CVanLnSel::initialize(CCfg oCfg, cv::Mat oImgBg)
{
	// configuration parameters
	m_oCfg = oCfg;

	// background image for plotting results
	m_oImgBg = oImgBg.clone();

	// flag of completing the selection of vanishing lines
	m_bSelVanLnFlg = false;
}

std::vector<cv::Point> CVanLnSel::process(void)
{
	std::vector<cv::Point> voVanPt;

	if (m_oCfg.getCalSelVanLnFlg())
	{
		std::cout << "Hot keys: \n"
			<< "\tESC - exit\n"
			<< "\tr - re-select two pairs of vanishing lines\n"
			<< "\to - finish selecting two pairs of vanishing lines\n";

		cv::Mat oImgBg = m_oImgBg.clone();

		cv::namedWindow("selector of vanishing lines", CV_WINDOW_NORMAL);
		cv::imshow("selector of vanishing lines", oImgBg);
		cv::setMouseCallback("selector of vanishing lines", on_mouse3);  // register for mouse event

		while (1)
		{
			int nKey = cv::waitKey(0);	// read keyboard event

			if (nKey == 27)
				break;

			if (nKey == 'r')  // reset the nodes
			{
				std::vector<cv::Point>().swap(m_voNd);
				m_bSelVanLnFlg = false;
				m_oImgBg = oImgBg.clone();
				cv::imshow("selector of vanishing lines", m_oImgBg);
			}

			if (nKey == 'o' && m_bSelVanLnFlg)	// finish selection of two pairs of parallel lines
			{
				voVanPt = compVanPts();
				break;
			}
		}

		cv::destroyWindow("selector of vanishing lines");
	}
	else
	{

		voVanPt.push_back(m_oCfg.getCalVr());
		voVanPt.push_back(m_oCfg.getCalVl());
	}

	std::cout << "Vanishing point (right): (" << voVanPt[0].x << "," << voVanPt[0].y << ")" << std::endl;
	std::cout << "Vanishing point (left): (" << voVanPt[1].x << "," << voVanPt[1].y << ")" << std::endl;

	return voVanPt;
}

void CVanLnSel::addNd(int nX, int nY)
{
	cv::Point oCurrNd;
	oCurrNd.x = nX;
	oCurrNd.y = nY;

	if (1 == m_voNd.size() % 2)
	{
		m_voNd.push_back(oCurrNd);
		// std::cout << "current node(" << oCurrNd.x << "," << oCurrNd.y << ")" << std::endl;	// for debug

		// draw the line if the node is in even order
		cv::Point oPrevNd = m_voNd[m_voNd.size() - 2];
		// std::cout << "previous node(" << oPrevNd.x << "," << oPrevNd.y << ")" << std::endl << endl;	// for debug
		cv::circle(m_oImgBg, oCurrNd, 6, cv::Scalar(255, 0, 0), 1, CV_AA);  // draw the circle
		cv::line(m_oImgBg, oPrevNd, oCurrNd, cv::Scalar(255, 255, 255), 2, CV_AA);
		cv::imshow("selector of vanishing lines", m_oImgBg);

		if ((2 == m_voNd.size()) || (6 == m_voNd.size()))
			std::cout << "Select a line parallel to the previous straight line.\n";
		else if (4 == m_voNd.size())
			std::cout << "Select a line perpendicular to the previous pair of parallel lines.\n";
		else
			m_bSelVanLnFlg = true;
	}
	else
	{
		m_voNd.push_back(oCurrNd);
		// std::cout << "current node(" << oCurrNd.x << "," << oCurrNd.y << ")" << std::endl;	// for debug
		cv::circle(m_oImgBg, oCurrNd, 6, cv::Scalar(255, 0, 0), 1, CV_AA);  // draw the circle
		cv::imshow("selector of vanishing lines", m_oImgBg);
	}
}

std::vector<cv::Point> CVanLnSel::compVanPts(void)
{
	if (8 != m_voNd.size())
	{
		std::printf("Error: not enough nodes of vanishing lines selected");
		cv::waitKey(0);
	}

	std::vector<cv::Point> voVanPt;

	double fSlpPr1Ln1 = (double)(m_voNd[0].y - m_voNd[1].y) / (double)(m_voNd[0].x - m_voNd[1].x);
	double fSlpPr1Ln2 = (double)(m_voNd[2].y - m_voNd[3].y) / (double)(m_voNd[2].x - m_voNd[3].x);
	cv::Point oVanPt1;
	oVanPt1.x = ((fSlpPr1Ln1 * m_voNd[0].x) - (fSlpPr1Ln2 * m_voNd[2].x)
		+ m_voNd[2].y - m_voNd[0].y) / (fSlpPr1Ln1 - fSlpPr1Ln2);
	oVanPt1.y = m_voNd[0].y + ((oVanPt1.x - m_voNd[0].x) * fSlpPr1Ln1);

	double fSlpPr2Ln1 = (double)(m_voNd[4].y - m_voNd[5].y) / (double)(m_voNd[4].x - m_voNd[5].x);
	double fSlpPr2Ln2 = (double)(m_voNd[6].y - m_voNd[7].y) / (double)(m_voNd[6].x - m_voNd[7].x);
	cv::Point oVanPt2;
	oVanPt2.x = ((fSlpPr2Ln1 * m_voNd[4].x) - (fSlpPr2Ln2 * m_voNd[6].x)
		+ m_voNd[6].y - m_voNd[4].y) / (fSlpPr2Ln1 - fSlpPr2Ln2);
	oVanPt2.y = m_voNd[4].y + ((oVanPt2.x - m_voNd[4].x) * fSlpPr2Ln1);

	if (oVanPt1.x >= oVanPt2.x)
	{
		voVanPt.push_back(oVanPt1);
		voVanPt.push_back(oVanPt2);
	}
	else
	{
		voVanPt.push_back(oVanPt2);
		voVanPt.push_back(oVanPt1);
	}

	return voVanPt;
}