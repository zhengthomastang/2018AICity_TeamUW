#include "Cfg.h"

CCfg::CCfg()
{
	m_oFrmSz = cv::Size(1920, 1080);
	//std::strcpy(m_acInFrmPth, ".\\data\\frm.png");	// in Windows
	std::strcpy(m_acInFrmPth, "./data/frm.png");	// in Linux
	//std::strcpy(m_acOutCamParamPth, ".\\data\\camParam.txt");	// in Windows
	std::strcpy(m_acOutCamParamPth, "./data/camParam.txt");	// in Linux
	m_nRszFrmHei = -1;
	m_nLenUnit = 1000;
	m_bCalSelVanLnFlg = false;
	m_oCalVr = cv::Point(-1, -1);
	m_oCalVl = cv::Point(-1, -1);
	m_fCalCamHeiMax = -1.0f;
	m_fCalCamHeiMin = -1.0f;
	m_nCalGrdSzR = 10;
	m_nCalGrdSzL = 10;
	m_bCalEdaOptFlg = true;
	std::vector<cv::Point>().swap(m_voCalMeasLnSegNdPt);
	std::vector<float>().swap(m_vfCalMeasLnSegDist);
}

CCfg::~CCfg()
{

}

void CCfg::ldCfgFl()
{
	FILE * poCfgFl;
	long nlFlSz, nlRdRst;
	char * pcBuf;

	//poCfgFl = std::fopen(".\\data\\cfg.json", "r");	// in Windows
	poCfgFl = std::fopen("./data/cfg.json", "r");	// in Linux
	if (poCfgFl == NULL) { std::fputs("Error: configuration file not opened", stderr); exit(1); }

	// obtain file size:
	fseek(poCfgFl, 0, SEEK_END);
	nlFlSz = ftell(poCfgFl);
	rewind(poCfgFl);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFlSz);
	if (pcBuf == NULL) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFlSz, poCfgFl);
	//if (nlRdRst != nlFlSz) { fputs("Reading error", stderr); exit(3); }

	std::string strCfg(pcBuf);
	//strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), [](char c) { return c >= 0 && isspace(c); }), strCfg.end());	// in Windows
    strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), ::isspace), strCfg.end());	// in Linux

	int nParamPos = strCfg.find("\"inFrmPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acInFrmPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outCamParamPth\"");
	if (nParamPos != std::string::npos)
		std::strcpy(m_acOutCamParamPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"rszFrmHei\"");
	if (nParamPos != std::string::npos)
		m_nRszFrmHei = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"lenUnit\"");
	if (nParamPos != std::string::npos)
		m_nLenUnit = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calSelVanLnFlg\"");
	if (nParamPos != std::string::npos)
		m_bCalSelVanLnFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calVr\"");
	if (nParamPos != std::string::npos)
		m_oCalVr = rd2dPt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calVl\"");
	if (nParamPos != std::string::npos)
		m_oCalVl = rd2dPt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calCamHeiMax\"");
	if (nParamPos != std::string::npos)
		m_fCalCamHeiMax = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calCamHeiMin\"");
	if (nParamPos != std::string::npos)
		m_fCalCamHeiMin = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calGrdSzR\"");
	if (nParamPos != std::string::npos)
		m_nCalGrdSzR = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calGrdSzL\"");
	if (nParamPos != std::string::npos)
		m_nCalGrdSzL = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calEdaOptFlg\"");
	if (nParamPos != std::string::npos)
		m_bCalEdaOptFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calMeasLnSegNdPt\"");
	if (nParamPos != std::string::npos)
		m_voCalMeasLnSegNdPt = rdVec2dPt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"calMeasLnSegDist\"");
	if (nParamPos != std::string::npos)
		m_vfCalMeasLnSegDist = rdVecFlt(strCfg, nParamPos);

	// assertion
	CV_Assert(m_fCalCamHeiMax >= m_fCalCamHeiMin);
	CV_Assert(0 == m_voCalMeasLnSegNdPt.size() % 2);
	CV_Assert(m_vfCalMeasLnSegDist.size() == (m_voCalMeasLnSegNdPt.size() / 2)); 

	// terminate
	fclose(poCfgFl);
	free(pcBuf);
}

std::string CCfg::rdCharArr(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValLen = strCfg.find("\"", (nValPos + 1)) - nValPos;

	return strCfg.substr(nValPos, nValLen);
}

int CCfg::rdInt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atoi(strCfg.substr(nValPos, nValLen).c_str());
}

float CCfg::rdFlt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atof(strCfg.substr(nValPos, nValLen).c_str());
}

bool CCfg::rdBool(std::string strCfg, int nParamPos)
{
	int nBoolVal, nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	nBoolVal = std::atoi(strCfg.substr(nValPos, nValLen).c_str());
	if (nBoolVal > 0)
		return true;
	else if (nBoolVal <= 0)
		return false;
}

cv::Point CCfg::rd2dPt(std::string strCfg, int nParamPos)
{
	int nPtXPos = nParamPos, nPtXLen, nPtYPos, nPtYLen;

	nPtXPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nPtXLen = strCfg.find(",", (nPtXPos + 1)) - nPtXPos;
	nPtYPos = nPtXPos + nPtXLen + 1;
	nPtYLen = strCfg.find("]", (nPtYPos + 1)) - nPtYPos;
	cv::Point oPt(std::atof(strCfg.substr(nPtXPos, nPtXLen).c_str()), std::atof(strCfg.substr(nPtYPos, nPtYLen).c_str()));

	return oPt;
}

std::vector<float> CCfg::rdVecFlt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd;
	std::vector<float> vfVal;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValEnd = strCfg.find("]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nValLen = strCfg.find(",", (nValPos + 1)) - nValPos;
		if (0 > nValLen)
			nValLen = nValEnd - nValPos;
		vfVal.push_back(std::atof(strCfg.substr(nValPos, nValLen).c_str()));
		nValPos = nValPos + nValLen + 1;
	}

	return vfVal;
}

std::vector<cv::Point> CCfg::rdVec2dPt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd, nXPos = nParamPos, nXLen, nYPos, nYLen;
	std::vector<cv::Point> vo2dPt;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd = strCfg.find("]]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nXPos = strCfg.find("[", (nValPos + 1)) + 1;
		nXLen = strCfg.find(",", (nXPos + 1)) - nXPos;
		nYPos = nXPos + nXLen + 1;
		nYLen = strCfg.find("]", (nYPos + 1)) - nYPos;
		vo2dPt.push_back(cv::Point(std::atoi(strCfg.substr(nXPos, nXLen).c_str()), std::atoi(strCfg.substr(nYPos, nYLen).c_str())));
		nValPos = nYPos + nYLen + 1;
	}

	return vo2dPt;
}