#include "stdafx.h"
#include "SlamDataSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   清除数据集。
//
void CSlamDataSet::Clear()
{
	vector<CSlamStepData>::clear();

	m_ScannerParam.clear();
}

//   装入二进制原始扫描点数据文件。
//   返回值：
//   < 0 - 出错
//     0 - 判断此文件为未经综合的原始数据集文件，且目前已读完
//     1 - 尚未读完数据集的原始部分，状态正常
//     2 - 判断此文件为综合后的数据集文件，且目前原始数据已读完
//
int CSlamDataSet::LoadRawScanBinary(FILE* fp)
{
	Clear();

	if (fread(&m_nFileVersion, sizeof(int), 1, fp) != 1)
		return -1;
	
	unsigned int uTimeStamp;
	if (m_nFileVersion >= 210)
	{
		if (fread(&uTimeStamp, sizeof(unsigned int), 1, fp) != 1)
			return false;

		m_uStartTime = uTimeStamp;
	}

	if (!m_ScannerParam.LoadBinary(fp, m_nFileVersion))
		return -1;

	clear();

	CSlamStepData StepData;

	// 如果版本在V2.00以上，接下来将直接读入总的步数
	if (m_nFileVersion >= 200)
	{
		int nStepsCount = 0;
		if (fread(&nStepsCount, sizeof(int), 1, fp) != 1)
			return -1;

		nStepsCount--;
		bool bEof = false;
		// 根据步数读入全部的局部数据
		for (int i = 0; i < nStepsCount; i++)
		{
			if (!StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, (i == 0)))
			{
				bEof = true;
				break;
			}
			push_back(StepData);
		}

		// 试读下一个标志数据，看看文件是否结束(要求标志字符需要为'O')
		if (!bEof)
		{
			unsigned char ch;
			if (fread(&ch, sizeof(unsigned char), 1, fp) != 1 || ch != 'O')
			{
				// 已到文件尾，说明这是一个原始数据集文件，不含帧间匹配数据
			}
		}
	}
	// 如果文件版本在V2.00以下，需要以文件尾作为结束判断条件
	else
	{
		// 读入全部的局部数据
		bool bFirstStep = true;
		while (StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, bFirstStep))
		{
			push_back(StepData);
			bFirstStep = false;
		}
	}

	// 生成全部的全局数据
	CreateGlobalData();

	return true;
}

//
//   将原始扫描点数据写入二进制文件。
//
bool CSlamDataSet::SaveRawScanBinary(FILE* fp, int nFileVersion, int nFrom, int nTo, bool bReverseOrder, 
	bool bSaveGlobalPosture)
{
	if (fwrite(&nFileVersion, sizeof(int), 1, fp) != 1)
		return false;

	// 依次写入各激光扫描器的参数
	if (!m_ScannerParam.SaveBinary(fp, nFileVersion))
		return false;

	if (nTo < 0)
		nTo = (int)size();

	// 如果版本在V2.00以上，接下来将直接写入总的步数
	if (nFileVersion >= 200)
	{
		int nCount = nTo - nFrom + 1;
		if (fwrite(&nCount, sizeof(int), 1, fp) != 1)
			return false;

	}

	// 写入全部的局部数据
	// 如果是正序输出
	if (!bReverseOrder)
	{
		for (int i = nFrom; i < nTo; i++)
			at(i).SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
	}
	// 如果是反序输出
	else
	{
		for (int i = nTo - 1; i >= nFrom; i--)
		{
			CSlamStepData Step = at(i);

			// 第一步的移动里程一定为(0, 0, 0)
			if (i == nTo - 1)
				Step.m_pstMoveEst.SetPosture(0, 0, 0);
			
			// 其它步的移动里程为其后一步移动里程的逆
			else
			{
				CTransform trans(at(i + 1).m_pstMoveEst);
				trans = trans.Inv();
				Step.m_pstMoveEst.GetPostureObject() = trans;	
			}
			Step.SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
		}
	}

	// 写入帧间匹配数据
	if (nFileVersion >= 200)
	{
		unsigned char ch = 'O';
		if (fwrite(&ch, sizeof(unsigned char), 1, fp) != 1)
			return false;
#if 0
		// 保存帧间匹配数据
		for (int i = 0; i < (int)size(); i++)
			if (!at(i).SaveCorrData(fp))
				return false;
#endif
	}

	return true;
}

//
//   启用新的激光器参数。
//
bool CSlamDataSet::SetScannerParam(const CScannerGroupParam& Param)
{
	m_ScannerParam = Param;

	// 重新生成全局数据
	CreateGlobalData();

	return true;
}

//
//   取得指定的世界散点。
//
CScanPoint* CSlamDataSet::GetWorldRawPoint(int nStepId, int nScannerId, int nIdxPoint)
{
	return at(nStepId).GetWorldRawPoint(nScannerId, nIdxPoint);
}

// 将两个数据进行合并
void CSlamDataSet::operator += (const CSlamDataSet& other)
{
	for (int i = 0; i < other.size(); i++)
		push_back(other[i]);
}

//
//   对数据集应用滤波规则。
//
void CSlamDataSet::ApplyFilterRules(const CScanFilterRules& Rules)
{
	// 依次处理各条规则
	for (int i = 0; i < (int)Rules.size(); i++)
	{
		const CScanFilterRule& Rule = Rules[i];

		// 依次处理各步
		for (int j = 0; j < (int)size(); j++)
		{
			// 如果步编号处于规则范围之内，则启用规则
			if (j >= Rule.m_nStartId && j <= Rule.m_nEndId)
			{
				// 如果是角度规则
				if (Rule.m_nType == 1)
					at(j).ApplyScanAngleRule(Rule.m_nScannerId, Rule.m_fParam[0], Rule.m_fParam[1]);

				// 如果是距离规则
				else if (Rule.m_nType == 2)
					at(j).ApplyScanDistRule(Rule.m_nScannerId, Rule.m_fParam[0], Rule.m_fParam[1]);
			}
		}
	}
}

//
//   判断在某一步时，指定的屏幕点是否触及某个原始点。
//
int CSlamDataSet::PointHitRawPoint(int nStepId, const CPnt& pt, float fDistGate)
{
	return at(nStepId).PointHitRawPoint(pt, fDistGate);
}

bool CSlamDataSet::MatchScans(int nStepId, int nScanId1, int nScanId2, CPosture& result)
{
	return at(nStepId).MatchScans(nScanId1, nScanId2, result);
}

//
//   根据给定的初始姿态生成全局数据。
//
void CSlamDataSet::CreateGlobalData()
{
	CPosture pstInit(0, 0, 0);
	for (int i = 0; i < (int)size(); i++)
		at(i).CreateGlobalData(pstInit, m_ScannerParam);
}

#ifdef _MFC_VER

void CSlamDataSet::Plot(int nStepId, CScreenReference& ScrnRef, CDC* pDC, COLORREF clrRawPoint,
	bool bShowScanner, bool bShowFeature, COLORREF clrFeature)
{
	at(nStepId).Plot(ScrnRef, pDC, clrRawPoint, RGB(255, 255, 0), bShowScanner, bShowFeature, clrFeature);
}

#elif defined QT_VERSION

void CSlamDataSet::Plot(int nStepId, CScreenReference& ScrnRef, QPainter* pPainter, QColor clrRawPoint,
	bool bShowScanner, bool bShowFeature, QColor clrFeature)
{
	at(nStepId).Plot(ScrnRef, pPainter, clrRawPoint, Qt::yellow, bShowScanner, bShowFeature, clrFeature);
}

#endif
