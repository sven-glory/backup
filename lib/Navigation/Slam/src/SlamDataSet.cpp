#include "stdafx.h"
#include "SlamDataSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ������ݼ���
//
void CSlamDataSet::Clear()
{
	vector<CSlamStepData>::clear();

	m_ScannerParam.clear();
}

//   װ�������ԭʼɨ��������ļ���
//   ����ֵ��
//   < 0 - ����
//     0 - �жϴ��ļ�Ϊδ���ۺϵ�ԭʼ���ݼ��ļ�����Ŀǰ�Ѷ���
//     1 - ��δ�������ݼ���ԭʼ���֣�״̬����
//     2 - �жϴ��ļ�Ϊ�ۺϺ�����ݼ��ļ�����Ŀǰԭʼ�����Ѷ���
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

	// ����汾��V2.00���ϣ���������ֱ�Ӷ����ܵĲ���
	if (m_nFileVersion >= 200)
	{
		int nStepsCount = 0;
		if (fread(&nStepsCount, sizeof(int), 1, fp) != 1)
			return -1;

		nStepsCount--;
		bool bEof = false;
		// ���ݲ�������ȫ���ľֲ�����
		for (int i = 0; i < nStepsCount; i++)
		{
			if (!StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, (i == 0)))
			{
				bEof = true;
				break;
			}
			push_back(StepData);
		}

		// �Զ���һ����־���ݣ������ļ��Ƿ����(Ҫ���־�ַ���ҪΪ'O')
		if (!bEof)
		{
			unsigned char ch;
			if (fread(&ch, sizeof(unsigned char), 1, fp) != 1 || ch != 'O')
			{
				// �ѵ��ļ�β��˵������һ��ԭʼ���ݼ��ļ�������֡��ƥ������
			}
		}
	}
	// ����ļ��汾��V2.00���£���Ҫ���ļ�β��Ϊ�����ж�����
	else
	{
		// ����ȫ���ľֲ�����
		bool bFirstStep = true;
		while (StepData.LoadRawScanBinary(fp, m_ScannerParam, m_nFileVersion, bFirstStep))
		{
			push_back(StepData);
			bFirstStep = false;
		}
	}

	// ����ȫ����ȫ������
	CreateGlobalData();

	return true;
}

//
//   ��ԭʼɨ�������д��������ļ���
//
bool CSlamDataSet::SaveRawScanBinary(FILE* fp, int nFileVersion, int nFrom, int nTo, bool bReverseOrder, 
	bool bSaveGlobalPosture)
{
	if (fwrite(&nFileVersion, sizeof(int), 1, fp) != 1)
		return false;

	// ����д�������ɨ�����Ĳ���
	if (!m_ScannerParam.SaveBinary(fp, nFileVersion))
		return false;

	if (nTo < 0)
		nTo = (int)size();

	// ����汾��V2.00���ϣ���������ֱ��д���ܵĲ���
	if (nFileVersion >= 200)
	{
		int nCount = nTo - nFrom + 1;
		if (fwrite(&nCount, sizeof(int), 1, fp) != 1)
			return false;

	}

	// д��ȫ���ľֲ�����
	// ������������
	if (!bReverseOrder)
	{
		for (int i = nFrom; i < nTo; i++)
			at(i).SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
	}
	// ����Ƿ������
	else
	{
		for (int i = nTo - 1; i >= nFrom; i--)
		{
			CSlamStepData Step = at(i);

			// ��һ�����ƶ����һ��Ϊ(0, 0, 0)
			if (i == nTo - 1)
				Step.m_pstMoveEst.SetPosture(0, 0, 0);
			
			// ���������ƶ����Ϊ���һ���ƶ���̵���
			else
			{
				CTransform trans(at(i + 1).m_pstMoveEst);
				trans = trans.Inv();
				Step.m_pstMoveEst.GetPostureObject() = trans;	
			}
			Step.SaveRawScanBinary(fp, nFileVersion, bSaveGlobalPosture);
		}
	}

	// д��֡��ƥ������
	if (nFileVersion >= 200)
	{
		unsigned char ch = 'O';
		if (fwrite(&ch, sizeof(unsigned char), 1, fp) != 1)
			return false;
#if 0
		// ����֡��ƥ������
		for (int i = 0; i < (int)size(); i++)
			if (!at(i).SaveCorrData(fp))
				return false;
#endif
	}

	return true;
}

//
//   �����µļ�����������
//
bool CSlamDataSet::SetScannerParam(const CScannerGroupParam& Param)
{
	m_ScannerParam = Param;

	// ��������ȫ������
	CreateGlobalData();

	return true;
}

//
//   ȡ��ָ��������ɢ�㡣
//
CScanPoint* CSlamDataSet::GetWorldRawPoint(int nStepId, int nScannerId, int nIdxPoint)
{
	return at(nStepId).GetWorldRawPoint(nScannerId, nIdxPoint);
}

// ���������ݽ��кϲ�
void CSlamDataSet::operator += (const CSlamDataSet& other)
{
	for (int i = 0; i < other.size(); i++)
		push_back(other[i]);
}

//
//   �����ݼ�Ӧ���˲�����
//
void CSlamDataSet::ApplyFilterRules(const CScanFilterRules& Rules)
{
	// ���δ����������
	for (int i = 0; i < (int)Rules.size(); i++)
	{
		const CScanFilterRule& Rule = Rules[i];

		// ���δ������
		for (int j = 0; j < (int)size(); j++)
		{
			// �������Ŵ��ڹ���Χ֮�ڣ������ù���
			if (j >= Rule.m_nStartId && j <= Rule.m_nEndId)
			{
				// ����ǽǶȹ���
				if (Rule.m_nType == 1)
					at(j).ApplyScanAngleRule(Rule.m_nScannerId, Rule.m_fParam[0], Rule.m_fParam[1]);

				// ����Ǿ������
				else if (Rule.m_nType == 2)
					at(j).ApplyScanDistRule(Rule.m_nScannerId, Rule.m_fParam[0], Rule.m_fParam[1]);
			}
		}
	}
}

//
//   �ж���ĳһ��ʱ��ָ������Ļ���Ƿ񴥼�ĳ��ԭʼ�㡣
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
//   ���ݸ����ĳ�ʼ��̬����ȫ�����ݡ�
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
