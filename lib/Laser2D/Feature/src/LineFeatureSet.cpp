#include "stdafx.h"
#include "misc.h"
#include "scan.h"
#include "LineFeatureSet.h"
#include "FeatureCreationParam.h"
#include "DebugTrace.h"
#include "assert.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

extern CFeatureCreationParam FeatureCreationParam;

///////////////////////////////////////////////////////////////////////////////

/*
** Calculates a line in the form (n1, n2) * (x, y) + c = 0,
** so that the leastsquare sum of the corresponding scan points
** is minimized.
** sp is an array of scan points, num specifies the number of scan points,
** n1, n2 and c are the result values.
*/
bool RegressionLine(const CScanPoint *sp, long num, CLineBase& ln);

/*
** Calculates variance of distance from a set of scan points
** given in (sp, num) to a line given in (n1, n2, c).
*/
float LineDeviation(CScanPoint *sp, long num, CLineBase& lb);

int MinPointsOnLine(float r, float fMinPointsOnLineRatio)
{
	int n = (int)(fMinPointsOnLineRatio / r);
	if (n < 20)
		n = 20;
	return n;
}

///////////////////////////////////////////////////////////////////////////////
//   ��CLineFeatureSet�����ʵ�֡�

//
//   ���캯����
//
CLineFeatureSet::CLineFeatureSet(int nNum)
{
	Clear();
	m_rect.Clear();
}

//
//   �����������캯����
//
CLineFeatureSet::CLineFeatureSet(const CLineFeatureSet& Obj, bool bFilterDisabled)
{
	Clear();

	for (int i = 0; i < (int)Obj.size(); i++)
	{
		// ������Ҫ���˳���Щ����ֹ����
		if (bFilterDisabled && !Obj.at(i)->IsEnabled())
			continue;

		CLineFeature* p = Obj.at(i)->Duplicate();
		if (p == NULL)
			assert(false);
		else
			push_back(p);
	}

	m_Param = Obj.m_Param;    // ֱ�����ɲ���
	m_pstScanner = Obj.m_pstScanner;           // ����ͷ�ο���̬
	
	UpdateCoveringRect();
}

//
//   �������������ͷ������ѷ�����ڴ档
//
CLineFeatureSet::~CLineFeatureSet()
{
	Clear();
}

//
//   ���ء�=����������
//
void CLineFeatureSet::operator = (const CLineFeatureSet& Obj)
{
	Clear();

	m_rect = Obj.GetCoveringRect();
	m_Param = Obj.m_Param;    // ֱ�����ɲ���
	m_pstScanner = Obj.m_pstScanner;           // ����ͷ�ο���̬

	for (int i = 0; i < (int)Obj.size(); i++)
	{
		CLineFeature* p = Obj.at(i)->Duplicate();
		if (p == NULL)
			assert(false);
		else
			push_back(p);
	}
}

//
//   ����ֱ���������ͷ���ռ䡣
//
CLineFeature* CLineFeatureSet::NewLineFeature(int nSubType)
{
	switch (nSubType)
	{
	case GENERIC_LINE_FEATURE:
		return new CLineFeature;

	case SINGLE_SIDED_LINE_FEATURE:
//		return new CSingleSidedLineFeature;

	default:
		return NULL;
	}
}

//
//   ����ֱ�����������ɲ�����
//
void CLineFeatureSet::SetCreationParam(CLineFeatureCreationParam* pParam) 
{
	if (pParam != NULL)
		m_Param = *pParam; 
}

//
//   �������ṩ��ɨ�赽��ֱ����������ֱ���������ϡ�
//
//   ע�⣺����ֱ�����������Ϊ���ز�������ֱ������(���۲���̬Ϊ(0, 0, 0))
//
bool CLineFeatureSet::CreateFromLocalLines(int nNum, CLineFeature* pLineData)
{
	Clear();

	// ֱ�����ݱ����ṩ
	if (pLineData == NULL)
		return false;

	for (int i = 0; i < nNum; i++)
	{
		CLineFeature* p = pLineData[i].Duplicate();
		if (p == NULL)
			assert(false);
		else
			push_back(p);
	}

	// ����ֱ�������Ĺ۲ⷽ��
	SetDetectPosture(CPosture(0, 0, 0));

	// ����߽�ֵ
	UpdateCoveringRect();
	return true;
}

//
//   ����ɨ��⵽��Щֱ������ʱ�ļ���ͷ��̬���Ա�������ֱ�������Ĺ۲ⷽ��
//
void CLineFeatureSet::SetDetectPosture(const CPosture& pstDetect)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->SetDetectPosture(pstDetect);
}

//
//   ��һ��ɨ�輯�г�ȡ��������ֱ�߶Ρ�
//
bool CLineFeatureSet::CreateFromScan(const CScan& scan)
{
	Clear();

	const float fac = 5.0;
	const float div = 1.0 / fac;

	// ���Ʋ���Ϊ��
	if (scan.m_nCount == 0)
		return false;

	m_pstScanner = scan.m_poseScanner;

	CScanPoint* sp = scan.m_pPoints;
	sp[0].m_nLineID = -1;

	long i, start = 0;
	float last_dist = 0.0; 

	for (i = 1; i < scan.m_nCount; i++)
	{
		// ���㵱ǰ�㵽��һ��֮��ľ���
		float d = sp[i-1].DistanceTo(sp[i]);

		// ��������ȡ10mm
		float dist = MAX(d, 0.01f);
		float rel = last_dist / dist;

		// �ȼٶ��õ�����Ӧ��ֱ�߲�����
		sp[i].m_nLineID = -1;

		// ��ǰ��ļ���
		float r = sp[i].r;

		// ֱ�߶����������С����
		int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

		// �ж����������Ƿ����
		bool bPointsDistTooBig = (dist > m_Param.fMaxDistPointToPoint * r);

		// �ж�ɨ����Ƿ�̫������(���������)
		bool bPointsDistChangeTooMuch = (i > start + 1 && (rel > fac || rel < div));

		// ���������̫�󣬻��ߵ�����仯����(������)
		if (bPointsDistTooBig || bPointsDistChangeTooMuch)
		{
			// �����һ��������������һ���߶Σ����Է��ѳ�һ�����߶�
			if (i - start >= m_Param.nMinPointsOnLine)
				SplitPoints(sp, start, i - 1);

			// ������һ�ε���ʼ��λ��
			start = i;
		}

		// �������������ľ�ֵ
		if (i <= start + 1)
			last_dist = dist;
		else
			last_dist = (last_dist + dist) / 2;
	}

	// ֱ�߶����������С����
	float r = sp[i].r;
	int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

	// �����start��i�������ĵ��������һ����С�߶���������������Է��ѳ�һ��ֱ��
	if (i - start >= nMinPointsOnLine)
		SplitPoints(sp, start, i - 1);

	// �ϲ����ߵ�ֱ������
	LineScanMergeLines(&((CScan&)scan), NULL);

	// ɾ����Щɨ���̫С��ֱ������
	RemoveBadLines(m_pstScanner, scan);

	for (i = 0; i < (int)size(); i++)
	{
		at(i)->m_nId = i+1;
		at(i)->ComputeParam();

		// �����Ϸ�Χ
		CRange range(0, at(i)->Length());
		at(i)->m_Ranges.push_back(range);
	}

	// ����ֱ�������Ĺ۲ⷽ��
	SetDetectPosture(m_pstScanner);

	// ����߽�ֵ
	UpdateCoveringRect();

	return true;
}

//
//   ɾ����Щɨ��ǲ��ѵ�ֱ��������
//
void CLineFeatureSet::RemoveBadLines(const CPosture& pstScanner, const CScan& scan)
{
	CPnt ptScanner = m_pstScanner;

	for (int i = 0; i < (int)size(); i++)
	{
		CLineFeature* pLineFeature = at(i);

		// ȡֱ���������е�
		CPnt ptMid = pLineFeature->GetMidpoint();

		// ����ɨ����
		CLine ln1(ptScanner, ptMid);
		
		// ����ֱ��������ɨ����֮��ļн�
		CAngle angDiff = ln1.AngleToUndirectionalLine(*pLineFeature);
		float fAngDiff = angDiff.m_fRad;
		if (angDiff > PI / 2)
			fAngDiff = PI - fAngDiff;

		// �������н�С��ָ��������ֵ������Ϊ��ֱ���������ϸ�
		bool bBad = fAngDiff < m_Param.fMinScanToLineAngle;

		// ����нǺϸ�����˶Ե�����
		if (!bBad)
		{
			float fDistLimit = (float)(m_Param.fMaxDistPointToPoint * ln1.Length() / sin(fAngDiff));

			CScanPoint* sp = scan.m_pPoints;
			for (long j = pLineFeature->m_lStart; j < pLineFeature->m_lEnd - 1; j++)
			{
				// ȡ�����ڵ�����ɨ���
				CPnt pt1 = sp[j].GetPntObject();
				CPnt pt2 = sp[j + 1].GetPntObject();

				// ��������������ֱ�������ϵ�ͶӰ��
				CPnt ptFoot1, ptFoot2;
				pLineFeature->DistanceToPoint(false, pt1, NULL, &ptFoot1);
				pLineFeature->DistanceToPoint(false, pt2, NULL, &ptFoot2);

				// ��������ͶӰ��֮��ľ���
				float d = ptFoot1.DistanceTo(ptFoot2);

				// �����������ɨ���֮��ľ�����ڼ���ֵ��˵����ֱ���������ϸ�
				if (d > fDistLimit)
				{
					bBad = true;
					break;
				}
			}
		}

		// ɾ�����ϸ��ֱ������
		if (bBad)
		{
			delete at(i);
			erase(begin() + i);
			i--;
		}
	}
}

//
//   ���ݵ�ǰ��̬�����ɨ��뾶��ֱ��ģ��������ֱ���������ϡ�
//
bool CLineFeatureSet::CreateFromWorldLines(CPosture& pst, float fMaxRange, int nNumOfLines, 
												  CLine* pLines)
{
	Clear();

	// ����һ������ΧԲ��
	CCircle RangeCircle(pst.GetPntObject(), fMaxRange);

	// �����Բ��ȡ����ֱ�ߣ��õ���ֱ�߽������뵽ֱ������������
	int nResultCount = 0;
	CLineFeature* pNewLine = new CLineFeature;

	for (int i = 0; i < nNumOfLines; i++)
	{
		// ����м��е����ȴ���300mm���߶�
		if (RangeCircle.CutLine(pLines[i], *pNewLine) && (pNewLine->Length() > 300))       // 0.3�ף���λ����!!
		{
			push_back(pNewLine);
		}
	}

	// ͨ�����ù۲���̬�������ֱ����������Ч����
	SetDetectPosture(pst);

	// ������������(�˾ٽ����´洢�ռ����ж���δ�õĲ���)
//	m_nCount = nResultCount;

	// ����߽�ֵ
	UpdateCoveringRect();

	return true;
}

//
//   ��������ڵ����е�ֱ�ߡ�
//
void CLineFeatureSet::Clear()
{
	for (int i = 0; i < (int)size(); i++)
		delete at(i);

	clear();
	m_rect.Clear();
}

//
//   �����ڴ沢���Ƶ�ǰ�������ݣ������¸��ƵĶ���ָ�롣
//
CLineFeatureSet* CLineFeatureSet::Duplicate()
{
	// Ϊ�¸����ĸ�ֱ�߶η���ռ�
	CLineFeatureSet *copy = new CLineFeatureSet;

	if (copy == NULL)
		return NULL;

	*copy = *this;
	return copy;
}

//
//   ������һ��ֱ�߶μ������ֱ�߶μ��С�
//
bool CLineFeatureSet::Merge(const CLineFeatureSet& LineScan)
{
	for (int i = 0; i < (int)LineScan.size(); i++)
	{
		CLineFeature* p = LineScan.at(i)->Duplicate();
		if (p == NULL)
			return false;
		else
		{
			push_back(p);
			m_rect += *p;
		}
	}


	return true;
}

//
//   ����һ��ֱ��������
//   (Ŀǰ�˺���Ч�ʺܵͣ���ҪƵ���ͷ�/�����ڴ棬�����Ľ�)
//
bool CLineFeatureSet::Add(const CLineFeature& LineFeature)
{
	CLineFeature* p = LineFeature.Duplicate();
	if (p == NULL)
		return false;
	else
	{
		push_back(p);
		m_rect += *p;
	}

	return true;
}

//
//   ͨ���ϲ����ߵ��߶����򻯴�ֱ�߶μ��ϡ�
//
//   ˵�������������ص����򵫹��ߵ��߶�֮�����С����С��fMaxGapBetweenLinesʱ��
//   ���Խ��������߶ν��кϲ�(����Ϊ��������С�����Ժ���)��
//
bool CLineFeatureSet::Simplify(float fMaxGapBetweenLines)
{
	bool bChange;
	int i, j;

	// ������һ���߶μ��ĸ�������Ϊÿ���߶α����Ƿ����á�
	CLineFeatureSet* pScan1 = Duplicate();
	bool* pUse = new bool[size()];

	do {
		// �ȱ��������Ӧ����
		for (i = 0; i < (int)pScan1->size(); i++)
			pUse[i] = true;

		// �ٽ�pScan1����һ��(�õ�pScan2)����Ϊ�Ƚ�֮��
		CLineFeatureSet* pScan2 = pScan1->Duplicate();
		bChange = false;

		// ���Զ����е��߶ν�������ϲ�
		for (i = 0; i < (int)pScan1->size(); i++)
		{
			// ������Щ�ѱ����������á����߶�
			if (!pUse[i])
				continue;

			for (j = i + 1; j < (int)pScan1->size(); j++)
			{
				//	if (!pUse[j])
				//	  continue;

				// ����ϲ��ɹ���������ڶ����߶�Ϊ�������á�
				if (pScan1->at(i)->ColinearMerge(*pScan1->at(j), m_Param.fMaxLineMergeAngle, 
					m_Param.fMaxLineMergeDist, fMaxGapBetweenLines))
				{
					pUse[j] = false;
					bChange = true;
				}
				else if (pScan1->at(j)->ColinearMerge(*pScan1->at(i), m_Param.fMaxLineMergeAngle, 
					m_Param.fMaxLineMergeDist, fMaxGapBetweenLines))
				{
					pUse[i] = false;
					bChange = true;
				}
			}
		}

		// �����С��ն�����������С���ѹ����ʹ֮��Ϊ�������е�����(�����г�Ա�������õ�)
		for (int j = pScan1->size() - 1; j >= 0; j--)
		{
			if (!pUse[j])
			{
				delete pScan1->at(j);
				pScan1->erase(pScan1->begin() + j);
			}
		}

		delete pScan2;                 // �ͷ�pScan2
	} while (bChange);                // һֱ���������ٺϲ�

	Clear();
	*this = *pScan1;

	delete pScan1;
	delete []pUse;

	return true;
}

//
//   �ڼ������ҵ����й��ߵ������������з����¼��
//   ����ֵ��������š�
//
int CLineFeatureSet::FindColinearGroups(float fMaxAngDiff, float fMaxDistDiff)
{
	// �Ƚ����з����־��Ϊ-1
	for (int i = 0; i < (int)size(); i++)
	{
		CLineFeature* pLine = at(i);
		pLine->m_nId = -1;
	}

	// ���������ֱ�߶ΰ��չ���������з���
	int nNextGroupId = 0;
	for (int i = 0; i < (int)size(); i++)
	{
		// ȡ��һ��ֱ������
		CLineFeature* pLine1 = at(i); 

		// �����ѷ��������
//		if (Line1.m_nGroupId >= 0)
		if (pLine1->m_nId >= 0)
			continue;
		else
			pLine1->m_nId = nNextGroupId++;
		//Line1.m_nGroupId = nNextGroupId++;

		for (int j = i + 1; j < (int)size(); j++)
		{
			// ȡ�ڶ���ֱ������
			CLineFeature* pLine2 = at(j);

			// �����ѷ��������
//			if (Line2.m_nGroupId >= 0)
			if (pLine2->m_nId >= 0)
				continue;

			// ��������������ߣ�����Ϊͬһ��
			if (pLine1->IsColinearWith(*pLine2, fMaxAngDiff, fMaxDistDiff))
				pLine2->m_nId = pLine1->m_nId;
//			Line2.m_nGroupId = Line1.m_nGroupId;
		}
	}

	return nNextGroupId;
}

//
//   ��Թ��ߵ�������ͨ������ߵ�������ʽ���кϲ���
//
bool CLineFeatureSet::ColinearSimplify(float fMaxAngDiff, float fMaxDistDiff)
{
	// �Ȱ����Ƿ��߽��з���
	int nMaxGroupId = FindColinearGroups(fMaxAngDiff, fMaxDistDiff);

	// �Զ���ߵķ�ʽ����������������
	CLineFeatureSet setNew;

	for (int i = 0; i < nMaxGroupId; i++)
	{
		// ����һ����ʱ����
		CLineFeatureSet temp;

		// �������з����ʶΪi���������������Ǽ��뵽��ʱ������
		for (int j = 0; j < (int)size(); j++)
		{
			CLineFeature* pLine = at(j);

			// �ҵ��������ڵ�i�������
//			if (Line.m_nGroupId == i)
			if (pLine->m_nId == i)
				temp.Add(*pLine);
		}

		// �Դ���ʱ���Ͻ����Ż�����
		temp.ColinearRectify();

		// ������뵽�¼�����
		setNew.Merge(temp);
	}

	*this = setNew;
	return true;
}

//
//   �Ż����������顣
//
CLineFeatureSet* CLineFeatureSet::Optimize()
{
	CLineFeatureSet* pNew = Duplicate();
	pNew->ColinearSimplify(CAngle::ToRadian(5), 0.08f);
	return pNew;
}

//
//   �������������й��ߵ���
//
bool CLineFeatureSet::ColinearRectify()
{
	if (size() < 2)
		return true;

	// ����ֱ�߶�����ռ�
	CLine* pLines = new CLine[size()];
	for (int i = 0; i < (int)size(); i++)
		pLines[i] = *at(i);

	// ���ɵȼ۶����
	CMultiSegLine MultiLine;
	MultiLine.Create(pLines, size());

	// ����Ψһһ������
	CLineFeature Feature;
	Feature.Create(MultiLine);
//	Feature.m_nGroupId = at(0).m_nGroupId;
	Feature.m_nId = at(0)->m_nId;

	// ����������ݣ��ٰ���Ψһһ���������뼯����
	Clear();
	Add(Feature);

	delete[]pLines;

	return true;
}

//
//   ɾ��ָ�����߶Ρ�
//
bool CLineFeatureSet::DeleteAt(int nIdx)
{
	if (nIdx >= (int)size() - 1 || nIdx < 0)
		return false;

	delete at(nIdx);
	erase(begin() + nIdx);

	UpdateCoveringRect();
	return true;
}

//
//   ��������ֱ�߶ε��ܳ��ȡ�
//
float CLineFeatureSet::TotalLength()
{
	float fSum = 0;
	for (long i = 0; i < (int)size(); i++)
		fSum += at(i)->m_fTotalLen;

	return fSum;
}

//
//   ��������и������ָ��ֱ��(n1*x + n2*y + c = 0)������Զ�ľ��롣
//
//   ˵������(x0, y0)��ֱ��(n1*x + n2*y + c = 0)���빫ʽΪ��
//     d = abs(n1*x0 + n2*y0 + c)/sqrt(n1*n1 + n2*n2)
//
//   ����ֵ��
//     ����: ������Զ������
//     pMaxDist: ָ����Զ����ֵ��ָ��
//
float FindMaxDist(const CScanPoint *sp, long num, CLineBase& ln, long* pMaxDistIdx)
{
	if (pMaxDistIdx != NULL)
		*pMaxDistIdx = 0;

	float fMaxDist = 0;
	for (long i = 0; i < num; i++)
	{
		// ����㵽ֱ�ߵľ���
		float d = ln.DistanceToPoint(sp[i]);

		// ʼ�ձ���������ֵ
		if (d > fMaxDist)
		{
			fMaxDist = d;            // ������Զ����ֵ

			if (pMaxDistIdx != NULL)
				*pMaxDistIdx = i;     // ��Զ���������Ӧ�����
		}
	}

	return fMaxDist;
}

//
//   ����һ���Ƿ�����ҵ�һ�������߶εĶ˵�����ġ��á��ϵ㣬�����������ƽ�е�ǽ�����ֱ����ȡЧ����
//   ע�⣺
//      ��(x0, y0)��ֱ��  n1*x +n2*y + c = 0 �ľ��빫ʽΪ��
//	     d = abs(n1 * x0 + n2 * y0 + c)/sqrt(n1*n1 + n2*n2)
//
static long RefineBreakPoint(const CScanPoint *sp, long num, long brk, CLineBase& lb)
{
	// ȡ�õ�ǰ�ϵ�����
	CPnt pt(sp[brk]);

	// ����öϵ㵽ֱ�ߵľ���
	float fMaxDist = lb.DistanceToPoint(pt);

	// ���㡰��̾���1��- ������ľ����15����
	float fMinD1 = fMaxDist - MAX_SIGMA / 2.0;

	// ���㡰��̾���2��- Ϊ�ϵ㴦�����80%
	float fMinD2 = fMaxDist * 0.8;

	// ȡ������������Ľϴ�ֵ
	float fMinD = MAX(fMinD1, fMinD2);

	long end = num - 1;

	// ����öϵ㵽���ľ���
	float fStartD = pt.DistanceTo(sp[0]);

	// ����öϵ㵽�յ�ľ���
	float fEndD = pt.DistanceTo(sp[end]);

	// ��������Ͻ�
	if (fStartD < fEndD)
	{
		// ��������㵽�öϵ㴦��û�о���ֱ��ƫ�����Ҫ��ġ���С���롱�ĵ�
		for (long i = 1; i < brk; i++)
			if (lb.DistanceToPoint(sp[i]) > fMinD)
				return i;
	}
	// ������յ�Ͻ�
	else
	{
		// �������յ㵽�öϵ㴦��û�о���ֱ��ƫ�����ĵ�
		for (long i = end - 1; i > brk; i--)
			if (lb.DistanceToPoint(sp[0]) > fMinD)
				return i;
	}

	return brk;
}

//
//  �ж�ɨ���������Ƿ���������������ƽֱ��
//
static bool IsZigZag(const CScanPoint *sp, long num, long brk, CLineBase& lb)
{
	static const float eps = 0.01;

	long i;
	long lNegCount = 0;     // �����������
	long lPosCount = 0;     // �����������

	// �ȷ�����㵽�ϵ�֮������е�
	for (i = 0; i < brk; i++)
	{
		// ����㵽ֱ�ߵľ���
		float val = lb.DistanceToPoint(sp[i]); //sp[i].x * n1 + sp[i].y * n2 + c;

		// ������Щ����ֱ��ƫ��ϴ�ĵ������(������ƫ��͸���ƫ���������)
		if (val < -eps)
			lNegCount++;
		else if (val > eps)
			lPosCount++;
	}

	// ������ƫ�롢����ƫ�����ֵ�е�һ������һ����2�����󣬿���Ϊ�ǡ�����������
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return true;

	// ���¼���
	lNegCount = lPosCount = 0;

	// �ٷ����ϵ㵽�յ�֮������е㣬ԭ��ͬ��
	for (i = brk + 1; i < num; i++)
	{
		// ����㵽ֱ�ߵľ���
		float val = lb.DistanceToPoint(sp[i]); //sp[i].x * n1 + sp[i].y * n2 + c;

		// ������Щ����ֱ��ƫ��ϴ�ĵ������(������ƫ��͸���ƫ���������)
		if (val < -eps)
			lNegCount++;
		else if (val > eps)
			lPosCount++;
	}

	// ������ƫ�롢����ƫ�����ֵ�е�һ������һ����2�����󣬿���Ϊ�ǡ�����������
	if ((lNegCount >= 3 && lNegCount > 2 * lPosCount) || (lPosCount >= 3 && lPosCount > 2 * lNegCount))
		return true;

	return false;
}

//
//   ��λ��ָ����ŵ�ɨ����е�ֱ�߽����Ż����ҵ�׼ȷ�������յ㡣
//   ע�⣺����ҵ���ֱ�߶���ʵ�п��ܻ���Ҫ�����ٴη��ѣ�������ȡ�����ֱ�߶Ρ�
//
//   ����ֵ��
//      true - �Ż��ɹ����õ����µ�lStart, lEndֵ
//      false - ��������ɨ�����ȡ������Ч��ֱ��
//
bool RefineLineBetween(CScanPoint* sp, long& lStart, long& lEnd, float& sigma2, CLineFeatureCreationParam* pParam)
{
	const float max_sigma2 = MAX_SIGMA * MAX_SIGMA;
	const float thresh1 = max_sigma2 / 2.0;               // ����ֵ1

	long num_points = lEnd + 1 - lStart;

	// ����ɵ�ǰֱ��
	CPnt ptStart(sp[lStart]), ptEnd(sp[lEnd]);
	CLine ln(ptStart, ptEnd);

	// ������Ҫȷ��������Щ������ֱ�ߵı�׼����(n1, n2, c)
	if (!RegressionLine(sp + lStart, num_points, ln))
		return false;

	// ȷ����ֱ�ߵ���ʼ��(��������˳�����)
	long i;
	for (i = lStart; i < lEnd; i++)
	{
		float val = ln.DistanceToPoint(sp[i]);
		val = val * val;
		sigma2 += val;

		if (val < thresh1)
			break;		  // �õ��Ѻܿ���ֱ�ߣ��ҵ���ʼ��
	}

	long lNewStart = i;       // ��ʼ�����

	// ��ǰ��ʼ��ļ���
	float r = sp[lNewStart].r;
	int nMinPointsOnLine = MinPointsOnLine(r, pParam->nMinPointsOnLine);

	// ȷ����ֱ�ߵ���ֹ��(�ط������������)
	for (i = lEnd; i > lNewStart; i--)
	{
		float val = ln.DistanceToPoint(sp[i]);
		val = val * val;
		sigma2 += val;

		if (val < thresh1)
			break;		  // �õ��Ѻܿ���ֱ�ߣ��ҵ���ֹ��
	}

	long lNewEnd = i;         // ��ֹ�����

	// ����������ĵ�̫�٣����޷�����߶�
	if (lNewEnd + 1 - lNewStart < nMinPointsOnLine - 2)
		return false;

	return true;
}

//
//   ��ָ����ɨ�����Ѳ���ȡ��ֱ�߶Ρ�
//
void CLineFeatureSet::SplitPoints(CScanPoint *sp, long lStart, long lEnd)
{
	bool refine_break_point = true;
	long num_points = lEnd + 1 - lStart;

	// ����ɵ�ǰֱ��
	CPnt ptStart(sp[lStart]), ptEnd(sp[lEnd]);
	CLine ln(ptStart, ptEnd);

	// �ж�ֱ�߳����Ƿ�̫��
	if (ln.Length() < m_Param.fMinLineLength)
		return;		 // ̫��

	// ��lStart�㿪ʼ���������ֱ��(n1*x + n2*y + c = 0)��Զ�����ţ����������Զ����(����fMaxDist��)
	long lMaxDistIdx;
	float fMaxDist = FindMaxDist(&sp[lStart], num_points, ln, &lMaxDistIdx);
	long brk = lStart + lMaxDistIdx;

	// ���������õ���Զ����������������룬��ֱ����Ҫ����
	bool bSplit = (fMaxDist > m_Param.fMaxDistPointToLine);

	// �����ֱ�߲��ط���
	if (!bSplit)
	{
		const float max_sigma2 = MAX_SIGMA * MAX_SIGMA;
		const float thresh2 = max_sigma2 * 16.0;              // ����ֵ2

		long lNewStart = lStart, lNewEnd = lEnd;
		float sigma2 = 0;

		// ����������ĵ�̫�٣����޷�����߶Σ���Ҫ���׷���
		if (!RefineLineBetween(sp, lNewStart, lNewEnd, sigma2, &m_Param))
			bSplit = true;
		else
		{
			// ��ͼ��ʣ�µĶ����ҵ�һ���µľ����Զ�Ķϵ�
			long newbrk = -1;

			for (long i = lNewStart + 1; i < lNewEnd; i++)
			{
				float val = ln.DistanceToPoint(sp[i]);
				val = val * val;
				sigma2 += val;

				if (val > thresh2 && newbrk < 0)
					newbrk = i;
			}

			sigma2 /= num_points;
			if (sigma2 > max_sigma2)
				bSplit = true;

			// ��������ɷ�
			else if (newbrk >= 0)
			{
				brk = newbrk;
				refine_break_point = false;
				bSplit = true;
			}

			// ���򣬲������ٷ���
			else
			{
				// ����һ���߶�
				long new_num = lNewEnd + 1 - lNewStart;

				if (num_points - new_num > 3)
					RegressionLine(&sp[lNewStart], new_num, ln);

				// ��ǰ��ʼ��ļ���
				float r = sp[lNewStart].r;
				int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

				// ���λ��lNewStart֮ǰ�ĵ������Ҳ��һ���߶Σ�Ӧ����Щ��Ҳ���з���
				if (lNewStart + 1 - lStart >= nMinPointsOnLine)
					SplitPoints(sp, lStart, lNewStart);

				CPnt ptStart1(sp[lNewStart]), ptEnd1(sp[lNewEnd]);
				CLine ln1(ptStart1, ptEnd1);

				// �����߶γ���
				float fDist = ln1.Length();
				float fSigmaRatio = sqrt(sigma2) / fDist;

				// ����߶εĳ��ȳ�������С���ȡ�����ƽ���������С��ָ�����ޣ���ȷ���ҵ�һ����Ч�߶�
				if (fDist >= m_Param.fMinLineLength && fSigmaRatio < MAX_SIGMA_RATIO)
				{
					// ����Щ���ڸ�ֱ���ϵĵ�����ֱ�ߵ����
					for (long i = lNewStart; i <= lNewEnd; i++)
						sp[i].m_nLineID = (int)size();

					CLineFeature* pLine = new CLineFeature(ptStart1, ptEnd1);
					pLine->m_lStart = lNewStart;
					pLine->m_lEnd = lNewEnd;
					pLine->m_fSigma2 = sigma2;

					push_back(pLine);
				}

				// ���λ��lNewEnd֮��ĵ������Ҳ��һ���߶Σ�Ӧ����Щ��Ҳ���з���
				if (lEnd + 1 - lNewEnd >= nMinPointsOnLine)
					SplitPoints(sp, lNewEnd, lEnd);
			}
		}
	}

	// �����Ҫ��һ������
	if (bSplit)
	{
		if (refine_break_point)
		{
			brk = lStart + RefineBreakPoint(&sp[lStart], lEnd + 1 - lStart, brk - lStart, ln);
		}

		// ��ǰ����
		float r = sp[lStart].r;
		int nMinPointsOnLine = MinPointsOnLine(r, m_Param.nMinPointsOnLine);

		// �������ʼ�㵽�ϵ㴦���н϶�㣬����һ����Ҫ��������
		if (brk + 1 - lStart >= nMinPointsOnLine)
			SplitPoints(sp, lStart, brk);

		// ����Ӷϵ㵽��ʼ�㴦���н϶�㣬����һ����Ҫ��������
		if (lEnd + 1 - brk >= nMinPointsOnLine)
			SplitPoints(sp, brk, lEnd);
	}
}

///////////////////////////////////////////////////////////////////////////////

#define LINE_MERGE_MAX_SIGMA2 (25.0 * 25.0)
#define LINE_MERGE_MAX_SIGMA2_FAC 2.0

#define MIN_COS_ALPHA       cos(DEG2RAD(30.0))

//
//   ��ָ���ĵ��ƶν������Իع�ֱ����ϡ�
//   ˵����
//     sp  - �������黺����ָ��
//     num - �������
//
bool RegressionLine(const CScanPoint *sp, long num, CLineBase& lb)
{
	float xm = 0.0, ym = 0.0, xxm = 0.0, yym = 0.0, xym = 0.0;
	float a, dx, dy;
	long i;

	if (num <= 1)
		return false;

	for (i = 0; i < num; i++)
	{
		float x = sp[i].x;
		float y = sp[i].y;

		xm += x;
		ym += y;
		xxm += x*x;
		yym += y*y;
		xym += x*y;
	}

	a = 0.5f * atan2((float)(-2.0*(xym - xm*ym / num)), (float)(yym - ym*ym / num - xxm + xm*xm / num));
	float n1 = cos(a);
	float n2 = sin(a);
	dx = sp[num - 1].x - sp[0].x;
	dy = sp[num - 1].y - sp[0].y;

	if (dx * n2 - dy * n1 > 0.0)
	{
		n1 = -n1;
		n2 = -n2;
	}
	float c = -(n1 * xm + n2 * ym) / num;
	
	lb.DirectCreate(n1, n2, c);
	return true;
}

float LineDeviation(CScanPoint *sp, long num, CLineBase& lb)
{
	float sum = 0.0, sigma2;
	long i;

	for (i = 0; i < num; i++)
	{
		float val = lb.DistanceToPoint(sp[i]);
		sum += val * val;
	}

	sigma2 = sum / (float)num;
	return sigma2;
}

//
//   ����Щ�������������߶κϲ���
//
void CLineFeatureSet::LineScanMergeLines(CScan *scan, long *lineNum)
{
	static const float fac = 1.0;

	long i, j;

	if (scan == NULL)
		return;

	CScanPoint* tmp = (CScanPoint *)SSmalloc(scan->m_nCount * sizeof(scan->m_pPoints[0]));
	if (tmp == NULL)
		return;

	CScanPoint* sp = scan->m_pPoints;
	long totalLines = size(); //m_nCount;

	bool change = true;
	while (change)
	{
		change = false;

		// �����е�ֱ�߶������Աȣ����Ƿ���������
		for (i = 0; i < (int)size() - 1; i++)
			for (j = i + 1; j < (int)size(); j++)
			{
				// ����ȡ�������߶�
				CLineFeature *ln1 = at(i);
				CLineFeature *ln2 = at(j);

				// ��������ֱ�ߵļнǣ�
				float cosa = ln1->a * ln2->a + ln1->b * ln2->b;

				float d1, d2, lambda1, lambda2;
				long num1, num2, k, l, l1, l2;
				float n1, n2, c;
				float x1, y1, x2, y2;
				float val, dist, appdist, sigma2, maxSigma2;

				if (cosa < MIN_COS_ALPHA)
					continue;

				// ����߶�1�ĳ���С��ֱ��2���������߶ζԻ�(����ǣ��߶�1�������߶�2)
				if (ln1->m_fTotalLen < ln2->m_fTotalLen)
				{
					CLineFeature *h = ln1;
					ln1 = ln2;
					ln2 = h;
				}

				d1 = ln1->DistanceToPoint(false, ln2->m_ptStart, &lambda1);
				d2 = ln1->DistanceToPoint(false, ln2->m_ptEnd, &lambda2);

				if (d1 > fac * m_Param.fMaxDistPointToLine)
					continue;

				if (d2 > fac * m_Param.fMaxDistPointToLine)
					continue;

				if ((lambda1 < 0.0 || lambda1 > 1.0) && (lambda2 < 0.0 || lambda2 > 1.0))
					continue;

				k = 0;
				l1 = ln1->m_lStart;
				l2 = ln2->m_lStart;
				n1 = ln1->a;
				n2 = ln1->b;

				while (l1 <= ln1->m_lEnd && l2 <= ln2->m_lEnd)
				{
					float x1 = sp[l1].x * n2 - sp[l1].y * n1;
					float x2 = sp[l2].x * n2 - sp[l2].y * n1;

					if (x1 > x2)
						tmp[k++] = sp[l1++];
					else
						tmp[k++] = sp[l2++];
				}

				while (l1 <= ln1->m_lEnd)
					tmp[k++] = sp[l1++];

				while (l2 <= ln2->m_lEnd)
					tmp[k++] = sp[l2++];

				CLineBase lb;
				if (!RegressionLine(tmp, k, lb))
					continue;

				n1 = lb.a;
				n2 = lb.b;
				c = lb.c;

				maxSigma2 = LINE_MERGE_MAX_SIGMA2_FAC * (ln1->m_fSigma2 + ln2->m_fSigma2);
				maxSigma2 = MIN(maxSigma2, LINE_MERGE_MAX_SIGMA2);


				if ((lambda1 >= 0.0 && 1.0 - lambda1 > lambda2 - 1.0) || (lambda2 <= 1.0 && lambda2 > -lambda1))
					maxSigma2 *= 20.0;
				else
				{
					// ����ɨ�������tmp��ֱ��lb��Զ��һ��(�����brk��)
					long brk;
					FindMaxDist(tmp, k, lb, &brk);
					brk = RefineBreakPoint(tmp, k, brk, lb);

					// ���ֱ�߶��ǡ�����ŤŤ����
					if (IsZigZag(tmp, k, brk, lb))
						continue;
				}

				num1 = ln1->m_lEnd + 1 - ln1->m_lStart;

				sigma2 = LineDeviation(&sp[ln1->m_lStart], num1, lb);

				if (sigma2 > maxSigma2)
					continue;

				num2 = ln2->m_lEnd + 1 - ln2->m_lStart;
				sigma2 = LineDeviation(&sp[ln2->m_lStart], num2, lb);

				if (sigma2 > maxSigma2)
					continue;

				sigma2 = LineDeviation(tmp, k, lb);
				if (sigma2 > maxSigma2)
					continue;

				// �������߶κϲ�
				if (ln1->m_lStart > ln2->m_lStart)
				{
					CLineFeature *h = ln1;
					ln1 = ln2;
					ln2 = h;
					l = 0;
				}

				if (ln1->m_lEnd + 1 < ln2->m_lStart)
				{
					long num2 = ln2->m_lEnd + 1 - ln2->m_lStart;
					long src = ln1->m_lEnd + 1;
					long dst = src + num2;
					long size0 = ln2->m_lStart - src;

					memmove(&sp[dst], &sp[src], size0 * sizeof(*sp));

					for (l = 0; l < (long)size(); l++)
					{
						CLineFeature *ln3 = at(l);
						if (ln3->m_lStart >= ln1->m_lEnd && ln3->m_lEnd <= ln2->m_lStart)
						{
							if (ln3->m_lStart == ln1->m_lEnd)
								ln3->m_lStart++;

							if (ln3->m_lEnd == ln2->m_lStart)
								ln3->m_lEnd--;

							ln3->m_lStart += num2;
							ln3->m_lEnd += num2;
						}
					}
					ln2->m_lStart = src;
					ln2->m_lEnd = src + num2 - 1;
				}

				x1 = tmp[0].x;
				y1 = tmp[0].y;
				x2 = tmp[k - 1].x;
				y2 = tmp[k - 1].y;

				val = c + n1 * x1 + n2 * y1;
				x1 -= val * n1;
				y1 -= val * n2;

				val = c + n1 * x2 + n2 * y2;
				x2 -= val * n1;
				y2 -= val * n2;

				dist = _hypot(x2 - x1, y2 - y1);

				if (dist < 0.01f)
					continue;

				k = ln2->m_lEnd + 1 - ln1->m_lStart;
				appdist = dist / (float)k;

				ln1->m_lEnd = ln2->m_lEnd;
				ln1->m_ptStart.x = x1;
				ln1->m_ptStart.y = y1;
				ln1->m_ptEnd.x = x2;
				ln1->m_ptEnd.y = y2;
				ln1->m_fTotalLen = dist;
				ln1->a = n1;
				ln1->b = n2;
				ln1->c = c;
				ln1->m_fSigma2 = sigma2;

				if (ln1 != at(i))
					*at(i) = *ln1;

				memcpy(&sp[ln1->m_lStart], tmp, k * sizeof(*sp));  // �˴��ı���ԭ����ɨ���????

				delete at(j);
				erase(begin() + j);
				change = true;
				j--;
			}
	}


	// ���¸����Ӧ��ֱ�߶α��
	for (long i = 0; i < (long)size(); i++)
	{
		CLineFeature *ln = at(i);
		for (long j = ln->m_lStart; j <= ln->m_lEnd; j++)
		{
			long old = sp[j].m_nLineID;

			if (lineNum != NULL && old >= 0 && old < totalLines)
				lineNum[old] = i;

			sp[j].m_nLineID = i;
		}
	}

	SSfree(tmp);
}

///////////////////////////////////////////////////////////////////////////////

//
//   ȥ�����г��ȶ���minLineLength��ֱ�ߡ�
//
void CLineFeatureSet::LengthFilter(float minLineLength)
{
	for (int i = size() - 1; i >= 0; i--)
		if (at(i)->m_fTotalLen < minLineLength)
		{
			delete at(i);
			erase(begin() + i);
		}
}

//
//   �ж�ֱ��ɨ�輯�Ƿ����ָ���ĵ㡣
//
bool CLineFeatureSet::ContainScanPoint(const CScanPoint& sp)
{
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i)->DistanceToPoint(true, sp) < 50)
			return true;
	}

	return false;
}

//
//   �Ƴ�λ��ָ�������ڵ��߶Ρ�
//
void CLineFeatureSet::RemoveWithin(const CRectangle& r)
{
	for (int j = size() - 1; j >= 0; j--)
	{
		if (r.Contain(*at(j)))
		{
			delete at(j);
			erase(begin() + j);
		}
	}
}

//
//   ����������ƽ�ơ�
//
void CLineFeatureSet::Move(float fX, float fY)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Move(fX, fY);
}

//
//   ������������ת��
//
void CLineFeatureSet::Rotate(CAngle ang, CPnt ptCenter)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Rotate(ang, ptCenter);
}

//
//   ѡ��/ȡ��ѡ��ָ�����߶Ρ�
//
void CLineFeatureSet::Select(int nIdx, bool bOn)
{
	if (nIdx < 0)
	{
		for (int i = 0; i < (int)size(); i++)
			at(i)->Select(bOn);
	}
	else
		at(nIdx)->Select(bOn);
}

//
//   ���ı��ļ�װ��ֱ���������ϡ�
//   ����ֵ��
//     < 0 : ��ȡʧ��
//     >= 0: ��ȡ����������
//
int CLineFeatureSet::LoadText(FILE* fp)
{
	Clear();

	// �ȶ���ֱ������
	int nCount;
	if (fscanf(fp, "%d\n", &nCount) != 1 || nCount < 0)
		return -1;

	// ���ζ����ֱ������
	for (int i = 0; i < nCount; i++)
	{
		// ����ֱ������
		int nSubType;
		if (fscanf(fp, "%d\t", &nSubType) != 1)
			return -1;

		// �������ͷ���ռ�
		CLineFeature* pFeature = NewLineFeature(nSubType);
		if (pFeature == NULL)
			return -1;

		// �������Ͷ���ֱ����������
		if (pFeature->LoadText(fp) < 0)
			return -1;

		push_back(pFeature);
	}

	UpdateCoveringRect();

	return true;
}

//
//   ��ֱ���������ϴ浽�ı��ļ���
//
int CLineFeatureSet::SaveText(FILE* fp)
{
	// ��дֱ������
	int nCount = (int)size();
	fprintf(fp, "%d\n", nCount);

	// ����д���ֱ������
	for (int i = 0; i < nCount; i++)
		at(i)->SaveText(fp);

	return nCount;
}

//
//   �Ӷ������ļ�װ��ֱ���������ϡ�
//
int CLineFeatureSet::LoadBinary(FILE* fp)
{
	Clear();

	// �ȶ���ֱ������
	int nCount;
	if (fread(&nCount, sizeof(int), 1, fp) != 1 || nCount < 0)
		return -1;

	// ���ζ����ֱ������
	for (int i = 0; i < nCount; i++)
	{
		// ����ֱ������
		int nSubType;
		if (fread(&nSubType, sizeof(int), 1, fp) != 1)
			return -1;

		// �������ͷ���ռ�
		CLineFeature* pFeature = NewLineFeature(nSubType);
		if (pFeature == NULL)
			return -1;

		// �������Ͷ���ֱ����������
		if (!pFeature->LoadText(fp))
			return -1;

		push_back(pFeature);
	}

	UpdateCoveringRect();

	return nCount;
}

//
//   ��ֱ���������ϴ浽�������ļ���
//
int CLineFeatureSet::SaveBinary(FILE* fp)
{
	// ��дֱ������
	int nCount = (int)size();
	if (!fwrite(&nCount, sizeof(int), 1, fp) != 1)
		return -1;

	// ����д���ֱ������
	for (int i = 0; i < nCount; i++)
		if (at(i)->SaveBinary(fp) < 0)
			return -1;

	return nCount;
}

//
//   ���¼���߽�ֵ��
//
void CLineFeatureSet::UpdateCoveringRect()
{
	m_rect.Clear();

	for (int i = 0; i < (int)size(); i++)
		m_rect += *at(i);
}

//
//   ����ֱ�������������ɽǵ㼯�ϡ�
//
int CLineFeatureSet::CreateCornerPoints(vector<CPointFeature>& ptCorners)
{
	// �����������������ֱཻ�ߵĽ��㼯�ϣ���Ϊ���ǵ�������
	ptCorners.clear();

	for (int i = 0; i < (int)size() - 1; i++)
	{
		CLineFeature* pLine1 = at(i);
		if (pLine1->Length() < MIN_LINE_LEN)
			continue;

		for (int j = i + 1; j < (int)size(); j++)
		{
			CLineFeature* pLine2 = at(j);
			if (pLine2->Length() < MIN_LINE_LEN)
				continue;

			// ��������ֱ�ߵĲ��
			CAngle angDiff = pLine1->SlantAngle() - pLine2->SlantAngle();
			if ((angDiff > MIN_CORNER_ANGLE && angDiff < (PI - MIN_CORNER_ANGLE)) ||
				(angDiff >(PI + MIN_CORNER_ANGLE) && angDiff < (2 * PI - MIN_CORNER_ANGLE)))
			{
				float x, y;
				bool bOnLine1 = false;
				bool bOnLine2 = false;

				// ��������ֱ�߶εĽ��㣬���жϽ����Ƿ��������߶���
				if (pLine1->Intersect(*pLine2, &x, &y, &bOnLine1, &bOnLine2))
				{
					CPointFeature pt;
					pt.x = x;
					pt.y = y;
					pt.m_nType = 1;                  // �ǵ�����
					pt.m_nParam[0] = pLine1->m_nId;      // ֱ��1��ID��
					pt.m_nParam[1] = pLine2->m_nId;      // ֱ��2��ID��
#if 0
					pt.m_fParam[0] = pLine1->SlantAngle().m_fRad;  // ֱ��1�����
					pt.m_fParam[1] = pLine2->SlantAngle().m_fRad;  // ֱ��2�����
#endif
					float fDist;

					// ������㲻��ֱ��1��
					if (!bOnLine1)
					{
						pLine1->FindNearPoint(pt, &fDist);
						float fLen1 = pLine1->Length();

						// ���ں̵ܶ�ֱ��������Ҫ���ӳ����Ȳ��ܳ���ֱ����������ĳ���
						if (fLen1 < MIN_LINE_LEN * 3)
						{
							if (fDist > fLen1)
								continue;
						}
						// ����һ�㳤�ȵ�ֱ���������ӳ����Ȳ��ܳ����涨�Ĺ̶�����(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// ������㲻��ֱ��2��
					if (!bOnLine2)
					{
						pLine2->FindNearPoint(pt, &fDist);
						float fLen2 = pLine2->Length();

						// ���ں̵ܶ�ֱ��������Ҫ���ӳ����Ȳ��ܳ���ֱ����������ĳ���
						if (fLen2 < MIN_LINE_LEN * 3)
						{
							if (fDist > fLen2)
								continue;
						}

						// ����һ�㳤�ȵ�ֱ���������ӳ����Ȳ��ܳ����涨�Ĺ̶�����(MAX_HIDDEN_LINE_LEN)
						else if (fDist > MAX_HIDDEN_LINE_LEN)
							continue;
					}

					// ����õ�����ǰ����ĵ�̫�����򲻽�������
					if (/*!PointTooCloseToSomeCorner(pt, ptCorners)*/1)
						ptCorners.push_back(pt);
				}
			}
		}
	}
	return (int)ptCorners.size();
}

//
//   �жϸ����ĵ��Ƿ���ĳ���ǵ���������
//
bool CLineFeatureSet::PointTooCloseToSomeCorner(CPnt& pt, vector<CPnt>& ptCorners)
{
	bool bTooClose = false;
	for (vector<CPnt>::iterator iter = ptCorners.begin(); iter != ptCorners.end(); iter++)
	{
		CPnt& pti = iter->GetPntObject();
		if (pt.DistanceTo(pti) < 200)
		{
			bTooClose = true;
			break;
		}
	}

	return bTooClose;
}


//
//   �Ӷ������ļ���װ���û��༭���ݡ�
//
bool CLineFeatureSet::LoadUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		if (fread(&n, sizeof(int), 1, fp) != 1)
			return false;

		// ����Ϊ0ʱ����ʹ��
		at(i)->Enable(n == 0);
	}

	return true;
}

//
//   ���û��༭���ݱ��浽�������ļ��С�
//
bool CLineFeatureSet::SaveUserData(FILE* fp)
{
	int n;
	for (int i = 0; i < (int)size(); i++)
	{
		// ����ʹ��ʱ��0
		n = !(at(i)->IsEnabled());
		if (fwrite(&n, sizeof(int), 1, fp) != 1)
			return false;
	}

	return true;
}

//
//   ������һ��LineFeatureSet�������û�ʹ�����á�
//
bool CLineFeatureSet::CopyUserData(const CLineFeatureSet& another)
{
	if (another.size() != size())
		return false;

	for (int i = 0; i < (int)size(); i++)
	{
		bool bEnabled = another.at(i)->IsEnabled();
		at(i)->Enable(bEnabled);
	}

	return true;
}

//
//   ��ֱ�߼���ת�����ֲ�����ϵ�У�ת�����ԭ����̬�䵽ָ������̬����
//
void CLineFeatureSet::Transform(CPosture& pstLocal)
{
	// ��������任
	CTransform trans(pstLocal);

	// ����ֱ�ߵ�����任
	for (int i = 0; i < (int)size(); i++)
	{
		// ��ȡ��ֱ�߶ε������˵�
		CPnt pt1 = at(i)->m_ptStart;
		CPnt pt2 = at(i)->m_ptEnd;

		// �������˵����α任���ֲ�����ϵ��
		CPnt ptLocal1 = trans.GetLocalPoint(pt1);
		CPnt ptLocal2 = trans.GetLocalPoint(pt2);

		// ���ɱ任���ֱ�߶�
		at(i)->Create(ptLocal1, ptLocal2);
	}

	// �����߽�ֵ
	UpdateCoveringRect();
}

//
//   ��ֱ�߼���ת������������ϵ�У�ת����ԭ����ԭ����̬��Ҫ����ָ������̬��
//
void CLineFeatureSet::InvTransform(CPosture& pstOrigin)
{
	// ��������任
	CTransform trans(pstOrigin);

	// ����ֱ�ߵ�����任
	for (int i = 0; i < (int)size(); i++)
	{
		// ��ȡ��ֱ�߶ε������˵�
		CPnt pt1 = at(i)->m_ptStart;
		CPnt pt2 = at(i)->m_ptEnd;
		
		// �������˵����α任����������ϵ��
		CPnt ptWorld1 = trans.GetWorldPoint(pt1);
		CPnt ptWorld2 = trans.GetWorldPoint(pt2);

		vector<CRange> ranges = at(i)->m_Ranges;

		// ���ɱ任���ֱ�߶�
		at(i)->Create(ptWorld1, ptWorld2);
		at(i)->m_Ranges = ranges;
	}

	// �����߽�ֵ
	UpdateCoveringRect();
}

//
//   �Ը����ĵ�Ϊ���ģ���ָ���ķ�ΧΪ�뾶����ȡ��һ�������Ӽ���
//
bool CLineFeatureSet::GetSubset(CPnt& ptCenter, float fRange, CLineFeatureSet& Subset)
{
	Subset.Clear();

	// �����ж�����ֱ�������Ƿ�Ӧ�����Ӽ���
	for (int i = 0; i < (int)size(); i++)
	{
		bool bSelect = false;
		CLineFeature* pFeature = at(i);

		float fDist1 = pFeature->m_ptStart.DistanceTo(ptCenter);      // ֱ����㵽���ĵ�ľ���
		float fDist2 = pFeature->m_ptEnd.DistanceTo(ptCenter);        // ֱ���յ㵽���ĵ�ľ���

		// ���ֱ�ߵ������˵�����һ��λ��Բ�ڣ�ѡ�������Ӽ�
		if (fDist1 < fRange || fDist2 < fRange)
			bSelect = true;
		else
		{
			float fLambda;
			CPnt ptFoot;

			// ����ֱ�ߵ����ĵ�ľ��룬����ô��������
			pFeature->DistanceToPoint(false, ptCenter, &fLambda, &ptFoot);
			
			// �����������߶����ڣ����Ҵ����Ҳ��Բ�ڣ�ѡ�������Ӽ�
			if (fLambda >= 0 && fLambda <= 1 && ptFoot.DistanceTo(ptCenter) < fRange)
				bSelect = true;
		}

		if (bSelect)
			Subset.Add(*pFeature);
	}

	return true;
}

#define Q_MIN_LINE_LEN          0.3f
#define Q_MIN_INC_ANGLE         TO_RADIAN(45)            // ��С�н�45��
#define Q_MAX_INC_ANGLE         TO_RADIAN(135)           // ���н�135��

//
//   ��Ե�ǰ��ֱ���������ϣ���������Ϻõ�ֱ���������ϡ��͡��ǵ��������ϡ���
//
bool CLineFeatureSet::SeperateFeatures(CLineFeatureSet& GoodLineFeatureSet, CPointFeatureSet& CornerFeatureSet)
{
	GoodLineFeatureSet.Clear();
	CornerFeatureSet.Clear();

	for (int i = 0;i < (int)size(); i++)
	{
		CLineFeature* p = at(i);

		// �����ֱ�߶ι�����ֱ�ӽ�����롰�ϸ�ֱ���������ϡ���
		if (p->Length() > Q_MIN_LINE_LEN)
		{
			GoodLineFeatureSet.Add(*p);
			continue;
		}

		// ����Ƕ�ֱ��
		CPnt pt;
		
		// ��ǰһ��ֱ�߽��бȽϣ����Ƿ��ܹ��ɽǵ�
		if (i > 0)
		{
			CLineFeature* p1 = at(i - 1);

			// ����ܹ��ɽǵ㣬����Ӵ˽ǵ�
			if (p1->Length() > Q_MIN_LINE_LEN && p->MakeCorner(*p1, Q_MIN_INC_ANGLE, Q_MAX_INC_ANGLE, 0.03f, pt))
			{
				CPointFeature* pFeature = new CPointFeature;
				pFeature->SetSubType(CORNER_FEATURE);
				pFeature->SetCenterPoint(pt);
				CornerFeatureSet += pFeature;
			}
		}
		else if (i < (int)size() - 1)
		{
			CLineFeature* p1 = at(i + 1);

			// ����ܹ��ɽǵ㣬����Ӵ˽ǵ�
			if (p1->Length() > Q_MIN_LINE_LEN && p->MakeCorner(*p1, Q_MIN_INC_ANGLE, Q_MAX_INC_ANGLE, 0.03f, pt))
			{
				CPointFeature* pFeature = new CPointFeature;
				pFeature->SetSubType(CORNER_FEATURE);
				pFeature->SetCenterPoint(pt);
				CornerFeatureSet += pFeature;
			}
		}
	}

	return true;
}

//
//   �����������任��
//
void CLineFeatureSet::Transform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->Transform(frame);

	m_pstScanner.Transform(frame);

	UpdateCoveringRect();
}

//
//   ����������任��
//
void CLineFeatureSet::InvTransform(const CFrame& frame)
{
	for (int i = 0; i < (int)size(); i++)
		at(i)->InvTransform(frame);

	m_pstScanner.InvTransform(frame);

	UpdateCoveringRect();
}

#ifdef _MFC_VER

void CLineFeatureSet::Dump()
{
//	TRACE("Dumping Line Scan (%d lines):\n", size());

	for (int i = 0; i < (int)size(); i++)
	{
		DebugTrace(_T("Line #%d:\t%.2f\t%.2f\t\t%.2f\t%.2f\n"), i,
			at(i)->m_ptStart.x, at(i)->m_ptStart.y,
			at(i)->m_ptEnd.x, at(i)->m_ptEnd.y);

	}

	DebugTrace(_T("\n"));
}

//
//   ����ֱ���������ϡ�
//
void CLineFeatureSet::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
	int nPointSize, bool bShowActiveSide, bool bShowId, bool bShowRefPoint, int nShowDisabled)
{
	for (int i = 0; i < (int)size(); i++)
	{
		at(i)->SetIntParam(0, (int)bShowActiveSide);
		at(i)->SetIntParam(1, (int)bShowId);
		at(i)->SetIntParam(2, (int)bShowRefPoint);

		at(i)->Plot(ScrnRef, pDC, crColor, crSelected, nPointSize, nShowDisabled);
	}
}

//
//   �ж�һ���������Ƿ�����ĳһ��ֱ������(������Ļ��ֱ�ߴ����ж�)��
//   ����ֵ��
//      -1��������û�д������κ�ֱ��������
//
int CLineFeatureSet::PointHit(const CPnt& pt, float fDistGate)
{
	// ���������ֱ�����������ж�
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i)->PointHit(pt, fDistGate))
			return i;
	}

	return -1;
}

#endif

