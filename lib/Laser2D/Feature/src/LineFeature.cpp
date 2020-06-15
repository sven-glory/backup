#include "stdafx.h"
#include <math.h>
#include "LineFeature.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define ANGLE_COST_FACTOR       (180.0f/PI*0.1f)     // �Ƕȴ�������(ÿ���ۺϾ���100mm)
#define PROJECT_LINE_LEN        2.0f

///////////////////////////////////////////////////////////////////////////////

CLineFeature::CLineFeature(CPnt& ptStart, CPnt& ptEnd)
{
	Create(ptStart, ptEnd);
	m_nSubType = GENERIC_LINE_FEATURE;

	m_lStart = m_lEnd = 0;
	m_fSigma2 = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;
	m_nId = -1;
	m_bCreateRef = false;
}

CLineFeature::CLineFeature() 
{
	m_lStart = m_lEnd = 0;
	m_fSigma2 = 0;
	m_nWhichSideToUse = FEATURE_DIR_BOTH_SIDES;
	m_bSelected = false;
	m_nId = -1;
	m_bCreateRef = false;
}

//
//   ����һ������
//
CLineFeature* CLineFeature::Duplicate() const
{
	CLineFeature* p = new CLineFeature;
	*p = *this;
	return p;
}

int CLineFeature::LoadText(FILE* fp)
{
	int nCount;

	CPnt ptStart, ptEnd;
	if (fscanf(fp, "%f\t%f\t%f\t%f\t%d\t%d", &ptStart.x, &ptStart.y, &ptEnd.x, &ptEnd.y, &m_nWhichSideToUse, &nCount) != 6)
		return -1;

	Create(ptStart, ptEnd);

	m_Ranges.clear();

	CRange range;
	if (nCount == 1)
	{
		range.fFrom = 0;
		range.fTo = Length();
		m_Ranges.push_back(range);
	}
	else
	{
		for (int i = 0; i < nCount; i++)
		{
			if (fscanf(fp, "\t%f\t%f\n", &range.fFrom, &range.fTo) != 2)
				return -1;
			m_Ranges.push_back(range);
		}
	}

	return m_nSubType;
}

int CLineFeature::SaveText(FILE* fp)
{
	int nCount = m_Ranges.size();

	fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%d\t\t%d\n", m_nSubType, m_ptStart.x, m_ptStart.y,
		m_ptEnd.x, m_ptEnd.y, m_nWhichSideToUse, nCount);

	if (nCount > 1)
	{
		for (int i = 0; i < nCount; i++)
			fprintf(fp, "\t%f\t%f\n", m_Ranges[i].fFrom, m_Ranges[i].fTo);
		fprintf(fp, "\n");
	}

	return m_nSubType;
}

//
//   �Ӷ������ļ��������ݡ�
//
int CLineFeature::LoadBinary(FILE* fp)
{
	// ���������˵������
	float f[4];
	if (fread(f, sizeof(float), 4, fp) != 4)
		return -1;

	CPnt ptStart, ptEnd;
	ptStart.x = f[0];
	ptStart.y = f[1];
	ptEnd.x = f[2];
	ptEnd.y = f[3];

	Create(ptStart, ptEnd);

	// ����ֱ����������Ч���򼰹�������
	int n[2];
	if (fread(n, sizeof(int), 2, fp) != 2)
		return -1;

	m_nWhichSideToUse = n[0];
	int nCount = n[1];

	m_Ranges.clear();

	CRange range;

	// ���ֻ��һ�����߶�
	if (nCount == 1)
	{
		range.fFrom = 0;
		range.fTo = Length();
		m_Ranges.push_back(range);
	}
	else
	{
		// ����ж�����߶Σ������������
		for (int i = 0; i < nCount; i++)
		{
			if (fread(f, sizeof(float), 2, fp) != 2)
				return false;

			range.fFrom = f[0];
			range.fTo = f[1];

			m_Ranges.push_back(range);
		}
	}

	return m_nSubType;
}

//
//   ������д�뵽�������ļ���
//
int CLineFeature::SaveBinary(FILE* fp)
{
	// д��ֱ������
	if (fwrite(&m_nSubType, sizeof(int), 1, fp) != 1)
		return -1;

	// д�������˵������
	float f[4] = { m_ptStart.x, m_ptStart.y, m_ptEnd.x, m_ptEnd.y };
	if (fwrite(f, sizeof(float), 4, fp) != 4)
		return -1;

	int nCount = (int)m_Ranges.size();
	int n[2] = { m_nWhichSideToUse, nCount };

	// д��ֱ����������Ч���򼰹�������
	if (fwrite(n, sizeof(float), 2, fp) != 2)
		return -1;

	// ����ж�����߶Σ���������д�����λ��
	if (nCount > 1)
	{
		for (int i = 0; i < nCount; i++)
		{
			f[0] = m_Ranges[i].fFrom;
			f[1] = m_Ranges[i].fTo;

			if (fwrite(n, sizeof(float), 2, fp) != 2)
				return -1;
		}
	}

	return m_nSubType;
}

//
//   �����߶Ρ�
//
void CLineFeature::Create(CPnt& ptStart, CPnt& ptEnd)
{
	CMultiSegLine::Create(ptStart, ptEnd);

	m_bSelected = false;
}

//
//   �����߶Ρ�
//
void CLineFeature::Create(CLine& ln)
{
	Create(ln.m_ptStart, ln.m_ptEnd);
}

//
//   ���ݶ��������������
//
void CLineFeature::Create(CMultiSegLine& MultiSegLine)
{
	CMultiSegLine::Create(MultiSegLine);
	m_bSelected = false;
}

//
//   ����ɨ��⵽��ֱ������ʱ�ļ���ͷ��̬���Ա������Ч�Ĺ۲⳯��
//
void CLineFeature::SetDetectPosture(const CPosture& pstDetect)
{
	CPnt ptDetect = pstDetect;

	CAngle ang1(ptDetect, m_ptStart);
	CAngle ang2(ptDetect, m_ptEnd);

	// ȡֱ�߶ε��е�ΪͶӰ��
	m_bCreateRef = true;
	m_ptProjectFoot = GetMidpoint();
	CAngle ang = SlantAngle();

	// ����ang1��ang2��ת��С��PIʱ��ɨ��λ����ֱ������������(����)
	if ((ang2 - ang1).NormAngle() < PI)
	{
		m_nWhichSideToUse = FEATURE_DIR_FRONT_SIDE_ONLY;
		ang += PI / 2;
	}
	else
	{
		m_nWhichSideToUse = FEATURE_DIR_BACK_SIDE_ONLY;
		ang -= PI / 2;
	}

	// ����ͶӰ��
	CLine ln(m_ptProjectFoot, ang, PROJECT_LINE_LEN);

	// ��¼�ο���
	m_ptRef = ln.GetEndPoint();
}

//
//   �жϱ�������Ƿ�����һ��������ص�����
//
bool CLineFeature::IsOverlapWith(CLineFeature& Feature2)
{
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line1;
		GetSegment(i, Line1);

		for (int j = 0; j < (int)Feature2.m_Ranges.size(); j++)
		{
			CLine Line2;
			Feature2.GetSegment(j, Line2);

			// �����ж��Ƿ������ص��Ĳ���
			float lambda1, lambda2;

			// �����߶�LineFeature2����㡢�յ㵽��ֱ��(�����߶�!)�ľ���
			Line1.DistanceToPoint(false, Line2.m_ptStart, &lambda1);
			Line1.DistanceToPoint(false, Line2.m_ptEnd, &lambda2);

			// �������ͶӰ�㶼�����߶�����
			if (((lambda1 < 0) && (lambda2 < 0)) || ((lambda1 > 1) && (lambda2 > 1)))
				continue;
			else
				return true;
		}
	}

	return false;
}

bool CLineFeature::ColinearMerge1(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines)
{
	// ���ж��Ƿ�ƽ�У������ƽ��ֱ�ӷ���false
	if (!IsParallelTo(Feature2, fMaxAngDiff))
		return false;

	// ������������������ֱ�߷ֶε�����
	int nNum = m_Ranges.size() + Feature2.m_Ranges.size();
	
	// ��������������ֱ������ռ�
	CLine* pLine = new CLine[nNum];

	// ��������ֱ�߷ֶ����ݵ�������
	// �ȸ��Ʊ������еķֶ�
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line;
		GetSegment(i, Line);
		pLine[i] = Line;
	}

	// �ٸ��Ƶڶ��������еķֶ�
	for (int i = 0; i < (int)Feature2.m_Ranges.size(); i++)
	{
		CLine Line;
		Feature2.GetSegment(i, Line);
		pLine[m_Ranges.size() + i] = Line;
	}

	// ����ͳһ�Ķ����
	CMultiSegLine MultiSegLine;
	MultiSegLine.Create(pLine, nNum);
	MultiSegLine.m_nId = m_nId;

	// �������ɱ�����
	Create(MultiSegLine);

	// �ͷ���ʱ����ռ�
	delete []pLine;

	return true;
}

//
//   ���˶����������һ�������(������ߵĻ�)���кϲ���
//
bool CLineFeature::ColinearMerge(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines)
{
	// ���ж��Ƿ�ƽ�У������ƽ��ֱ�ӷ���false
	if (!IsParallelTo(Feature2, fMaxAngDiff))
		return false;

	// �����ж��Ƿ������ص��Ĳ���
	float lambda1, lambda2;

	// �����߶�Feature2����㡢�յ㵽��ֱ��(�����߶�!)�ľ���
	float fDist1 = DistanceToPoint(false, Feature2.m_ptStart, &lambda1);
	float fDist2 = DistanceToPoint(false, Feature2.m_ptEnd, &lambda2);

	// ������볬�ޣ��򲻹���
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return false;

	// ���Feature2�����λ�ڱ��߶ε�����֮��
	if (lambda1 < 0)
	{
		// ���Feature2���յ�Ҳλ�ڱ��߶ε�����֮�⣬Feature2��ȫλ�ڱ��߶�����
		if (lambda2 < 0)
		{
			int nNearPoint;
			float fDist;

			// ����������߶ε������֮��ľ���С�ڸ����������ޣ�����Ϊ������������
			nNearPoint = Feature2.FindNearPoint(m_ptStart, &fDist);
			if (fDist < fMaxGapBetweenLines)
			{
				if (nNearPoint == 0)
					Create(Feature2.m_ptEnd, m_ptEnd);
				else
					Create(Feature2.m_ptStart, m_ptEnd);

				return true;
			}
			else
				return false;
		}
		// Feature2���յ�λ�ڱ��߶ε��յ��֮��
		else if (lambda2 > 1)
		{
			// ˵��Feature2�����˱��߶�
			*this = Feature2;
			return true;
		}
		else
		{
			// ˵��Feature2����ʼ�����ڱ��߶�(��ʼ�㷽��)���棬������������ڱ��߶���
			Create(Feature2.m_ptStart, m_ptEnd);
			return true;
		}
	}

	// ���Feature2�����λ�ڱ��߶ε��յ��֮��
	else if (lambda1 > 1)
	{
		// ���Feature2���յ�Ҳλ�ڱ��߶ε��յ��֮��
		if (lambda2 > 1)
		{
			int nNearPoint;
			float fDist;

			// ����������߶ε������֮��ľ���С�ڸ����������ޣ�����Ϊ������������
			nNearPoint = Feature2.FindNearPoint(m_ptEnd, &fDist);
			if (fDist < fMaxGapBetweenLines)
			{
				if (nNearPoint == 0)
					Create(m_ptStart, Feature2.m_ptEnd);
				else
					Create(m_ptStart, Feature2.m_ptStart);
				return true;
			}
			else
				return false;
		}
		// Feature2���յ�λ�ڱ��߶ε�����֮��
		else if (lambda2 < 0)
		{
			// ˵��Feature2(����)�����˱��߶�(����Feature2������Ӧ�ı䱾�߶�ԭ���ķ���)
			Create(Feature2.m_ptEnd, Feature2.m_ptStart);
			return true;
		}
		else
		{
			// ˵��Feature2��������ڱ��߶�(��ֹ�㷽��)���棬������������ڱ��߶���
			Create(m_ptStart, Feature2.m_ptStart);
			return true;
		}
	}

	// Feature2�����λ�ڱ��߶���
	else
	{
		// Feature2���յ�λ�ڱ��߶ε�����֮��
		if (lambda2 < 0)
		{
			Create(Feature2.m_ptEnd, m_ptEnd);
		}
		// ���Feature2���յ�λ�ڱ��߶ε��յ��֮��
		else if (lambda2 > 1)
		{
			Create(m_ptStart, Feature2.m_ptEnd);
		}
		// ���Feature2���յ�λ�ڱ��߶�֮��(���ظı䱾�߶�)
		else
		{
		}
		return true;
	}
	return true;
}

//
//   ����������ƽ�ơ�
//
void CLineFeature::Move(float fX, float fY)
{
	CPnt ptStart = m_ptStart;
	CPnt ptEnd = m_ptEnd;

	ptStart.Move(fX, fY);
	ptEnd.Move(fX, fY);

	// ֻ�ƶ����߶ε������յ㣬���ֶη�Χ����
	CLine::Create(ptStart, ptEnd);
}

//
//   ������������ת��
//
void CLineFeature::Rotate(CAngle ang, CPnt ptCenter)
{
	// ���������յ㾭��ת���λ��
	m_ptStart.Rotate(ang.m_fRad, ptCenter);
	m_ptEnd.Rotate(ang.m_fRad, ptCenter);

	// ֻ�ı����߶ε������յ㣬���ֶη�Χ����
	CLine::Create(m_ptStart, m_ptEnd);
}

//
//   ��pstScannerΪ�۲���̬���жϱ�ֱ�������Ƿ�����������һֱ������������׼��
//
bool CLineFeature::RegisterWith(CLineFeature& another, CPosture& pstScanner,
	float fDistGate, float fAngGate)
{
	CPnt ptFoot1, ptFoot2;
	float fDist1 = DistanceToPoint(false, pstScanner, NULL, &ptFoot1);
	float fDist2 = another.DistanceToPoint(false, pstScanner, NULL, &ptFoot2);

	CAngle angDiff = AngleToUndirectionalLine(another);
	float fAngDiff = angDiff.m_fRad;
	if (fAngDiff > PI / 2)
		fAngDiff = PI - fAngDiff;

	// �Ƚ�������ɨ��㵽��������֮��ļн�
	CLine ln1(pstScanner, ptFoot1);
	CLine ln2(pstScanner, ptFoot2);

	CAngle angDiff2 = ln1.AngleToLine(ln2);
	float fAngDiff2 = fabs(angDiff2.NormAngle2());

	// ���롢�ǶȲ��������ʱ����Ϊ��׼�ɹ�
	if ((fabs(fDist2 - fDist1) < fDistGate) && (fAngDiff < fAngGate) && (fAngDiff2 < fAngGate))
		return true;
	else
		return false;
}

//
//   ���㱾ֱ����������һֱ�������ı任���ۡ�
//
float CLineFeature::TransformCostTo(const CLineFeature& another) const
{
	float fMidAngle;

	// ��ȡ������ֱ�ߵľ������
	float fAng1 = StdSlantAngleRad();
	float fAng2 = another.StdSlantAngleRad();

	// �þ�����Ǵ�ļ�ȥ��С���Ǹ����õ������ߵ���ǲ�
	float fAngCost = fAng1 - fAng2;
	if (fAng1 < fAng2)
	{
		fAngCost = -fAngCost;

		// ���������ǣ�ȷ��fAng1�ϴ�
		float fTemp = fAng1;
		fAng1 = fAng2;
		fAng2 = fTemp;
	}

	// ����������ǲ����PI/2��˵����ǽϴ��ֱ��Ӧ���÷����
	if (fAngCost > PI / 2)
	{
		fAngCost = PI - fAngCost;
		fMidAngle = (fAng1 + fAng2 - PI) / 2;
	}
	else
	{
		fMidAngle = (fAng1 + fAng2) / 2;
	}
	fAngCost *= ANGLE_COST_FACTOR;

	// ���湹������ֱ�������ε��м�ֱ��
	CPnt ptMid1 = GetMidpoint();
	CPnt ptMid2 = another.GetMidpoint();
	CLine lnTemp(ptMid1, ptMid2);
	CLineBase MidLine(lnTemp.GetMidpoint(), fMidAngle);

	// ��������ֱ�����������м�ֱ�ߵľ���
	CPnt ptFoot1 = MidLine.GetProjectPoint(ptMid1);

	float fDistCost = ptMid1.DistanceTo(ptFoot1) * 2;

	// �����任����Ϊ���Ƕȴ���+�������
	return fAngCost + fDistCost;
}

//
//   �жϸ�ֱ�������ܷ�������һ��ֱ����������һ�����ǵ㡱��
//
bool CLineFeature::MakeCorner(const CLineFeature& another, float fMinAngle, float fMaxAngle, float fMaxGap, CPnt& ptCorner)
{
	// ��������ֱ�ߵļн�
	CAngle ang = AngleToUndirectionalLine(another);
	float fAng = ang.m_fRad;

	// ����нǴ�С������Χ�����޷����ɽǵ�
	if (fAng < fMinAngle || fAng > fMaxAngle)
		return false;

	// ��������ֱ�ߵĽ���
	float x, y;
	if (!Intersect(another, &x, &y, NULL, NULL, 0.001f))
		return false;

	CPnt pt(x, y);
	
	// �жϽ����Ƿ����һ��ֱ�ߵ�ĳ���˵�ܽ�
	if (pt.DistanceTo(m_ptStart) > fMaxGap && pt.DistanceTo(m_ptEnd) > fMaxGap)
		return false;

	// �жϽ����Ƿ���ڶ���ֱ�ߵ�ĳ���˵�ܽ�
	if (pt.DistanceTo(another.m_ptStart) > fMaxGap && pt.DistanceTo(another.m_ptEnd) > fMaxGap)
		return false;

	ptCorner = pt;
	return true;
}

//
//   �����������任��
//
void CLineFeature::Transform(const CFrame& frame)
{
	CMultiSegLine::Transform(frame);

	if (m_bCreateRef)
	{
		m_ptRef.Transform(frame);
		m_ptProjectFoot.Transform(frame);
	}
}

//
//   ����������任��
//
void CLineFeature::InvTransform(const CFrame& frame)
{
	CMultiSegLine::InvTransform(frame);

	if (m_bCreateRef)
	{
		m_ptRef.InvTransform(frame);
		m_ptProjectFoot.InvTransform(frame);
	}
}

///////////////////////////////////////////////////////////////////////////////

#ifdef _MFC_VER
//
//   (����Ļ������)����ָ���ĵ��Ƿ����߶��ϡ�
//
bool CLineFeature::HitTest(CScreenReference& ScrnRef, CPoint point)
{
	CPoint pntStart = ScrnRef.GetWindowPoint(m_ptStart);
	CPoint pntEnd = ScrnRef.GetWindowPoint(m_ptEnd);

	CPnt ptStart((float)pntStart.x, (float)pntStart.y);
	CPnt ptEnd((float)pntEnd.x, (float)pntEnd.y);
	CPnt pt((float)point.x, (float)point.y);

	float fCheckDist = 5 / ScrnRef.m_fRatio;

	return PointHit(pt, fCheckDist);
}

//
//   ����ֱ��������
//
//   ע�⣺�����и�����������չ����ֵȷ���Ƿ���Ҫ��ʾ��1. ֱ�������ĳ���; 2.������ID��
//
void CLineFeature::Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
	int nSize, int nShowDisabled, bool bShowSuggested)
{
	bool bShowActiveSide = m_nParam[0];
	bool bShowId = m_nParam[1];
	bool bShowRefPoint = m_nParam[2];

	// ����ֱ������
	if (!m_bSelected)
	{
		// ������ʾ��ֹ��
		if (IsEnabled() || nShowDisabled == DISABLED_FEATURE_NORMAL)
			Draw(ScrnRef, pDC, crColor, nSize, nSize, false);

		// ��ɫ��ʾ��ֹ��
		else if (nShowDisabled == DISABLED_FEATURE_UNDERTONE)
		{
			BYTE r = GetRValue(crColor) / 3;
			BYTE g = GetGValue(crColor) / 3;
			BYTE b = GetBValue(crColor) / 3;
			Draw(ScrnRef, pDC, RGB(r, g, b), nSize, nSize, false);
		}
	}
	else
		Draw(ScrnRef, pDC, crSelected, 3 * nSize, nSize, false);

	// ��ʾ�ο�ͶӰ��
	if (bShowRefPoint && m_bCreateRef)
	{
		m_ptRef.Draw(ScrnRef, pDC, RGB(0, 0, 255), 3);
		m_ptProjectFoot.Draw(ScrnRef, pDC, RGB(255, 255, 0), 3);
	}

	// ��ʾֱ����������
	if (bShowActiveSide)
	{
		CAngle ang = SlantAngle();
		if (m_nWhichSideToUse == FEATURE_DIR_FRONT_SIDE_ONLY)
			ang += PI / 2;
		else if (m_nWhichSideToUse == FEATURE_DIR_BACK_SIDE_ONLY)
			ang -= PI / 2;

		float fArrowLen = 8 / ScrnRef.m_fRatio;
		CLine lnArrow(GetMidpoint(), ang, fArrowLen);
		lnArrow.Draw(ScrnRef, pDC, RGB(64, 64, 64));
	}

	// ��ʾֱ���������
	if (bShowId)
	{
		CString str;
		str.Format(_T("%d"), m_nId);

		pDC->SetTextColor(RGB(0, 255, 0));

		CPoint pnt = ScrnRef.GetWindowPoint(GetMidpoint());
		pDC->TextOut(pnt.x - 20, pnt.y - 20, str);
		pDC->SetTextColor(RGB(0, 0, 0));
	}
}
#endif

