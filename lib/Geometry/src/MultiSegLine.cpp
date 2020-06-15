#include <stdafx.h>
#include "MultiSegLine.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   Constructor form #1
//
CMultiSegLine::CMultiSegLine(const CPnt& ptStart, const CPnt& ptEnd) : CLine(ptStart, ptEnd)
{
	CreateSingleLine();
}

//
//   Constructor form #2
//
CMultiSegLine::CMultiSegLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen) :
	CLine(ptStart, angSlant, fTotalLen)
{
	CreateSingleLine();
}

//
//   Constructor form #3
//
CMultiSegLine::CMultiSegLine(const CLine& Line2) :
	CLine(Line2)
{
	CreateSingleLine();
}

//
//   ���ɵ�����ֱ�߶Ρ�
//
void CMultiSegLine::CreateSingleLine()
{
	CRange range(0, Length());

	m_Ranges.clear();
	m_Ranges.push_back(range);
}

//
//   ȡ��ָ�����ӶΡ�
//
bool CMultiSegLine::GetSegment(int i, CLine& Line) const
{
	// �κźϷ��Լ��
	if (i >= (int)m_Ranges.size())
		return false;

	float fFrom = m_Ranges[i].fFrom;
	float fTo = m_Ranges[i].fTo;

	CPnt pt1 = TrajFun(fFrom);
	CPnt pt2 = TrajFun(fTo);

	Line.Create(pt1, pt2);

	return true;
}

//
//   �������������߶Ρ�
//
void CMultiSegLine::Create(const CPnt& ptStart, const CPnt& ptEnd)
{
	CLine::Create(ptStart, ptEnd);
	CreateSingleLine();
}

//
//   ����һ���ֱ�ߵ���б�������߶Ρ�
//
void CMultiSegLine::Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen)
{
	CLine::Create(ptStart, angSlant, fTotalLen);
	CreateSingleLine();
}

//
//   �Ӷ����߶����ɴ˶���ߡ�
//   ˵����
//       �˺����ٶ������Ķ���ֱ�߶��Ǵ���ƽ�еġ��ڴ�����£����򽫶���Щֱ�߶�
//   ������ɢ����ȡ�㣬����ϳ�һ��ֱ�ߡ���󣬳���ȡ������
//
void CMultiSegLine::Create(CLine* pLines, int nNum)
{
	m_Ranges.clear();

	if (nNum == 1)
	{
		Create(pLines[0].m_ptStart, pLines[0].m_ptEnd);
		return;
	}

	// �ȼ����ܳ���
	float fTotalLen = 0;
	for (int i = 0; i < nNum; i++)
		fTotalLen += pLines[i].Length();

	// �������������(5cm���)
	float fUnitLen = 0.05f;
	int nCountPoints = (int)(fTotalLen / fUnitLen);
	nCountPoints += nNum * 2;                  // ���ǵ���ͷ�����������һЩ�ռ�

	CPnt* pPoints = new CPnt[nCountPoints];

	// ͨ����ɢ������������߷���
	nCountPoints = 0;
	for (int i = 0; i < nNum; i++)
	{
		CLine& Line = pLines[i];
		float fLen = Line.Length();

		// �������������
		for (float fCurLen = 0; fCurLen < fLen; fCurLen += fUnitLen)
			pPoints[nCountPoints++].GetPntObject() = Line.TrajFun(fCurLen);
	}

	// ���Իع飬���ֱ�߷���
	CLineBase lineBase;
	bool bOk = lineBase.CreateFitLine(pPoints, nCountPoints);
	delete[]pPoints;

	if (!bOk)
		return;

	CLine* pProjectLine = new CLine[nNum];

	// �����ֱ�߶ε�������ߵ�ͶӰ
	for (int i = 0; i < nNum; i++)
	{
		pProjectLine[i] = lineBase.GetProjectLine(pLines[i]);

		// ��������Ϊλ����|�·���
		if (pProjectLine[i].m_ptStart > pProjectLine[i].m_ptEnd)
			pProjectLine[i].Reverse();

	}

	// �������С����������󡱵��յ�
	int nMinStartId = 0;
	int nMaxEndId = 0;

	for (int i = 1; i < nNum; i++)
	{
		if (pProjectLine[i].m_ptStart < pProjectLine[nMinStartId].m_ptStart)
			nMinStartId = i;

		if (pProjectLine[i].m_ptEnd > pProjectLine[nMaxEndId].m_ptEnd)
			nMaxEndId = i;
	}

	// �õ������߶ε���㡢�յ�
	CPnt ptStart = pProjectLine[nMinStartId].m_ptStart;
	CPnt ptEnd = pProjectLine[nMaxEndId].m_ptEnd;

	// �������������߶�
	Create(ptStart, ptEnd);
	m_Ranges.clear();

	// �������ɸ��ֶ�
	for (int i = 0; i < nNum; i++)
	{
		float fFrom = pProjectLine[i].m_ptStart.DistanceTo(ptStart);
		float fTo = pProjectLine[i].m_ptEnd.DistanceTo(ptStart);
		
		if (fTo < fFrom)
		{
			float fTemp = fFrom;
			fFrom = fTo;
			fTo = fTemp;
		}

		CRange range(fFrom, fTo);
		m_Ranges.push_back(range);
	}

	m_Ranges.Normalize();

	delete []pProjectLine;
}

//
//   �Ӷ�������ɴ˶���ߡ�
//
void CMultiSegLine::Create(const CMultiSegLine& MultiSegLine)
{
	*this = MultiSegLine;
}

//
//   �ж�һ�������ĵ��Ƿ񡰴��������ö��ֱ�ߡ�
//
bool CMultiSegLine::PointHit(const CPnt& pt, float fDistGate)
{
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line;
		GetSegment(i, Line);
		
		if (Line.PointHit(pt, fDistGate))
			return true;
	}

	return false;
}

#ifdef _MSC_VER

//
//   ����Ļ�ϻ��ƴ�ֱ�ߡ�
//
void CMultiSegLine::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bBigVertex)
{
	for (int i = 0; i < (int)m_Ranges.size(); i++)
	{
		CLine Line;
		GetSegment(i, Line);

		Line.Draw(ScrnRef, pDC, crColor, nWidth, nPointSize, bBigVertex);
	}
}
#endif
