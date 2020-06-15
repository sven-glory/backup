#ifndef __CMultiSegLine
#define __CMultiSegLine

#include <vector>
#include "Geometry.h"
#include "Range.h"

using namespace std;


///////////////////////////////////////////////////////////////////////////////
// ���ֱ��
class CMultiSegLine : public CLine
{
public:
	CRangeSet m_Ranges;

private:
	void CreateSingleLine();
	void ProjectToBaseLine(CLineBase& Line);

public:
	// Constructor form #1
	CMultiSegLine(const CPnt& ptStart, const CPnt& ptEnd);

	// Constructor form #2
	CMultiSegLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// Constructor form #3
	CMultiSegLine(const CLine& Line2);

	// The default constructor
	CMultiSegLine() {}

	// ȡ��ָ�����Ӷ�
	bool GetSegment(int i, CLine& Line) const;

	// �������������߶�
	void Create(const CPnt& ptStart, const CPnt& ptEnd);

	// ����һ���ֱ�ߵ���б�������߶�
	void Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen);

	// �Ӷ����߶����ɴ˶����
	void Create(CLine* pLines, int nNum);

	// �Ӷ�������ɴ˶����
	void Create(const CMultiSegLine& MultiSegLine);

	// �ж�һ�������ĵ��Ƿ񡰴���������ֱ��
	bool PointHit(const CPnt& pt, float fDistGate);

#ifdef _MFC_VER
	// ����Ļ�ϻ��ƴ�ֱ��
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth = 1, int nPointSize = 1, bool bBigVertex = false);
#endif
};
#endif
