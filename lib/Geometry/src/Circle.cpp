//                         - CIRCLE.CPP -
//
//   Implementation of class "CCircle", which depicts circle.
//
//   Author: Zhang Lei
//   Date:   2014. 9. 11
//

#include "stdafx.h"
#include <math.h>
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPainter>
#include <QColor>
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CArc".

//
//   The constructor.
//
CCircle::CCircle(const CPnt& ptCenter, float fRadius)
{
	Create(ptCenter, fRadius);
}

//
//   ���ɴ�Բ��
//
void CCircle::Create(const CPnt& ptCenter, float fRadius)
{
	m_ptCenter = ptCenter;
	m_fRadius = fRadius;
}

//
//   ȡ�ô�Բ��ֱ�߶εĵ�һ�����㡣
//
bool CCircle::IntersectLineAt(const CLine& Line, CPnt& ptNear, float& fDist)
{
	CPnt ptFoot;
	float fLambda;
	fDist = Line.DistanceToPoint(true, m_ptCenter, &fLambda, &ptFoot);

	// ����Բ�ĵ�������ֱ��ln1
	CLine ln1(ptFoot, m_ptCenter);

	// �����������Բ�ĵ�
	if (ln1.Length() < 1E-8)
	{
		CLine ln(m_ptCenter, Line.m_ptStart);

		// ���Բ�����߶�Line������غ�
		if (ln.Length() < 1E-8)
		{
			CLine ln0(m_ptCenter, Line.m_ptEnd);
			if (ln0.Length() >= m_fRadius)
			{
				ptNear = ln0.TrajFun(m_fRadius);
				fDist = Line.m_ptStart.DistanceTo(ptNear);
				return true;
			}
			else
				return false;
		}

		// ���Բ�����߶�Line����㲻�غ�
		else
		{
			CLine ln0(m_ptCenter, Line.m_ptStart);
			ptNear = ln0.TrajFun(m_fRadius);
			if (Line.ContainPoint(ptNear))
			{
				fDist = Line.m_ptStart.DistanceTo(ptNear);
				return true;
			}
			else
				return false;
		}
	}

	// ���ln1�ĳ���С��Բ�İ뾶��˵���������Բ�ڣ���һ��˵��ֱ��Line��Բ�н���
	else if (ln1.Length() <= m_fRadius)
	{
		// ���ڼ��������
		float fL = (float)sqrt(m_fRadius*m_fRadius - fDist*fDist);

		// ��λLine��Բ�Ľ���
		CLine ln2(ptFoot, Line.m_ptStart);
		if (ln2.Length() < 1E-8)
		{
			CLine ln4(ptFoot, Line.m_ptEnd);
			ptNear = ln4.TrajFun(fL);
			fDist = Line.m_ptStart.DistanceTo(ptNear);
			return true;
		}
		else
		{
			ptNear = ln2.TrajFun(fL);

			// �����ж�ptNear���Ƿ������߶�Line֮��(��Ϊ��Ҳ���������߶�Line���ӳ�����)
			CLine ln3(ptNear, Line.m_ptStart);
			if (ln3.Length() < Line.Length())
			{
				fDist = Line.m_ptStart.DistanceTo(ptNear);
				return true;
			}
			else
				return false;
		}
	}
	else
		return false;
}

//
//   �Դ�Բ��ȡһ��ֱ�ߣ�������ȡ������ֱ�߱��浽Line�С�
//
bool CCircle::CutLine(const CLine& Line, CLine& NewLine) const
{
	CPnt ptFoot, pt;
	float fLambda;
	float fDist = Line.DistanceToPoint(true, m_ptCenter, &fLambda, &ptFoot);

	// ������߳��ȳ����뾶��˵��û�н�ȡ��ֱ��
	if (fDist > m_fRadius)
		return false;
	
	// ������ҳ�
	float fL = (float)sqrt(m_fRadius*m_fRadius - fDist*fDist);

	// ��������Բ��
	if (Contain(Line.m_ptStart))
	{
		// �յ�Ҳ��Բ��
		if (Contain(Line.m_ptEnd))
		{
			// Line�ޱ仯
			NewLine = Line;
			return true;
		}
		// ����յ���Բ��
		else
		{
			CLine ln(ptFoot, Line.m_ptEnd);
			pt = ln.TrajFun(fL);

			// ���¹���ֱ��Line(���յ��б仯)
			NewLine.Create(Line.m_ptStart, pt);

			return true;
		}
	}

	// ��������Բ�⣬�յ���Բ��
	else if (Contain(Line.m_ptEnd))
	{
		CLine ln(ptFoot, Line.m_ptStart);

		// ������ҳ�
		float fL = (float)sqrt(m_fRadius*m_fRadius - fDist*fDist);
		pt = ln.TrajFun(fL);

		// ���¹���ֱ��Line(������б仯)
		NewLine.Create(pt, Line.m_ptEnd);

		return true;
	}

	// ��㡢�յ����Բ��
	else
	{
		// �ֱ��촹��㵽Line��㡢�յ������
		CLine ln1(ptFoot, Line.m_ptStart);
		CLine ln2(ptFoot, Line.m_ptEnd);
		
		// �������ͬ��˵��Line�������˵����Բ�⣬����Բû�н�ȡ��ֱ��Line
		if (ln1.SlantAngle() == ln2.SlantAngle())
			return false;

		// ���߷���˵��Բ��ֱ��Line����������ȡ��
		else
		{
			CPnt pt1 = ln1.TrajFun(fL);
			CPnt pt2 = ln2.TrajFun(fL);

			// Line�������˵㶼�仯��
			NewLine.Create(pt1, pt2);

			return true;
		}
	}
}

//
//   �ж�һ�����Ƿ���Բ��(��Բ��)��
//
bool CCircle::Contain(const CPnt& pt) const
{
	return (m_ptCenter.DistanceTo(pt) <= m_fRadius);
}

#ifdef _MFC_VER

//
//   ����Ļ�ϻ��ƴ�Բ��
//
void CCircle::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPenStyle)
{
	CPen pen(nPenStyle, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPnt ptLeftTop(m_ptCenter.x - m_fRadius, m_ptCenter.y + m_fRadius);
	CPnt ptRightBottom(m_ptCenter.x + m_fRadius, m_ptCenter.y - m_fRadius);

	CPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	CPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);

	CRect r(pnt1, pnt2);
	pDC->Arc(r, pnt1, pnt1);

	pDC->SelectObject(pOldPen);
}

#elif defined QT_VERSION
//
//   ����Ļ�ϻ��ƴ�Բ��
//
void CCircle::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth, int nPenStyle)
{
	QPen pen(crColor);
	pen.setWidth(nWidth);
	QBrush brush(crColor, Qt::NoBrush);

	pPainter->setPen(pen);
	pPainter->setBrush(brush);

	CPnt ptLeftTop(m_ptCenter.x - m_fRadius, m_ptCenter.y + m_fRadius);
	CPnt ptRightBottom(m_ptCenter.x + m_fRadius, m_ptCenter.y - m_fRadius);

	QPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	QPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);

	QRect r(pnt1, pnt2);
	pPainter->drawEllipse(r);
}
#endif
