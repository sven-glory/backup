//                         - ELLIPSE.CPP -
//
//   Implementation of class "CEllipse", which depicts ellipse.
//
//   Author: Zhang Lei
//   Date:   2014. 9. 11
//

#include "stdafx.h"
#include <math.h>
#include "Ellipse.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QTransform>
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
CEllipse::CEllipse(const CPnt& ptCenter, float fHalfMajorAxis, float fHalfMiorAxis, const CAngle& angSlant)
{
	Create(ptCenter, fHalfMajorAxis, fHalfMiorAxis, angSlant);
}

//
//   ���ɴ���ԲԲ��
//
void CEllipse::Create(const CPnt& ptCenter, float fHalfMajorAxis, float fHalfMinorAxis, const CAngle& angSlant)
{
	m_ptCenter = ptCenter;
	m_fHalfMajorAxis = fHalfMajorAxis;
	m_fHalfMinorAxis = fHalfMinorAxis;
	m_angSlant = angSlant;
}

//
//   �ж�һ�����Ƿ�����Բ���ڡ�
//
bool CEllipse::Contain(const CPnt& pt) const
{
	float d = m_ptCenter.DistanceTo(pt);
	
	// ����õ������Բ���ĵ���뼫������С�ڶ̰��ᣬ����Ϊ��������Բ��
	if (d < m_fHalfMinorAxis || d < 1E-5)
		return true;
	else
	{
		// ������Բ���ĵ��õ����ߵ����
		CAngle ang = atan2(pt.y - m_ptCenter.y, pt.x - m_ptCenter.x);
		CAngle ang2 = ang - m_angSlant;

		// �����Ӧ�ش���ǵ���Բ�㵽���ĵľ���
		float q1 = m_fHalfMajorAxis * cos(ang2);
		float q2 = m_fHalfMinorAxis *sin(ang2);
		float r = sqrt(q1*q1 + q2*q2);
		return (d <= r);
	}
}

#ifdef _MFC_VER

//
//   ����Ļ�ϻ��ƴ�Բ��
//
void CEllipse::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crLineColor, int nLineWidth,
	COLORREF crFillColor, bool bFill, int nPenStyle)
{
	CPen pen(nPenStyle, nLineWidth, crLineColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CBrush Brush(crFillColor);
	CBrush* pOldBrush;

	if (bFill)
		pOldBrush = pDC->SelectObject(&Brush);
	else
		pOldBrush = (CBrush*)pDC->SelectStockObject(NULL_BRUSH);


	CPnt ptLeftTop(m_ptCenter.x - m_fHalfMajorAxis, m_ptCenter.y + m_fHalfMinorAxis);
	CPnt ptRightBottom(m_ptCenter.x + m_fHalfMajorAxis, m_ptCenter.y - m_fHalfMinorAxis);

	CPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	CPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);
	
	// �õ�δ����ת�ľ�������
	CRect r(pnt1, pnt2);
	float x0 = r.CenterPoint().x;
	float y0 = r.CenterPoint().y;

	float c = cos(m_angSlant);
	float s = sin(m_angSlant);

	// ������ת�任����
	XFORM xf;
	xf.eDx = x0 - c * x0 + s * y0;
	xf.eDy = y0 - c * y0 - s * x0;
	xf.eM11 =  c;
	xf.eM12 =  s;
	xf.eM21 = -s;
	xf.eM22 =  c;

	HDC hdc = pDC->GetSafeHdc();
	int nGraphicsMode = ::SetGraphicsMode(hdc, GM_ADVANCED);
	::SetWorldTransform(hdc, &xf);

	// ������Բ
	pDC->Ellipse(r);

	// �����ָ���������
	xf.eM11 = (float)1.0;
	xf.eM12 = (float)0;
	xf.eM21 = (float)0;
	xf.eM22 = (float)1.0;
	xf.eDx = (float)0;
	xf.eDy = (float)0;

	SetWorldTransform(hdc, &xf);
	SetGraphicsMode(hdc, nGraphicsMode);

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);
}

#elif defined QT_VERSION

//
//   ����Ļ�ϻ��ƴ�Բ��
//
void CEllipse::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crLineColor, int nLineWidth,
	QColor crFillColor, bool bFill)
{
	QPen pen(crLineColor);
	pen.setWidth(nLineWidth);
	pPainter->setPen(pen);

	QBrush brush(crFillColor);

	if (!bFill)
		brush.setStyle(Qt::NoBrush);

	pPainter->setBrush(brush);

	CPnt ptLeftTop(m_ptCenter.x - m_fHalfMajorAxis, m_ptCenter.y + m_fHalfMinorAxis);
	CPnt ptRightBottom(m_ptCenter.x + m_fHalfMajorAxis, m_ptCenter.y - m_fHalfMinorAxis);

	QPoint pnt1 = ScrnRef.GetWindowPoint(ptLeftTop);
	QPoint pnt2 = ScrnRef.GetWindowPoint(ptRightBottom);
	QPoint pntCenter = (pnt1 + pnt2) / 2;

	float ang = m_angSlant.m_fRad;
	float cx = pntCenter.x();
	float cy = pntCenter.y();
	int dx = (int)( cos(ang)*cx + sin(ang)*cy);
	int dy = (int)(-sin(ang)*cx + cos(ang)*cy);

	// �õ�δ����ת�ľ�������
	QRect r(pnt1, pnt2);
	double rotateAngleDegree = (double)m_angSlant.Degree();

	// ������ת�任����
	pPainter->save();
	pPainter->rotate(rotateAngleDegree);
	pPainter->translate(dx - cx, dy - cy);
	pPainter->drawEllipse(r);
	pPainter->restore();
}
#endif
