#include "stdafx.h"
#include "Geometry.h"
#include "Frame.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   �����ƶ�ָ���ľ��롣
//
void CPnt::Move(float dx, float dy)
{
	x += dx;
	y += dy;
}

//
//   ������ԭ�������ת��
//
void CPnt::Rotate(float fAng, float fCx, float fCy)
{
	float fX = (float)((x - fCx) * cos(fAng) - (y - fCy) * sin(fAng) + fCx);
	float fY = (float)((x - fCx) * sin(fAng) + (y - fCy) * cos(fAng) + fCy);

	x = fX;
	y = fY;
}

//
//   ������ָ�������ĵ������ת��
//
void CPnt::Rotate(float fAng, const CPnt& ptCenter)
{
	float fX = (float)((x - ptCenter.x) * cos(fAng) - (y - ptCenter.y) * sin(fAng) + ptCenter.x);
	float fY = (float)((x - ptCenter.x) * sin(fAng) + (y - ptCenter.y) * cos(fAng) + ptCenter.y);

	x = fX;
	y = fY;
}

//
//   ���ز����� "==".
//
bool CPnt::operator == (const CPnt& pt) const
{
	return (x == pt.x && y == pt.y);
}

//
//   ���ز����� "!=".
//
bool CPnt::operator != (const CPnt& pt) const
{
	return (x != pt.x || y != pt.y);
}

//
//   ���� "+="
//
void CPnt::operator += (const CPnt& pt)
{
	x += pt.x;
	y += pt.y;
}

//
//   ���� "+"
//
CPnt CPnt::operator +(const CPnt& pt) const
{
	CPnt ptNew(x, y);
	ptNew += pt;
	return ptNew;
}

//
//   ���� "-="��
//
void CPnt::operator -= (const CPnt& pt)
{
	x -= pt.x;
	y -= pt.y;
}

//
//   ���� "-"��
CPnt CPnt::operator -(const CPnt& pt) const
{
	CPnt ptNew(x, y);
	ptNew -= pt;

	return ptNew;
}

//
//   �Ƚ�������ġ���С��(���ݡ���-�¡�ԭ��)��
//
bool CPnt::operator < (const CPnt& pt) const
{
	if (x < pt.x)
		return true;
	else if (x == pt.x)
		return (y < pt.y);
	else
		return false;
}

//
//   �Ƚ�������ġ���С��(���ݡ���-�¡�ԭ��)��
//
bool CPnt::operator > (const CPnt& pt) const
{
	if (x > pt.x)
		return true;
	else if (x == pt.x)
		return (y > pt.y);
	else
		return false;
}

//
//   ����������֮��ľ���.
//
float CPnt::DistanceTo(const CPnt& pt) const
{
	float dx = x - pt.x;
	float dy = y - pt.y;
	return (float)sqrt(dx*dx + dy*dy);
}

//
//   ����������֮��ľ����ƽ����
//
float CPnt::Distance2To(const CPnt& pt2) const
{
	float d_x = x - pt2.x;
	float d_y = y - pt2.y;

	return d_x * d_x + d_y * d_y;
}

//
//   �ж��������Ƿ���Խ��Ƶ���Ϊ��һ����(���ǳ���)��
//
bool CPnt::IsEqualTo(const CPnt& pt2, float limit) const
{
	if (Distance2To(pt2) < (limit*limit))
		return true;
	else
		return false;
}

//
//   ���ݵϿ�������������ļ����ꡣ
//
void CPnt::UpdatePolar()
{
	// ���㼫��
	r = (float)sqrt(y*y + x*x);

	// ���㼫��
	// �������̫С��˵���㴦��ԭ�㸽�����޼���
	if (r < 1E-7)
		a = 0;
	else
	{
		float fAngle = (float)atan2(y, x);
		a = CAngle::NormAngle(fAngle);
	}
}

//
//   �Ը�������̬��Ϊ����ϵ��X�ᣬ���ݵϿ�������������ļ����ꡣ
//
void CPnt::UpdatePolar(const CPosture& pstRefFrame)
{
	// ���㼫��
	r = DistanceTo(pstRefFrame);

	// ���㼫��
	// �������̫С��˵���㴦��ԭ�㸽�����޼���
	if (r < 1E-7)
		a = 0;
	else
	{
		float fAngle = (float)atan2(y - pstRefFrame.y, x - pstRefFrame.x);
		fAngle -= pstRefFrame.fThita;
		a = CAngle::NormAngle(fAngle);
	}
}

//
//   ���ݼ�����������ĵϿ������ꡣ
//
void CPnt::UpdateCartisian()
{
	x = r * (float)cos(a);
	y = r * (float)sin(a);
}

//
//   ����õ㵽��һ��ĽǶȾ��롣
//
float CPnt::AngleDistanceTo(const CPnt& pt) const
{
	return (float)fabs(CAngle::NormAngle(a - pt.a));
}

//
//   ���ļ�װ������ݡ�
//
bool CPnt::Load(FILE* fp)
{
	if (fscanf(fp, "%d\t%f\t%f\t%f\t%f\n", &id, &x, &y, &r, &a) != 5)
		return false;

	return true;
}

//
//   ��������д���ļ���
//
bool CPnt::Save(FILE* fp)
{
	fprintf(fp, "%d\t%f\t%f\t%f\t%f\n", id, x, y, r, a);
	return true;
}

//
//   �����������任��
//
void CPnt::Transform(const CFrame& frame)
{
	float c = (float)cos(frame.fThita);
	float s = (float)sin(frame.fThita);
	float dx = x - frame.x;
	float dy = y - frame.y;

	x = dx * c + dy * s;
	y = dy * c - dx * s;
}

//
//   ����������任��
//
void CPnt::InvTransform(const CFrame& frame)
{
	float c = (float)cos(frame.fThita);
	float s = (float)sin(frame.fThita);

	CPnt pt(frame.x, frame.y);

	pt.x += x * c - y * s;
	pt.y += y * c + x * s;

	x = pt.x;
	y = pt.y;
}

#ifdef _MFC_VER

//
//   �����������캯����
//
void CPnt::operator = (const CPoint& point)
{
	x = (float)point.x;
	y = (float)point.y;
}

//
//   ���ز����� "==".
//
bool CPnt::operator == (const CPoint& point) const
{
	return ((x == (float)point.x) && (y == (float)point.y));
}

//
//   ���ز����� "!=".
//
bool CPnt::operator != (const CPoint& point) const
{
	return (x != (float)point.x || y != (float)point.y);
}

void CPnt::Dump()
{
	TRACE(_T("%d\t%f\t%f\t%f\t%f\n"), id, x, y, r, a);
}

//
//   ����Ļ�ϻ��Ƹõ�.
//   ע������߿�(nLineWidth)��Ϊ�㣬��ʾ��Ҫ������Բ��
//
void CPnt::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF color, int nPointSize, int nLineWidth)
{
	bool bSolidFill = true;

	// ����߿�Ϊ�㣬��ʾ��Ҫ������Բ
	if (nLineWidth > 0)
	{
		bSolidFill = false;
	}

	CPnt pt(x, y);

	CPoint pnt1 = ScrnRef.GetWindowPoint(pt);

	CRect r(pnt1.x - nPointSize, pnt1.y - nPointSize, pnt1.x + nPointSize, pnt1.y + nPointSize);

	CPen Pen;
	if (bSolidFill)
		Pen.CreatePen(PS_SOLID, 1, color);
	else
		Pen.CreatePen(PS_SOLID, nLineWidth, color);

	CPen* pOldPen = pDC->SelectObject(&Pen);

	CBrush Brush(color);
	CBrush* pOldBrush;

	if (bSolidFill)
		pOldBrush = pDC->SelectObject(&Brush);
	else
		pOldBrush = (CBrush*)pDC->SelectStockObject(NULL_BRUSH);

	pDC->Ellipse(&r);

	pDC->SelectObject(pOldPen);
	pDC->SelectObject(pOldBrush);
}
#elif defined QT_VERSION

CPnt::CPnt(const QPoint& point)
{
	x = (float)point.x();
	y = (float)point.y();
	a = 0;
	r = 0;
	id = 0;
}

//
//   �����������캯����
//
void CPnt::operator = (const QPoint& point)
{
	x = (float)point.x();
	y = (float)point.y();
}

//
//   ���ز����� "==".
//
bool CPnt::operator == (const QPoint& point) const
{
	return ((x == (float)point.x()) && (y == (float)point.y()));
}

//
//   ���ز����� "!=".
//
bool CPnt::operator != (const QPoint& point) const
{
	return (x != (float)point.x() || y != (float)point.y());
}

void CPnt::Dump()
{
//	TRACE(_T("%d\t%f\t%f\t%f\t%f\n"), id, x, y, r, a);
}

//
//   ����Ļ�ϻ��Ƹõ�.
//   ע������߿�(nLineWidth)��Ϊ�㣬��ʾ��Ҫ������Բ��
//
void CPnt::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor color, int nPointSize, int nLineWidth)
{
	bool bSolidFill = true;

	// ����߿�Ϊ�㣬��ʾ��Ҫ������Բ
	if (nLineWidth > 0)
	{
		bSolidFill = false;
	}

	CPnt pt(x, y);

	QPoint pnt1 = ScrnRef.GetWindowPoint(pt);

	QRect r(pnt1.x() - nPointSize, pnt1.y() - nPointSize, pnt1.x() + nPointSize, pnt1.y() + nPointSize);

	QPen Pen(color);
	if (bSolidFill)
		Pen.setWidth(1);
	else
		Pen.setWidth(nLineWidth);

	QBrush Brush(color);
	if (!bSolidFill)
		Brush.setStyle(Qt::NoBrush);

	pPainter->setPen(Pen);
	pPainter->setBrush(Brush);
	pPainter->drawEllipse(r);
}
#endif