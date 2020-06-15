//                          - LINE.CPP -
//
//   Implementation of class "CLine" - which defines the geometric concept
//   "Directional Straight Line".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#include "stdafx.h"
#include <math.h>
#include <float.h>
//#include "Tools.h"
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPoint>
#include <QPainter>
#include <QColor>
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//#define FLT_EPSILON        1.192092896e-07F
#define MIN_LINE_LENGTH       1e-3F

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   ��һ�ֹ��캯����
//
CLine::CLine(const CPnt& ptStart, const CPnt& ptEnd)
{
	Create(ptStart, ptEnd);
}

//
//   �ڶ��ֹ��캯����
//
CLine::CLine(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen)
{
	Create(ptStart, angSlant, fTotalLen);
}

//
//   �����ֹ��캯����
//
CLine::CLine(const CPosture& pstStart, float fTotalLen)
{
	Create(pstStart, pstStart.GetAngle(), fTotalLen);
}

//
//   �����ֹ��캯����
//
CLine::CLine(const CLine& Line2)
{
	*this = Line2;
}

//
//   �������������߶Ρ�
//
bool CLine::Create(const CPnt& ptStart, const CPnt& ptEnd)
{
	// ����ֱ�߳���
	float fTotalLen = ptStart.DistanceTo(ptEnd);

	// ���ֱ��̫�̣�����false
	if (fTotalLen < MIN_LINE_LENGTH)
		return false;

	// �������/�յ�/����
	m_ptStart = ptStart;
	m_ptEnd = ptEnd;
	m_fTotalLen = fTotalLen;
	m_nId = 0;

	// ����ֱ�ߵ����
	m_angSlant = (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);

	// �������a��b��c
	float xm = m_ptStart.x + m_ptEnd.x;
	float ym = m_ptStart.y + m_ptEnd.y;
	float xxm = m_ptStart.x * m_ptStart.x + m_ptEnd.x * m_ptEnd.x;
	float yym = m_ptStart.y * m_ptStart.y + m_ptEnd.y * m_ptEnd.y;
	float xym = m_ptStart.x * m_ptStart.y + m_ptEnd.x * m_ptEnd.y;

	float ang = 0.5f * (float)atan2((float)(-2.0*(xym - xm*ym / 2)), (float)(yym - ym*ym / 2 - xxm + xm*xm / 2));
	float n1 = (float)cos(ang);
	float n2 = (float)sin(ang);

	float dx = m_ptEnd.x - m_ptStart.x;
	float dy = m_ptEnd.y - m_ptStart.y;

	if (dx * n2 - dy * n1 > 0.0)
	{
		n1 = -n1;
		n2 = -n2;
	}

	a = n1;
	b = n2;
	c = -(n1 * xm + n2 * ym) / 2;
	
	return true;
}

//
//   ������ʼ�㡢��Ǻͳ��������߶Ρ�
//
bool CLine::Create(const CPnt& ptStart, const CAngle& angSlant, float fTotalLen)
{
	// ֱ�߲��ܹ���
	if (fTotalLen < MIN_LINE_LENGTH)
		return false;

	// �����յ�����
	CPnt ptEnd; 
	ptEnd.x = ptStart.x + fTotalLen * cos(angSlant);
	ptEnd.y = ptStart.y + fTotalLen * sin(angSlant);

	// ������������ֱ�߶�
	return Create(ptStart, ptEnd);
}

//
//   ��������ֱ�߶ε����в���(��ǡ�����)��
//
void CLine::ComputeParam()
{
	// Caculate the total length of the line
	float fDx = m_ptEnd.x - m_ptStart.x;
	float fDy = m_ptEnd.y - m_ptStart.y;
	m_fTotalLen = (float)sqrt(fDx*fDx + fDy*fDy);

	ASSERT(m_fTotalLen != 0);
	if (m_fTotalLen == 0)
		return;
	// Caculate the line's slant angle
	float fTemp = (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);
	m_angSlant = CAngle(fTemp);
}

//
//   ��ת�߶εķ���(����㡢�յ�Ե�)��
//
void CLine::Reverse()
{
	CPnt ptStart = m_ptStart;
	CPnt ptEnd = m_ptEnd;
	Create(ptEnd, ptStart);
}

//
//   �ı��߶εĳ���(�������ӳ�/����)��
//   fDist1: ����ʼ���ӳ��ľ��롣
//   fDist2: ����ֹ���ӳ��ľ��롣
//
//   ע�⣺������������ֵ����Ϊ������
//
bool CLine::Resize(float fDist1, float fDist2)
{
	// ·�γ��Ȳ��ù�С(������Ϊ��)
	if (m_fTotalLen + fDist1 + fDist2 < MIN_LINE_LENGTH)
		return false;

	m_ptStart.x -= fDist1 * cos(m_angSlant);
	m_ptStart.y -= fDist1 * sin(m_angSlant);

	m_ptEnd.x += fDist2 * cos(m_angSlant);
	m_ptEnd.y += fDist2 * sin(m_angSlant);

	m_fTotalLen += fDist1 + fDist2;
	
	return true;
}

//
//   ȡ���߶γ��ȵ�ƽ����
//
float CLine::Length2() const
{
	return m_ptStart.Distance2To(m_ptEnd);
}

//
//   �켣�㺯����
//
CPnt CLine::TrajFun(float fCurLen) const
{
	CPnt pt = m_ptStart;
	pt.x += fCurLen * cos(m_angSlant);
	pt.y += fCurLen * sin(m_angSlant);

	return pt;
}

//
//   ȡ���߶ε���б�ǡ�
//
float CLine::SlantAngleRad() const
{
	return (float)atan2(m_ptEnd.y - m_ptStart.y, m_ptEnd.x - m_ptStart.x);
}

//
//   �����߶ε��е㡣
//
CPnt CLine::GetMidpoint() const
{
	CPnt ptCenter;
	ptCenter.x = (m_ptStart.x + m_ptEnd.x) / 2;
	ptCenter.y = (m_ptStart.y + m_ptEnd.y) / 2;

	return ptCenter;
}

#if 0
//
//    The curvature generation function��
//
float CLine::CurvatureFun()
{
	return 0.0f;
}
#endif

//
//   �ж�һ�����Ƿ��ڴ�ֱ��(��)�ϡ�
// !!!!!!!!! ����ļ�����������⣬��ln1, ln2�ǳ���ʱ�����ܻ���ִ�����!!!!
//
bool CLine::ContainPoint(const CPnt& pt, bool bExtend) const
{
	if (pt == m_ptStart || pt == m_ptEnd)
		return true;

	// !!!!!!!!! ����ļ�����������⣬��ln1, ln2�ǳ���ʱ�����ܻ���ִ�����!!!!
	CLine ln1(pt, m_ptStart);
	CLine ln2(pt, m_ptEnd);

	// bExtend: ����������߶ε������ӳ�����
	if (bExtend)
	{
		if (ln1.SlantAngle() == m_angSlant || !ln1.SlantAngle() == m_angSlant)
			return true;
	}

	// ֻ����������߶�����
	else
	{
		if ((ln1.SlantAngle() == m_angSlant) && (ln2.SlantAngle() == !m_angSlant))
			return true;
		else if ((ln2.SlantAngle() == m_angSlant) && (ln1.SlantAngle() == !m_angSlant))
			return true;
	}

	return false;
}

//
//   �ж�����ֱ���Ƿ�ƽ�С�
//   fMaxAngDiff: ��������������(����)��
//
bool CLine::IsParallelTo(const CLine& Line, float fMaxAngDiff) const
{
	if (fMaxAngDiff == 0)
		fMaxAngDiff = CAngle::m_fReso;

	CAngle ang1 = Line.SlantAngle();
	if (m_angSlant.ApproxEqualTo(ang1, fMaxAngDiff) || (!m_angSlant).ApproxEqualTo(ang1, fMaxAngDiff))
		return true;
	else
		return false;
}

//
//   �ж�����ֱ���Ƿ�ֱ��
//
bool CLine::IsVerticalTo(const CLine& Line, float fMaxAngDiff) const
{
	if (fMaxAngDiff == 0)
		fMaxAngDiff = CAngle::m_fReso;

	CAngle ang1 = Line.SlantAngle() - PI / 2;
	if ((fabs(m_angSlant.GetDifference(ang1)) < fMaxAngDiff) ||
		(fabs(!m_angSlant.GetDifference(ang1)) < fMaxAngDiff))
		return true;
	else
		return false;
}

//
//   �ж�����ֱ���Ƿ���(fMaxAngDiff��fMaxDistDiff��������ĽǶȼ����������)��
//
bool CLine::IsColinearWith(const CLine& Line, float fMaxAngDiff, float fMaxDistDiff) const
{
	// ���ж��Ƿ�ƽ��
	if (!IsParallelTo(Line, fMaxAngDiff))
		return false;

	// ����Line�������˵㵽��ֱ��(�����߶�!)�ľ���
	float fDist1 = DistanceToPoint(false, Line.m_ptStart);
	float fDist2 = DistanceToPoint(false, Line.m_ptEnd);

	// ������볬�ޣ��򲻹���
	if (fDist1 > fMaxDistDiff || fDist2 > fMaxDistDiff)
		return false;

	return true;
}

//
//   ȡ������ֱ�ߵĽ��㡣
//
bool CLine::IntersectLineAt(const CLine& Line, CPnt& pt, float& fDist) const
{
	CPnt pt1;
	float k1, k2;

	if (IsParallelTo(Line))
		return false;

	// ����1����һ��ֱ��ƽ����Y��
	if (fabs(m_ptEnd.x - m_ptStart.x) < MIN_LINE_LENGTH)
	{
		if (fabs(Line.m_ptEnd.x - Line.m_ptStart.x) < MIN_LINE_LENGTH)
		{
			ASSERT(false);               // ���߾�ƽ����Y�ᣬ�������ڴ˳���
		}
		else
		{
			k2 = (Line.m_ptEnd.y - Line.m_ptStart.y) / (Line.m_ptEnd.x - Line.m_ptStart.x);
			pt1.x = m_ptEnd.x;
			pt1.y = Line.m_ptStart.y + (pt1.x - Line.m_ptStart.x) * k2;
		}
	}

	// ����2���ڶ���ֱ��ƽ����Y��
	else if (fabs(Line.m_ptEnd.x - Line.m_ptStart.x) < MIN_LINE_LENGTH)
	{
		if (fabs(m_ptEnd.x - m_ptStart.x) < MIN_LINE_LENGTH)
		{
			ASSERT(false);               // ���߾�ƽ����Y�ᣬ�������ڴ˳���
		}
		else
		{
			k1 = (m_ptEnd.y - m_ptStart.y) / (m_ptEnd.x - m_ptStart.x);
			pt1.x = Line.m_ptEnd.x;
			pt1.y = m_ptStart.y + (pt1.x - m_ptStart.x) * k1;
		}
	}

	// ����3������ֱ�߾���ƽ����Y��
	else
	{
		k1 = (m_ptEnd.y - m_ptStart.y) / (m_ptEnd.x - m_ptStart.x);
		k2 = (Line.m_ptEnd.y - Line.m_ptStart.y) / (Line.m_ptEnd.x - Line.m_ptStart.x);

		pt1.x = (Line.m_ptStart.y - m_ptStart.y + k1 * m_ptStart.x - k2 * Line.m_ptStart.x) / (k1 - k2);
		pt1.y = m_ptStart.y + (pt1.x - m_ptStart.x) * k1;
	}

	// ������Ҫ���������߶�����
	if (!ContainPoint(pt1) || !Line.ContainPoint(pt1))
		return false;

	pt = pt1;
	fDist = pt1.DistanceTo(m_ptStart);
	return true;
}

//
//   Intersects two lines.  Returns true if intersection point exists,
//   false otherwise.  (*px, *py) will hold intersection point,
//   *onSegment[12] is true if point is on line segment[12].
//   px, py, onSegment[12] might be NULL.
//
bool CLine::Intersect(const CLine& line2, float *px, float *py, bool *onSegment1, bool *onSegment2, float fSmallGate) const
{
	float l1dx, l1dy, l2dx, l2dy, det, ldx1, ldy1, lambda1, lambda2;

	l1dx = m_ptEnd.x - m_ptStart.x;
	l1dy = m_ptEnd.y - m_ptStart.y;
	l2dx = line2.m_ptEnd.x - line2.m_ptStart.x;
	l2dy = line2.m_ptEnd.y - line2.m_ptStart.y;
	det = l1dy * l2dx - l1dx * l2dy;

	if (fabs(det) < fSmallGate)
		return false;

	ldx1 = m_ptStart.x - line2.m_ptStart.x;
	ldy1 = m_ptStart.y - line2.m_ptStart.y;
	lambda1 = (ldx1 * l2dy - ldy1 * l2dx) / (l1dy * l2dx - l1dx * l2dy);
	lambda2 = (ldx1 * l1dy - ldy1 * l1dx) / (l1dy * l2dx - l1dx * l2dy);

	if (px != NULL)
		*px = m_ptStart.x + l1dx * lambda1;

	if (py != NULL)
		*py = m_ptStart.y + l1dy * lambda1;

	if (onSegment1 != NULL)
		*onSegment1 = (lambda1 >= 0.0 && lambda1 <= 1.0);

	if (onSegment2 != NULL)
		*onSegment2 = (lambda2 >= 0.0 && lambda2 <= 1.0);

	return true;
}

//
//   Calculates angle from line1 to line2, counterclockwise, result will be in interval [-PI;PI].
//
CAngle CLine::AngleToLine(const CLine& line2) const
{
	return line2.m_angSlant - m_angSlant;
}

//
//   �����ֱ������һ������ֱ�ߵĽǶȲ�(���������н�С��)��
//
//   ˵����line2Ϊһ���޷���ֱ�ߡ�������������ӱ�����ֱ�߿�ʼ������ʱ�뷽����ת����һ����
//   �޷���ֱ�����һ��ʱ����ת���ĽǶȡ�
//
CAngle CLine::AngleToUndirectionalLine(const CLine& line2) const
{
	CAngle ang1 = line2.m_angSlant - m_angSlant;
	CAngle ang2 = !line2.m_angSlant - m_angSlant;
	if (ang1.m_fRad < ang2.m_fRad)
		return ang1;
	else
		return ang2;
}

//
//   �����ֱ�ߵ�ָ����ľ��롣
//
float CLine::DistanceToPoint(const CPnt& pt) const
{
	return CLineBase::DistanceToPoint(pt);
}

//
//   �����ֱ�ߵ�ָ����ľ��롣
//
float CLine::DistanceToPoint(bool bIsSegment, const CPnt& pt, float* pLambda, CPnt* pFootPoint) const
{
	float dx = m_ptEnd.x - m_ptStart.x;
	float dy = m_ptEnd.y - m_ptStart.y;
	float d2 = dx*dx + dy*dy;

	float lambda = ((pt.y - m_ptStart.y) * dy + (pt.x - m_ptStart.x) * dx) / d2;

	if (pLambda != NULL)
		*pLambda = lambda;

	if (bIsSegment)
	{
		/* make sure point is on line (lambda <- [0..1]) */
		if (lambda < 0)
			lambda = 0;
		else if (lambda > 1)
			lambda = 1.0f;
	}

	float x = m_ptStart.x + lambda * dx;
	float y = m_ptStart.y + lambda * dy;

	if (pFootPoint != NULL)
	{
		pFootPoint->x = x;
		pFootPoint->y = y;
	}

	dx = pt.x - x;
	dy = pt.y - y;

	return (float)sqrt(dx*dx + dy*dy);
}

//
//   �ж�һ�������ĵ��Ƿ񡰴���������ֱ�ߡ�
//   ����ֵ��
//      0 - δ������ֱ��
//      1 - ���������
//      2 - �������յ�
//      3 - �����������˵������ֱ�߲���
//
int CLine::PointHit(const CPnt& pt, float fDistGate)
{
	float Lambda;

	// ����㵽ֱ�ߵ�ͶӰ����
	float fDist = DistanceToPoint(false, pt, &Lambda);

	// ���ͶӰ��λ���߶�֮�⣬����false
	if (Lambda < 0 || Lambda > 1)
		return 0;

	// ������볬�ޣ�Ҳ����false
	else if (fDist > fDistGate)
		return 0;
	else
	{
		if (pt.DistanceTo(m_ptStart) < fDistGate)
			return 1;
		else if (pt.DistanceTo(m_ptEnd) < fDistGate)
			return 2;
		else
			return 3;
	}
}

//
//   ��ֱ����ָ�������ĵ������ת��
//
void CLine::Rotate(float fAng, float fCx, float fCy)
{
	m_ptStart.Rotate(fAng, fCx, fCy);
	m_ptEnd.Rotate(fAng, fCx, fCy);

	Create(m_ptStart, m_ptEnd);
}

//
//   ��ֱ����ָ�������ĵ������ת��
//
void CLine::Rotate(float fAng, CPnt ptCenter)
{
	m_ptStart.Rotate(fAng, ptCenter);
	m_ptEnd.Rotate(fAng, ptCenter);

	Create(m_ptStart, m_ptEnd);
}

//
//   �任���ֲ�����ϵ��
//
CLine CLine::TransformToLocal(CTransform& trans)
{
	CPnt pt1 = trans.GetLocalPoint(m_ptStart);
	CPnt pt2 = trans.GetLocalPoint(m_ptEnd);

	CLine Line(pt1, pt2);
	return Line;
}

//
//   �任��ȫ������ϵ��
//
CLine CLine::TransformToGlobal(CTransform& trans)
{
	CPnt pt1 = trans.GetWorldPoint(m_ptStart);
	CPnt pt2 = trans.GetWorldPoint(m_ptEnd);

	CLine Line(pt1, pt2);
	return Line;
}

//
//   ȡ��ֱ�߶ε�б��k��Y��ؾ�b��(��ֱ�ߴ�ֱ��X��ʱ��)X��ؾ�c��
//   ����ֵ��
//     1 - ֱ�߲���ֱ��X�ᣬk��b����ֵ������, c������
//     0 - ֱ�߲���ֱ��X�ᣬk��b����ֵ��������, c������
//
int CLine::GetParam(float* k, float* b, float* c) const
{
	CLine lineXAxis(CPnt(0, 0), CAngle(0), FLT_MAX);

	// ֱ�߲���ֱ��X�ᣬ����ֱ�ߵ�б�ʼ���Y���ϵĽؾ�
	if (!IsVerticalTo(lineXAxis))
	{
		float K, B;
		
		// ����б��
		if (m_angSlant > PI)
			K = tan(!m_angSlant);
		else
			K = tan(m_angSlant);

		// ����Y��ؾ�
		B = m_ptStart.y - K * m_ptStart.x;

		if (k != NULL)
			*k = K;

		if (b != NULL)
			*b = B;

		return 1;
	}

	// ֱ�ߴ�ֱ��X�ᣬ����ֱ����X���ϵĽؾ�
	else
	{
		if (c != NULL)
			*c = GetMidpoint().x;
		return 0;
	}
}

//
//   ����ֱ�ߵ������˵����ĸ�����ָ���ĵ�pt����������fDist�з��ش˽����롣
//   ����ֵ��
//     0 - pt�������m_ptStart����
//     1 - pt�����յ�m_ptEnd����
//
int CLine::FindNearPoint(const CPnt& pt, float* pDist) const
{
	float f[2];
	
	f[0] = pt.DistanceTo(m_ptStart);
	f[1] = pt.DistanceTo(m_ptEnd);
	
	if (f[0] < f[1])
	{
		if (pDist != NULL)
			*pDist = f[0];
		
		return 0;
	}
	else
	{
		if (pDist != NULL)
			*pDist = f[1];
		
		return 1;
	}
}

//
//   ���㱾�߶ε���һ�߶ε�ͶӰ��
//   ����ֵ��
//       return - �����ͶӰ�ߣ�����true�����򷵻�false
//       lnProj - �õ���ͶӰ�߶�
//       fProjDist - ���߶��е㵽��һ���߶�(another)�ľ���
//       fProjLen  - lnProj�ĳ���
//
bool CLine::GetProjection(const CLine& another, CLine& lnProj, float& fProjDist, float& fProjLen) const
{
	float fLambda1, fLambda2;
	CPnt ptFoot1, ptFoot2;

	// ���㱾�߶ε������˵㵽another��ͶӰ��
	another.DistanceToPoint(false, m_ptStart, &fLambda1, &ptFoot1);
	another.DistanceToPoint(false, m_ptEnd, &fLambda2, &ptFoot2);

	// ���m_ptStart��ͶӰ������another����������
	if (fLambda1 < 0)
	{
		// ���m_ptEnd��ͶӰ������another���������⣬˵��ͶӰ�β�����
		if (fLambda2 < 0)
			return false;

		// ���m_ptEnd��ͶӰ������another��
		else if (fLambda2 <= 1)
			lnProj.Create(another.m_ptStart, ptFoot2);

		// ���m_ptEnd��ͶӰ������another�Ľ�����֮��
		else
			lnProj = another;
	}

	// ���m_ptStart��ͶӰ������another����
	else if (fLambda1 < 1)
	{
		// ���m_ptEnd��ͶӰ������another����������
		if (fLambda2 < 0)
			lnProj.Create(another.m_ptStart, ptFoot1);

		// ���m_ptEnd��ͶӰ������another��
		else if (fLambda2 <= 1)
			lnProj.Create(ptFoot1, ptFoot2);

		// ���m_ptEnd��ͶӰ������another�Ľ�����֮��
		else
			lnProj.Create(ptFoot1, another.m_ptEnd);
	}

	// ���m_ptStart��ͶӰ������another�Ľ���������
	else
	{
		// ���m_ptEnd��ͶӰ������another����������
		if (fLambda2 < 0)
			lnProj = another;

		// ���m_ptEnd��ͶӰ������another��
		else if (fLambda2 <= 1)
			lnProj.Create(another.m_ptEnd, ptFoot2);

		// ���m_ptEnd��ͶӰ������another�Ľ�����֮��
		else
			return false;
	}

	fProjDist = another.DistanceToPoint(false, GetMidpoint());
	fProjLen = lnProj.Length();

	return true;
}

//
//   �����������任��
//
void CLine::Transform(const CFrame& frame)
{
	CPnt pt1 = m_ptStart;
	CPnt pt2 = m_ptEnd;

	// �ֱ�������˵��������任
	pt1.Transform(frame);
	pt2.Transform(frame);

	// ��������ֱ�߶�
	Create(pt1, pt2);
}

//
//   ����������任��
//
void CLine::InvTransform(const CFrame& frame)
{
	CPnt pt1 = m_ptStart;
	CPnt pt2 = m_ptEnd;

	// �ֱ�������˵��������任
	pt1.InvTransform(frame);
	pt2.InvTransform(frame);

	// ��������ֱ�߶�
	Create(pt1, pt2);
}

#ifdef _MFC_VER

//
//   ����Ļ�ϻ��ƴ�ֱ�ߡ�
//
void CLine::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bBigVertex, int nPenStyle)
{
	CPen pen(nPenStyle, nWidth, crColor);
	CPen* pOldPen = pDC->SelectObject(&pen);

	CPnt pt1(m_ptStart.x, m_ptStart.y);
	CPnt pt2(m_ptEnd.x, m_ptEnd.y);

	CPoint pnt1 = ScrnRef.GetWindowPoint(pt1);
	CPoint pnt2 = ScrnRef.GetWindowPoint(pt2);

	pDC->MoveTo(pnt1);
	pDC->LineTo(pnt2);

	pDC->SelectObject(pOldPen);

	if (bBigVertex)
	{
		m_ptStart.Draw(ScrnRef, pDC, crColor, nPointSize * 2);
		m_ptEnd.Draw(ScrnRef, pDC, crColor, nPointSize * 2);
	}
}

#elif defined QT_VERSION
//
//   ����Ļ�ϻ��ƴ�ֱ�ߡ�
//
void CLine::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth, int nPointSize, bool bBigVertex, int nPenStyle)
{
	QPen pen(crColor);
	pen.setWidth(nWidth);
	pPainter->setPen(pen);

	CPnt pt1(m_ptStart.x, m_ptStart.y);
	CPnt pt2(m_ptEnd.x, m_ptEnd.y);

	QPoint pnt1 = ScrnRef.GetWindowPoint(pt1);
	QPoint pnt2 = ScrnRef.GetWindowPoint(pt2);

	pPainter->drawLine(pnt1, pnt2);

	if (bBigVertex)
	{
		m_ptStart.Draw(ScrnRef, pPainter, crColor, nPointSize * 2);
		m_ptEnd.Draw(ScrnRef, pPainter, crColor, nPointSize * 2);
	}
}
#endif
