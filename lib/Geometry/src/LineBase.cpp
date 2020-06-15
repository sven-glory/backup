#include "stdafx.h"
#include <math.h>
#include "Geometry.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ��бʽ������
//
CLineBase::CLineBase(const CPnt& pt, float fAng)
{
	Create(pt.x, pt.y, fAng);
}

//
//   ��бʽ����ֱ�ߡ�
//
void CLineBase::Create(float x, float y, float fAng)
{
	float cos_ = (float)cos(fAng);
	float sin_ = (float)sin(fAng);

	a = sin_;
	b = -cos_;
	c = -sin_ * x + cos_ * y;

	// ��a����Ϊ����
	if (a < 0)
	{
		a = -a;
		b = -b;
		c = -c;
	}

	// �����б�׼��
	float fNorm = (float)sqrt(a * a + b * b);
	a /= fNorm;
	b /= fNorm;
	c /= fNorm;
}

//
//   ͨ���Ե��Ƶ����Իع������������ߡ�
//   ˵����
//     p      - �������黺����ָ��
//     nCount - �������
//
bool CLineBase::CreateFitLine(const CPnt* sp, int num)
{
	if (num <= 1)
		return false;              // ����̫�٣��޷����ֱ��

	float xm = 0, ym = 0, xxm = 0, yym = 0, xym = 0;
	for (int i = 0; i < num; i++)
	{
		float x = sp[i].x;
		float y = sp[i].y;

		xm += x;
		ym += y;
		xxm += x * x;
		yym += y * y;
		xym += x * y;
	}

	float a = (float)atan2(-2 * (xym - xm * ym / num), yym - ym * ym / num - xxm + xm * xm / num) / 2;
	float n1 = (float)cos(a);
	float n2 = (float)sin(a);
	float dx = sp[num - 1].x - sp[0].x;
	float dy = sp[num - 1].y - sp[0].y;

	if (dx * n2 - dy * n1 > 0)
	{
		n1 = -n1;
		n2 = -n2;
	}

	float c = -(n1 * xm + n2 * ym) / num;

	DirectCreate(n1, n2, c);
	return true;
}

//
//   �����ֱ�ߵ�ָ����ľ��롣
//
//   ˵������(x0, y0)��ֱ��(a*x + b*y + c = 0)���빫ʽΪ��
//     d = abs(a*x0 + b*y0 + c)/sqrt(a*a + b*b)
//
float CLineBase::DistanceToPoint(const CPnt& pt) const
{
	return (float)(fabs(a * pt.x + b * pt.y + c) / sqrt(a*a + b*b));
}

//
//   ���ֱ����һ�㵽��ֱ�ߵ�ͶӰ��
//
CPnt CLineBase::GetProjectPoint(const CPnt& pt) const
{
	CPnt ptFoot;
	ptFoot.x = (b*b*pt.x - a*b*pt.y - a*c) / (a*a + b*b);
	ptFoot.y = (-a*b*pt.x + a*a*pt.y - b*c) / (a*a + b*b);

	return ptFoot;
}

//
//   ��һֱ�߶ε���ֱ�ߵ�ͶӰ��
//
CLine CLineBase::GetProjectLine(const CLine& Line) const
{
	CPnt pt1 = GetProjectPoint(Line.m_ptStart);
	CPnt pt2 = GetProjectPoint(Line.m_ptEnd);

	CLine lnProject(pt1, pt2);
	return lnProject;
}

//
//   ȡ��ֱ�ߵ���б��(����)��
//   ע��
//      ����CLineBase�������޳�ֱ�ߣ���Ҳû�з������Ա��������صĽǶ�ֵ��[0, PI)֮�ڡ�
//
float CLineBase::StdSlantAngleRad() const
{
	float fAng = (float)atan2(-a, b);
	if (fAng < 0)
		fAng += PI;

	return fAng;
}
