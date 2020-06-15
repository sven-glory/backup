//                          - RECTANGLE.CPP -
//
//   Implementation of class "CRectangle" - which defines the geometric concept
//   "Rectangle".
//
//   Author: Zhang Lei
//   Date:   2015. 4. 11
//

#include "stdafx.h"
#include <assert.h>
#include <math.h>
//#include "Tools.h"
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

/////////////////////////////////////////////////////////////////////////////
//   Implementation of the class.

//
//   CRectangle: The constructor form #1.
//
CRectangle::CRectangle(const CPnt& ptLeftTop, const CPnt& ptRightBottom)
{
	// �����������ĺϷ���
	if (ptLeftTop.x > ptRightBottom.x || ptLeftTop.y < ptRightBottom.y)
		assert(false);

	Create(ptLeftTop, ptRightBottom);
}

CRectangle::CRectangle(float fLeft, float fTop, float fRight, float fBottom)
{
	if (!Create(fLeft, fTop, fRight, fBottom))
		assert(false);
}

CRectangle::CRectangle()
{
	Clear();
}

//
//   ���ɾ��Ρ�
//
bool CRectangle::Create(const CPnt& ptLeftTop, const CPnt& ptRightBottom)
{
	// �����������ĺϷ���
	if (ptLeftTop.x > ptRightBottom.x || ptLeftTop.y < ptRightBottom.y)
		return false;

	m_ptLeftTop = ptLeftTop;
	m_ptRightBottom = ptRightBottom;
	m_bInit = true;

	return true;
}

//
//   ���ɾ��Ρ�
//
bool CRectangle::Create(float fLeft, float fTop, float fRight, float fBottom)
{
	// �����������ĺϷ���
	if (fLeft > fRight || fTop < fBottom)
		return false;

	m_ptLeftTop.Set(fLeft, fTop);
	m_ptRightBottom.Set(fRight, fBottom);
	m_bInit = true;

	return true;
}

//
//   �������
//
void CRectangle::Clear()
{
	Create(0, 0, 0, 0);
	m_bInit = false;
}

//
//   ȡ�����ĵ��λ�á�
//
CPnt CRectangle::GetCenterPoint() const
{
	CPnt ptCenter;
	ptCenter.x = (m_ptLeftTop.x + m_ptRightBottom.x) / 2;
	ptCenter.y = (m_ptLeftTop.y + m_ptRightBottom.y) / 2;

	return ptCenter;
}

//
//   �ж�һ�����Ƿ��ھ������ڡ�
//
bool CRectangle::Contain(float x, float y) const
{
	if (x >= m_ptLeftTop.x && x <= m_ptRightBottom.x &&
		y >= m_ptRightBottom.y && y <= m_ptLeftTop.y)
		return true;
	else
		return false;
}

//
//   �ж�һ�����Ƿ��ڴ˾����ڡ�
//
bool CRectangle::Contain(const CPnt& pt) const
{
	if (pt.x >= m_ptLeftTop.x && pt.x <= m_ptRightBottom.x &&
		 pt.y >= m_ptRightBottom.y && pt.y <= m_ptLeftTop.y)
		return true;
	else
		return false;
}

//
//   �ж�һ�߶��Ƿ��ھ������ڡ�
//
bool CRectangle::Contain(const CLine& ln) const
{
	if (Contain(ln.m_ptStart) && Contain(ln.m_ptEnd))
		return true;
	else
		return false;
}

//
//   �ж���һ�������Ƿ��ڸþ������ڡ�
//
bool CRectangle::Contain(const CRectangle& r) const
{
	if (Contain(r.m_ptLeftTop) && Contain(r.m_ptRightBottom))
		return true;
	else
		return false;
}

//
//   ȡ�þ�������Ŀ�ȡ�
//
float CRectangle::Width() const
{
	return m_ptRightBottom.x - m_ptLeftTop.x;
}

//
//   ȡ�þ�������ĸ߶ȡ�
//
float CRectangle::Height() const
{
	return m_ptLeftTop.y - m_ptRightBottom.y;
}

//
//   �������������С�Ա����ɸ����ĵ㡣
//
void CRectangle::operator += (const CPnt& pt)
{
	if (!m_bInit)
	{
		m_ptLeftTop = m_ptRightBottom = pt;
		m_bInit = true;
		return;
	}

	if (pt.x < m_ptLeftTop.x)
		m_ptLeftTop.x = pt.x;

	if (pt.x > m_ptRightBottom.x)
		m_ptRightBottom.x = pt.x;

	if (pt.y < m_ptRightBottom.y)
		m_ptRightBottom.y = pt.y;

	if (pt.y > m_ptLeftTop.y)
		m_ptLeftTop.y = pt.y;
}

//
//   �������������С�Ա����ɸ�����ֱ�ߡ�
//
void CRectangle::operator += (const CLine& line)
{
	*this += line.m_ptStart;
	*this += line.m_ptEnd;
}

//
//   �������������С�Ա����ɸ����ľ�������
//
void CRectangle::operator += (const CRectangle& rect)
{
	*this += rect.GetLeftTopPoint();
	*this += rect.GetRightBottomPoint();
}

//
//   ���ļ�װ��������ݡ�
//
bool CRectangle::Load(FILE* fp)
{
	if (fscanf(fp, "%f\t%f\t%f\t%f\n", &m_ptLeftTop.x, &m_ptLeftTop.y, &m_ptRightBottom.x, &m_ptRightBottom.y) != 4)
		return false;

	return true;
}

//
//   ����������д���ļ���
//
bool CRectangle::Save(FILE* fp)
{
	fprintf(fp, "%f\t%f\t%f\t%f\n", m_ptLeftTop.x, m_ptLeftTop.y, m_ptRightBottom.x, m_ptRightBottom.y);
	return true;
}

#ifdef _MSC_VER

//
//   ����Ļ�ϻ��ƴ˾��Ρ�
//
void CRectangle::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
{
	CPnt ptRightTop(m_ptRightBottom.x, m_ptLeftTop.y);
	CPnt ptLeftBottom(m_ptLeftTop.x, m_ptRightBottom.y);

	CLine ln1(m_ptLeftTop, ptRightTop);
	CLine ln2(m_ptLeftTop, ptLeftBottom);
	CLine ln3(ptLeftBottom, m_ptRightBottom);
	CLine ln4(ptRightTop, m_ptRightBottom);

	ln1.Draw(ScrnRef, pDC, crColor, nWidth);
	ln2.Draw(ScrnRef, pDC, crColor, nWidth);
	ln3.Draw(ScrnRef, pDC, crColor, nWidth);
	ln4.Draw(ScrnRef, pDC, crColor, nWidth);
}

#elif defined QT_VERSION

//
//   ����Ļ�ϻ��ƴ˾��Ρ�
//
void CRectangle::Draw(CScreenReference& ScrnRef, QPainter* pPainter, QColor crColor, int nWidth)
{
	CPnt ptRightTop(m_ptRightBottom.x, m_ptLeftTop.y);
	CPnt ptLeftBottom(m_ptLeftTop.x, m_ptRightBottom.y);

	CLine ln1(m_ptLeftTop, ptRightTop);
	CLine ln2(m_ptLeftTop, ptLeftBottom);
	CLine ln3(ptLeftBottom, m_ptRightBottom);
	CLine ln4(ptRightTop, m_ptRightBottom);

	ln1.Draw(ScrnRef, pPainter, crColor, nWidth);
	ln2.Draw(ScrnRef, pPainter, crColor, nWidth);
	ln3.Draw(ScrnRef, pPainter, crColor, nWidth);
	ln4.Draw(ScrnRef, pPainter, crColor, nWidth);
}

#endif
