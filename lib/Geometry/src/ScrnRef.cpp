//                           - SCRNREF.CPP -
//
//   Implementation of class "CScreenReference".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 26
//

#include "stdafx.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPoint>
#endif

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CScreenReference".

CScreenReference::CScreenReference(USHORT uWidth, USHORT uHeight, float fRatio,
											  const CPnt& ptCenter)
{
	m_uWidth = uWidth;
	m_uHeight = uHeight;
	m_fRatio = fRatio;
	m_ptCenter = ptCenter;
}

CScreenReference::CScreenReference()
{
	m_uWidth = 1;
	m_uHeight = 1;
	m_fRatio = 1.0;
	m_ptCenter.x = 0;
	m_ptCenter.y = 0;
}

//
//   Set the pointer that will map to the center of the view port.
//
void CScreenReference::SetCenterPoint(const CPnt& pt)
{
	m_ptCenter = pt;
}

//
//   �����ӿڵ����Ͻǵ�����Ӧ�������λ�á�
//
void CScreenReference::SetLeftTopPoint(const CPnt& pt)
{
	m_ptCenter.x = pt.x + m_uWidth/m_fRatio/2;
	m_ptCenter.y = pt.y - m_uHeight/m_fRatio/2;
}

//
//   ������ʾ�����ߡ�
//
void CScreenReference::SetRatio(float fRatio)
{
	m_fRatio = fRatio;
}

//
//   �����������ķ�Χ��
//
void CScreenReference::SetWorldRange(const CPnt& ptWorldLeftTop, const CPnt& ptWorldRightBottom)
{
	m_ptWorldLeftTop = ptWorldLeftTop;
	m_ptWorldRightBottom = ptWorldRightBottom;
}

//
//   �����ӿڵĴ�С��
//
void CScreenReference::SetViewPort(USHORT uWidth, USHORT uHeight)
{
	m_uWidth = uWidth;
	m_uHeight = uHeight;
}

#ifdef _MFC_VER

//
//   �����ӿ���һ�㵽��������ϵ��ĳһ��Ķ�Ӧ��ϵ��
//
void CScreenReference::SetPointMapping(const CPoint& ptWindow, const CPnt& ptWorld)
{
	// �����ӿ����ĵ��Ӧ�����������
	int dx = ptWindow.x - m_uWidth / 2;
	int dy = ptWindow.y - m_uHeight / 2;

	m_ptCenter.x = ptWorld.x - dx / m_fRatio;
	m_ptCenter.y = ptWorld.y + dy / m_fRatio;

#if 0
	// ���������ӿڵĴ�С
	m_nVPortWidth = GetWorldWidth() * m_fRatio;
	m_nVPortHeight = GetWorldHeight() * m_fRatio;
#endif

	// ������ʵ�ӿڵ����ϽǴ�����Ӧ�������ӿ��е������
	m_nLeft = 0;
	m_nTop = 0;
}

//
//   ȡ���ӿ���ָ��λ������Ӧ�����������ꡣ
//
CPnt CScreenReference::GetWorldPoint(const CPoint& pntWindow) const
{
	CPnt pt;

	CPnt ptLeftTop = GetLeftTopPoint();
	pt.x = (float)( pntWindow.x/m_fRatio + ptLeftTop.x);
	pt.y = (float)(-pntWindow.y/m_fRatio + ptLeftTop.y);

	return pt;
}

//
//   ȡ��ָ�������������Ӧ���ӿڵ��λ�á�
//
CPoint CScreenReference::GetWindowPoint(const CPnt& ptWorld) const
{
	CPoint pnt;

	CPnt ptLeftTop = GetLeftTopPoint();
	pnt.x = (int)((ptWorld.x - ptLeftTop.x) * m_fRatio + 0.5f);
	pnt.y = (int)((ptLeftTop.y - ptWorld.y) * m_fRatio + 0.5f);
	return pnt;
}

#elif defined QT_VERSION

//
//   �����ӿ���һ�㵽��������ϵ��ĳһ��Ķ�Ӧ��ϵ��
//
void CScreenReference::SetPointMapping(const QPoint& ptWindow, const CPnt& ptWorld)
{
	// �����ӿ����ĵ��Ӧ�����������
	int dx = ptWindow.x() - m_uWidth / 2;
	int dy = ptWindow.y() - m_uHeight / 2;

	m_ptCenter.x = ptWorld.x - dx / m_fRatio;
	m_ptCenter.y = ptWorld.y + dy / m_fRatio;

#if 0
	// ���������ӿڵĴ�С
	m_nVPortWidth = GetWorldWidth() * m_fRatio;
	m_nVPortHeight = GetWorldHeight() * m_fRatio;
#endif

	// ������ʵ�ӿڵ����ϽǴ�����Ӧ�������ӿ��е������
	m_nLeft = 0;
	m_nTop = 0;
}

//
//   ȡ���ӿ���ָ��λ������Ӧ�����������ꡣ
//
CPnt CScreenReference::GetWorldPoint(const QPoint& pntWindow) const
{
	CPnt pt;

	CPnt ptLeftTop = GetLeftTopPoint();
	pt.x = (float)( pntWindow.x()/m_fRatio + ptLeftTop.x);
	pt.y = (float)(-pntWindow.y()/m_fRatio + ptLeftTop.y);

	return pt;
}

//
//   ȡ��ָ�������������Ӧ���ӿڵ��λ�á�
//
QPoint CScreenReference::GetWindowPoint(const CPnt& ptWorld) const
{
	QPoint pnt;

	CPnt ptLeftTop = GetLeftTopPoint();
	pnt.setX((int)((ptWorld.x - ptLeftTop.x) * m_fRatio + 0.5f));
	pnt.setY((int)((ptLeftTop.y - ptWorld.y) * m_fRatio + 0.5f));
	return pnt;
}
#endif

//
//   ȡ���ӿ����ϽǴ�����Ӧ�����������ꡣ
//
CPnt CScreenReference::GetLeftTopPoint() const
{
	CPnt pt;
	pt.x = m_ptCenter.x - m_uWidth / (2 * m_fRatio);
	pt.y = m_ptCenter.y + m_uHeight / (2 * m_fRatio);
	return pt;
}

