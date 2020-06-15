//                          - POLYREGION.CPP -
//
//   Implementation of class "CLine" - which defines the geometric concept
//   "Directional Straight Line".
//
//   Author: Zhang Lei
//   Date:   2000. 4. 24
//

#include "stdafx.h"
#include <math.h>
//#include "Tools.h"
#include "Geometry.h"
#include "ScrnRef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

CPolyRegion::CPolyRegion(int nCount, CPnt* pPnt)
{
	m_pVertex = new CPnt[nCount];
	m_nCount = nCount;

	if (pPnt == NULL)
		return;

	for (int i = 0; i < nCount; i++)
		m_pVertex[i] = pPnt[i];

}

CPolyRegion::~CPolyRegion()
{
	if (m_pVertex != NULL)
		delete []m_pVertex;
}

//
//   �ж�ָ���ĵ��Ƿ�����ڴ˶���������С�
//
bool CPolyRegion::Contain(const CPnt& pt) const
{
	int i, j;
	bool bResult = false;

	for (i = 0, j = m_nCount-1; i < m_nCount; j = i++)
	{
		if ((m_pVertex[i].y < pt.y && m_pVertex[j].y >= pt.y || m_pVertex[j].y < pt.y && m_pVertex[i].y >= pt.y) &&  
		   (m_pVertex[i].x <= pt.x || m_pVertex[j].x <= pt.x))
		{
			if (m_pVertex[i].x + (pt.y-m_pVertex[i].y) / (m_pVertex[j].y - m_pVertex[i].y) * (m_pVertex[j].x - m_pVertex[i].x) 
				< pt.x)
	         bResult = !bResult;
		}
	}

	return bResult;
}

//
//   �жϴ������Ƿ��������һ��ָ��������
//
bool CPolyRegion::Contain(const CPolyRegion& PolyRgn) const
{
	for (int i = 0; i < PolyRgn.m_nCount; i++)
		if (!Contain(PolyRgn.m_pVertex[i]))
			return false;

	return true;
}

//
//   �жϴ������Ƿ�������һ��ָ�����������ص�����
//
bool CPolyRegion::OverlapWith(const CPolyRegion& PolyRgn) const
{
	int i;

	for (i = 0; i < PolyRgn.m_nCount; i++)
		if (Contain(PolyRgn.m_pVertex[i]))
			return true;

	for (i = 0; i < m_nCount; i++)
		if (PolyRgn.Contain(m_pVertex[i]))
			return true;

	return false;
}

#ifdef _MFC_VER

//
//   ����Ļ�ϻ��ƴ˶��������
//
void CPolyRegion::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth, int nPointSize, bool bBigVertex)
{
}
#endif
