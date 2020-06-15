//                          - SCPPATH.CPP -
//
//   Implementation of class "CScpPath" - a class defining a SCP path in
//   AGVS map.
//
//   Author: Zhang Lei
//   Date:   2000. 10. 28
//

#include "stdafx.h"

#include <stdlib.h>
#include "Tools.h"
#include "ScpPath.h"
   
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define SCP_DRAWING_POINTS     50

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CScpPath".

CScpPath::CScpPath()
{
	m_pTraj = NULL;
}

CScpPath::~CScpPath()
{
	if (m_pTraj != NULL)
		delete m_pTraj;
}

//
//   Size caculation function.
//
float CScpPath::SizeFun()
{
	// Construct a line to find out the leng of the path
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	CLine ln(ptStart, ptEnd);
	return (float)fabs(ln.Length() * cos(ln.SlantAngle() - m_angHeading));
}

//
//   Make a trajectory from the path.
//
CTraj* CScpPath::MakeTraj()
{
	CScpTraj* pScpTraj = new CScpTraj;
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pScpTraj->CreateTraj(ptStart, ptEnd, m_angHeading);
	return pScpTraj;
}

bool CScpPath::Create(FILE *StreamIn)
{
	if (!CSidePath::Create(StreamIn))
		return false;

	m_pTraj = (CScpTraj*)MakeTraj();
	return (m_pTraj != NULL);
}

#ifdef _MFC_VER
bool CScpPath::Create(CArchive& ar)
{
	if (!CSidePath::Create(ar))
		return false;

	m_pTraj = (CScpTraj*)MakeTraj();
	return (m_pTraj != NULL);
}

void CScpPath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	CPoint pnt1 = ScrnRef.GetWindowPoint(GetStartPnt());
	CPoint pnt2 = ScrnRef.GetWindowPoint(GetEndPnt());;

   CPen Pen(PS_SOLID, nWidth, cr);
   CPen* pOldPen;
   pOldPen = pDc->SelectObject(&Pen);

	// ���
	pDc->MoveTo(pnt1);

	float fRange = m_pTraj->GetRange();
	float fRate = fRange/SCP_DRAWING_POINTS;
	float fProgress;

	// �������
	for (int i = 0; i < SCP_DRAWING_POINTS; i++)
	{
		// ����켣��
		fProgress = i * fRate;
		m_pTraj->SetProgress(fRate, fProgress);
		CPnt pt = m_pTraj->TrajFun();

		// ת������������
		CPoint pnt = ScrnRef.GetWindowPoint(pt);

		// ����
		pDc->LineTo(pnt);
	}

	// �����յ�
	pDc->LineTo(pnt2);

   pDc->SelectObject(pOldPen);
}
#endif
