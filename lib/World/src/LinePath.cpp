//                              - LINEPATH.CPP -
//
//   Implementation of class "CLinePath" - a class defining a line path in
//   AGVS map.
//
//   Author: Zhang Lei
//   Date:   2001. 7. 30
//
                 
#include "stdafx.h"
#include <stdio.h>
#include "LinePath.h"
#include "Tools.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CLinePath".

//
//   CLinePath: The constructor.
//

CLinePath::CLinePath(USHORT uId, USHORT uStartNode, USHORT uEndNode, 
							float fVeloLimit, SHORT nGuideType,	USHORT uObstacle, 
							USHORT uDir, USHORT uExtType) :
CPath(uId, uStartNode, uEndNode, fVeloLimit, LINE_TYPE, nGuideType, uExtType)
{
	m_uObstacle = uObstacle;

	if (uDir == NEGATIVE_HEADING)
	{
		USHORT uTemp = m_uStartNode;
		m_uStartNode = m_uEndNode;
		m_uEndNode = uTemp;
		SwabWord(m_uObstacle);
	}

	Setup();
}

//
//
//   Setup: Find the vehicle's headings at the 2 nodes.
void CLinePath::Setup()
{
	// Do initializations
	// Get the path's start/end points
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();

	// Construct a line
	CLine ln(ptStart, ptEnd);

	// The start and end heading angle are the same
	m_angHeading = ln.SlantAngle();
	m_fSize = ln.Length();
}

//
//   GetHeading: Determine the vehicle's heading angle at the specified
//   node.
//
//   Note: MAKE SURE THAT "nd" IS A VERTEX OF THE PATH !
//
CAngle& CLinePath::GetHeading(CNode& /*nd*/)
{
	return m_angHeading;
}

//
//   Make a trajectory from the path.
//
CTraj* CLinePath::MakeTraj()
{
	CLineTraj* pLineTraj = new CLineTraj;
	
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pLineTraj->CreateTraj(ptStart, ptEnd, FORWARD);
	return pLineTraj;
}	

bool CLinePath::Create(FILE *StreamIn)
{
	unsigned int uObstacle, uDir;            // Positive input/negative input

	// Init the fields in the base class
	if (!CPath::Create(StreamIn))
		return false;

   if (fscanf(StreamIn, ",\t%u,\t%u\n", &uObstacle, &uDir) == EOF)
      return false;

	m_uObstacle = uObstacle;
	if (uDir == NEGATIVE_HEADING)
	{
		USHORT uTemp = m_uStartNode;
		m_uStartNode = m_uEndNode;
		m_uEndNode = uTemp;

		SwabWord(m_uObstacle);
	}

	// Find the heading angles at the 2 nodes
	Setup();
	return true;
}

bool CLinePath::Save(FILE *StreamOut)
{
	// Init the fields in the base class
	if (!CPath::Save(StreamOut))
		return false;

	USHORT uHeading = POSITIVE_HEADING;
   if (fprintf(StreamOut, ",\t%u,\t%u\n", m_uObstacle, uHeading) == EOF)
      return false;

	return true;
}

#ifdef _MFC_VER

bool CLinePath::Create(CArchive& ar)
{
	USHORT uDir;            // Positive input/negative input

	// Init the fields in the base class
	if (!CPath::Create(ar))
		return false;

	ar >> m_uFwdRotoScannerObstacle >> m_uFwdObdetectorObstacle >> m_uBwdRotoScannerObstacle >> m_uBwdObdetectorObstacle >> uDir;

	if (uDir == NEGATIVE_HEADING)
	{
		USHORT uTemp = m_uStartNode;
		m_uStartNode = m_uEndNode;
		m_uEndNode = uTemp;

		SwabWord(m_uObstacle);
	}

	// Find the heading angles at the 2 nodes
	Setup();

	return true;
}

bool CLinePath::Save(CArchive& ar)
{
	// Init the fields in the base class
	if (!CPath::Save(ar))
		return false;

	USHORT uHeading = POSITIVE_HEADING;
	ar << m_uFwdRotoScannerObstacle << m_uFwdObdetectorObstacle << m_uBwdRotoScannerObstacle << m_uBwdObdetectorObstacle << uHeading;

	return true;
}

void CLinePath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	CPoint pnt1 = ScrnRef.GetWindowPoint(GetStartPnt());
	CPoint pnt2 = ScrnRef.GetWindowPoint(GetEndPnt());

	CPen Pen(PS_SOLID, nWidth, cr);
	CPen* pOldPen = pDc->SelectObject(&Pen);

  	pDc->MoveTo(pnt1);
	pDc->LineTo(pnt2);

	pDc->SelectObject(pOldPen);
}

//
//   �жϸ�������Ļ���Ƿ�����·���ϡ�
//   ����ֵ��
//     -1:  δ����·����
//     0:   ����·����
//
int CLinePath::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
	// ȡ�ö�Ӧ�����������
	CPnt pt = ScrnRef.GetWorldPoint(pnt);
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();

	// ����ֱ��
	CLine ln(ptStart, ptEnd);

	float fLambda;
	CPnt ptFoot;

	// ����㵽��ֱ�ߵ���̾���
	float fDist = ln.DistanceToPoint(false, pt, &fLambda, &ptFoot);
	if (fLambda >= 0 && fLambda <= 1)
	{
		// ���㵽��Ļ���ھ���
		int nDist = (int)(fDist * ScrnRef.m_fRatio);

		// �����Ļ���ھ���С��3����Ϊ��괥����·��
		if (nDist <= 3)
			return 0;               // ����
	}

	return -1;          // ���
}
#endif
