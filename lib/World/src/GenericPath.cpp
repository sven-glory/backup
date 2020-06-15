//                          - GENERICPATH.CPP -
//
//   Implementation of class "CGenericPath" - a class defining a generic path in
//   AGVS map.
//
//
    
#include "stdafx.h"
#include <stdlib.h>
#include "Tools.h"
#include "GenericPath.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define BEZIER_K                     0.95f


//////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CGenericPath".

// ���캯��
CGenericPath::CGenericPath(int uId, int nStartNode, int nEndNode, CPosture& pstStart, 
									CPosture& pstEnd, float fLen1, float fLen2, 
									float fVeloLimit, SHORT nGuideType,	USHORT uObstacle,
									USHORT uDir, USHORT uExtType) :
CPath(uId, nStartNode, nEndNode, fVeloLimit, GENERIC_TYPE, nGuideType, uExtType)
{
	Create(pstStart, pstEnd, fLen1, fLen2);
}

//
//   ȱʡ�Ĺ��캯����
//
CGenericPath::CGenericPath()
{
	m_nCountCtrlPoints = 0;
	m_pptCtrl = NULL;
}

//
//   ����������
//
CGenericPath::~CGenericPath()
{
	if (m_pptCtrl != NULL)
		delete []m_pptCtrl;
}

//
//   ��������ױ��������ߡ�
//
bool CGenericPath::Create(CPosture& pstStart, CPosture& pstEnd, int nCountCtrlPoints, CPnt* pptCtrl)
{
	m_pstStart = pstStart;
	m_pstEnd = pstEnd;
	m_nCountCtrlPoints = nCountCtrlPoints;

	// Ϊ�ؼ������ռ�
	m_pptCtrl = new CPnt[m_nCountCtrlPoints];
	if (m_pptCtrl == NULL)
		return false;

	// ���ƿ��Ƶ�����
	for (int i = 0; i < m_nCountCtrlPoints; i++)
		m_pptCtrl[i] = pptCtrl[i];

	// ��ʼ��·��
	return Init();
}

//
//   �������ױ���������·����
//
bool CGenericPath::Create(CPosture& pstStart, CPosture& pstEnd, float fLen1, float fLen2)
{
	m_pstStart = pstStart;
	m_pstEnd = pstEnd;
	m_nCountCtrlPoints = 2;

	// Ϊ�ؼ������ռ�
	m_pptCtrl = new CPnt[m_nCountCtrlPoints];
	if (m_pptCtrl == NULL)
		return false;

	CLine ln1(pstStart, fLen1);
	m_pptCtrl[0] = ln1.GetEndPoint();

	CLine ln2(pstEnd, !pstEnd.GetAngle(), fLen2);
	m_pptCtrl[1] = ln2.GetEndPoint();

	Init();

	return true;
}

//
//   ����Բ��·������Bezier����·�Ρ�
//
bool CGenericPath::CreateFromArcPath(CArcPath& ArcPath)
{
	CPnt& pt1 = ArcPath.GetStartPnt();
	CPnt& pt2 = ArcPath.GetEndPnt();
	CPnt& ptCenter = ArcPath.GetCenter();

	CLine ln1(ptCenter, pt1);
	CLine ln2(ptCenter, pt2);

	// Բ�ĵ���ʼ/��ֹ��ķ����
	CAngle ang1 = ln1.SlantAngle();
	CAngle ang2 = ln2.SlantAngle();

	// ��ʼ�����ֹ�㴦�ķ����
	CAngle angStart, angEnd;

	if (ArcPath.GetTurnDir() == COUNTER_CLOCKWISE)
	{
		angStart = ang1 + PI / 2;
		angEnd = ang2 + PI / 2;
	}
	else
	{
		angStart = ang1 - PI / 2;
		angEnd = ang2 - PI / 2;
	}

	m_pstStart.GetPntObject() = pt1;
	m_pstStart.SetAngle(angStart);

	m_pstEnd.GetPntObject() = pt2;
	m_pstEnd.SetAngle(angEnd);

	CAngle angTurn = ArcPath.GetTurnAngle();
	float fLen = ArcPath.GetRadius() * tan(angTurn.m_fRad / 2);
	fLen *= BEZIER_K;

	// �������ױ���������·��
	if (!Create(m_pstStart, m_pstEnd, fLen, fLen))
		return false;

	m_uId = ArcPath.m_uId;
	m_uType = GENERIC_TYPE;
	m_uExtType = 0;
	m_uStartNode = ArcPath.m_uStartNode;
	m_uEndNode = ArcPath.m_uEndNode;
	m_fVeloLimit[0] = ArcPath.m_fVeloLimit[0];
	m_fVeloLimit[1] = ArcPath.m_fVeloLimit[1];
	m_uGuideType = ArcPath.m_uGuideType;
	m_fNavParam = ArcPath.m_fNavParam;
	m_uObstacle = ArcPath.m_uObstacle;
	m_uFwdRotoScannerObstacle = ArcPath.m_uFwdRotoScannerObstacle;
	m_uFwdObdetectorObstacle = ArcPath.m_uFwdObdetectorObstacle;
	m_uBwdRotoScannerObstacle = ArcPath.m_uBwdRotoScannerObstacle;
	m_uBwdObdetectorObstacle = ArcPath.m_uBwdObdetectorObstacle;
}

//
//   �������ṩ�Ĺؼ���������λ�ó�ʼ����·����
//
bool CGenericPath::Init()
{
	// ��ʱΪ���йؼ������ռ䣬�Ա��������߶���
	CPnt* pptKey = new CPnt[m_nCountCtrlPoints + 2];
	if (pptKey == NULL)
		return false;

	// ��һ�������һ���ؼ���ʵ������·���Ķ˽ڵ�
	pptKey[0] = m_pstStart.GetPntObject();
	pptKey[m_nCountCtrlPoints + 1] = m_pstEnd.GetPntObject();

	// ���Ƴ���ʼ�ڵ㡢��ֹ�ڵ�֮��Ĺؼ���
	for (int i = 0; i < m_nCountCtrlPoints; i++)
		pptKey[i + 1] = m_pptCtrl[i];

	// �������߶���
	m_Curve.Create(m_nCountCtrlPoints + 2, pptKey);

	// �ͷ���ʱ�ռ�
	delete []pptKey;

	// ��ʼ����㡢�յ㴦�ķ����
	m_angStartHeading = m_pstStart.GetAngle();
	m_angEndHeading = m_pstEnd.GetAngle();

	return true;
}

//
//   GetHeading: Get the vehicle's heading angle at the specified node.
//
CAngle& CGenericPath::GetHeading(CNode& nd)
{
	if (nd == m_uStartNode)
		return m_angStartHeading;
	else
		return m_angEndHeading;
}

//
//   Make a trajectory from the path.
//
CTraj* CGenericPath::MakeTraj()
{
#if 0
	CBezierTraj* pBezierTraj = new CBezierTraj;
	
	CPnt& ptStart = GetStartPnt();
	CPnt& ptEnd = GetEndPnt();
	pBezierTraj->CreateTraj(ptStart, ptEnd, m_CtrlPnt[1], ptEnd, FORWARD, m_TurnDir, m_bTangency, m_angShiftHeading);
	return pBezierTraj;
#endif
	return NULL;
}

bool CGenericPath::Create(FILE *StreamIn)
{
	return true;
}

bool CGenericPath::Save(FILE *StreamOut)
{
	return true;
}

#ifdef _MFC_VER

//
//   Create the characteristic part of the Bezier-type path's data
//   from a binary file.
//
bool CGenericPath::Create(CArchive& ar)
{
	USHORT uDir;            // Positive input/negative input
	USHORT uTemp;
	CPnt CtrlPnt[2];
	float angShiftHeading;
	bool bTangency;

	if (!CPath::Create(ar))
		return false;

	ar >> m_uFwdRotoScannerObstacle
		>> m_uFwdObdetectorObstacle
		>> m_uBwdRotoScannerObstacle
		>> m_uBwdObdetectorObstacle
		>> uDir;

#if 1
	// ����ؼ�������
	ar >> m_nCountCtrlPoints;
#else
	m_nCountCtrlPoints = 2;
#endif

	if (m_pptCtrl != NULL)
		delete []m_pptCtrl;

	m_pptCtrl = new CPnt[m_nCountCtrlPoints];

	for (int i = 0; i < m_nCountCtrlPoints; i++)
	{
		CPnt& pt = m_pptCtrl[i];
		ar >> pt.x >> pt.y;
	}

	ar >> bTangency;

	CPnt ptStart = GetStartNode().GetPntObject();
	CPnt ptEnd = GetEndNode().GetPntObject();
	
	m_pstStart.GetPntObject() = ptStart;
	m_pstEnd.GetPntObject() = ptEnd;

	CLine ln1(ptStart, m_pptCtrl[0]);
	CLine ln2(m_pptCtrl[m_nCountCtrlPoints - 1], ptEnd);

	m_pstStart.fThita = ln1.SlantAngle().m_fRad;
	m_pstEnd.fThita = ln2.SlantAngle().m_fRad;

	return Init();
}

//
//   Save the characteristic part of the Bezier-type path's data to a binary file.
//
bool CGenericPath::Save(CArchive& ar)
{
	// Load the fields in the base class
	if (!CPath::Save(ar))
		return false;

	USHORT uHeading = POSITIVE_HEADING;
	bool bTangency = !m_bTangency;

	// Load the fields specific to this class
	ar << m_uFwdRotoScannerObstacle
		<< m_uFwdObdetectorObstacle
		<< m_uBwdRotoScannerObstacle
		<< m_uBwdObdetectorObstacle
		<< uHeading;

	// ֻ������Ƶ�(�ڴ˲��ٱ��������˵�)
	int nCountCtrlPoints = m_Curve.m_nCountKeyPoints - 2;
	ar << nCountCtrlPoints;

	// �������Ƶ�����
	for (int i = 0; i < nCountCtrlPoints; i++)
	{
		CPnt& pt = m_Curve.m_ptKey[i + 1];
		ar << pt.x << pt.y;
	}

	ar << bTangency;

	return true;
}

void CGenericPath::Draw(CScreenReference& ScrnRef, CDC* pDc, COLORREF cr, int nWidth)
{
	m_Curve.Draw(ScrnRef, pDc, cr, nWidth, nWidth, false);
}


//
//   �жϸ�������Ļ���Ƿ����������ϡ�
//   ����ֵ��
//     -1: δ����������
//      0: �������ߵ�һ��λ����
//    1~n: (nΪ�ؼ������)����ĳ���ؼ����ϣ����عؼ������(��1Ϊ��ʼ���)
//
int CGenericPath::PointHitTest(CPoint& pnt, CScreenReference& ScrnRef)
{
	SIZE sizeZero = { 0,0 };

	// �ж���Ļ���Ƿ�����ĳ���ؼ��㴦
	for (int i = 1; i < m_Curve.m_nCountKeyPoints - 1; i++)
	{
		CPoint pntKey = ScrnRef.GetWindowPoint(m_Curve.m_ptKey[i]);

		CRect r(pntKey, sizeZero);   // Construct an emtpy rectangle
		r.InflateRect(2, 2);

		if (r.PtInRect(pnt))
			return i + 1;
	}

	// �����ж���Ļ���Ƿ�����������
	CPnt ptClosest;
	CPnt pt = ScrnRef.GetWorldPoint(pnt);

	// ����˵�����������ϵ�ڵ����ߵľ���
	if (m_Curve.GetClosestPoint(pt, &ptClosest))
	{
		// ����㵽���ߵ��������
		float fDist = pt.DistanceTo(ptClosest);

		// ���㵽��Ļ���ھ���
		int nDist = (int)(fDist * ScrnRef.m_fRatio);

		// �����Ļ���ھ���С��3����Ϊ��괥����·��
		if (nDist <= 3)
			return 0;
	}

	return -1;
}

//
//   ������·�����ߵĿ��Ƶ㡣
//
void CGenericPath::DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr)
{
	m_Curve.DrawCtrlPoints(ScrnRef, pDC, cr, 3);
}

#endif
