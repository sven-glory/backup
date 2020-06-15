#pragma once

#include "Path.h"
#include "ArcPath.h"
#include "Bezier.h"

//////////////////////////////////////////////////////////////////////////////
//   ����"CGenericPath"��: �����"CPath"�̳ж�������������һ�������·����
//
class DllExport CGenericPath : public CPath
{
protected:
	CAngle   m_angStartHeading;     // ��������ʼ�ڵ㴦�ķ����
	CAngle   m_angEndHeading;       // ��������ֹ�ڵ㴦�ķ����
	int      m_nCountCtrlPoints;    // �����п��Ƶ������(���������˽ڵ�)
	CPnt* m_pptCtrl;            // ָ�������и������Ƶ��ָ��(���������˽ڵ�)
	

	CPosture m_pstStart;            // ��ʼ��̬
	CPosture m_pstEnd;              // ��ֹ��̬

	bool	 m_bTangency;		 //���з�ʽΪ���л���ƽ��, �˶���ʽ 0:���� 1:ƽ��

public:
	CBezier  m_Curve;               // ���߶���
											  // ��������ױ���������
private:
	// ��������ױ���������·��
	bool Create(CPosture& pstStart, CPosture& pstEnd, int nCountCtrlPoints, CPnt* pptCtrl);

	// �������ױ���������·��
	bool Create(CPosture& pstStart, CPosture& pstEnd, float fLen1, float fLen2);

protected:
	bool Init();

public:
	// ���캯��: �������ױ���������
	CGenericPath(int uId, int nStartNode, int nEndNode, CPosture& pstStart, 
		CPosture& pstEnd, float fLen1, float fLen2, float fVeloLimit = 0.6f, 
		SHORT nGuideType = TAPE_GUIDANCE, USHORT uObstacle = 0, 
		USHORT uDir = POSITIVE_HEADING, USHORT uExtType = 0);

	// ȱʡ���캯��
	CGenericPath();

	// ��������
	~CGenericPath();

	// ����Բ��·������Bezier����·��
	bool CreateFromArcPath(CArcPath& ArcPath);

	// Get the vehicle's required heading at the specified node
	virtual CAngle& GetHeading(CNode& nd);

	virtual bool IsTangency() {return m_bTangency;}

	// Make a trajectory from the path
	virtual CTraj* MakeTraj();

	// Fuzzy node checking limit (5 degree)
	virtual float FuzzyNodeCheckLimit() { return 0.087f; }

	virtual bool Create(FILE *StreamIn);
	virtual bool Save(FILE *StreamOut);

#ifdef _MFC_VER
	// �Ӷ������ļ����ɶ�������
	virtual bool Create(CArchive& ar);

	// ����������д�뵽�������ļ���
	virtual bool Save(CArchive& ar);

	virtual void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, int nWidth = 2);
	virtual int PointHitTest(CPoint& pnt, CScreenReference& ScrnRef);

	// ������·�����ߵĿ��Ƶ�
	void DrawCtrlPoints(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr);
#endif
};
