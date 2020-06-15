#ifndef __CMoveObj
#define __CMoveObj

#include "Geometry.h"
#include "ScrnRef.h"

#define CYCLE_TIME              0.05f
#define MAX_MOVING_OBJ_NUM      50

///////////////////////////////////////////////////////////////////////////////

//
//   ���塰Χ�����򡱡�
//
class CFence : public CRectangle
{
public:
	// ��ָ�������������ȡ��һ�㲢����
	CPnt GetRandomPoint();
};

///////////////////////////////////////////////////////////////////////////////
//
//   ���塰�ƶ����塱��
//
class CMovingObj
{
private:
	CFence   m_Fence;                    // Χ������
	CPnt m_ptStart;                  // ��ǰ�˶�·�ε���ʼ��
	CPnt m_ptTarget;                 // ��ǰ�˶�·�ε�Ŀ���
	CLine    m_ln;                       // ��ǰ�˶�ֱ�߶�
	int      m_nStep;                    // ��ǰ����
	float    m_fVel;                     // �˶��ٶ�
	bool     m_bShown;                   // �Ƿ�ͼ���ѻ��ƹ�

public:
	CCircle  m_Circle;                   // ��ǰͼ��
	CCircle  m_Circle0;                  // ��һ�λ��Ƶ�ͼ��

private:
	// �����˶���Ŀ���
	void SetTargetPoint();

public:
	CMovingObj();

	// ���ļ���װ�����
	bool Load(FILE* fp, float fAmpRatio = 1);

	// �����Զ��ƶ�����
	void Start();

	// ������һ��λ�õ�
	void Run();

	// ��������һ�γ�ʼ���ƹ���
	void NewDraw();

#ifdef _MSC_VER
	// ���Ƹ��ƶ�Ŀ��
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
#endif
};

///////////////////////////////////////////////////////////////////////////////
//
//   ���塰�ƶ����弯�ϡ���
//
class CMovingObjSet
{
public:
	CMovingObj m_Obj[MAX_MOVING_OBJ_NUM];
	int        m_nCount;

public:
	CMovingObjSet() {}

	// ���ļ���װ�����
	bool Load(FILE* fp, float fAmpRatio = 1);

	// �����Զ��ƶ�����
	void Start();

	// ������һ��λ�õ�
	void Run();

	// ��������һ�γ�ʼ���ƹ���
	void NewDraw();

#ifdef _MSC_VER
	// ���Ƹ��ƶ�Ŀ��
	void Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth);
#endif
};

#endif
