#include "stdafx.h"
#include <stdlib.h>
#include "MovingObj.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define CYCLE_TIME              0.05f

///////////////////////////////////////////////////////////////////////////////

//
//   ��ָ�������������ȡ��һ�㲢���ء�
//
CPnt CFence::GetRandomPoint()
{
	CPnt pt;
	
	float fRatioX = (rand() % 100) / 100.0f;
	float fRatioY = (rand() % 100) / 100.0f;

	pt.x = Left() + Width() * fRatioX;
	pt.y = Bottom() + Height() * fRatioY;

	return pt;
}

///////////////////////////////////////////////////////////////////////////////

CMovingObj::CMovingObj()
{
	m_fVel = 4.5f;
	m_Circle.m_fRadius = 0.2f;
	m_Fence.Create(4.0f, 4.0f, 8.0f, 10.0f);
	m_bShown = false;
}

//
//   ���ļ���װ�������
//
bool CMovingObj::Load(FILE* fp, float fAmpRatio)
{
	float fLeft, fTop, fRight, fBottom;
	fscanf(fp, "%f\t%f\t%f\t%f\t%f\t%f\n", &fLeft, &fTop, &fRight, &fBottom,
		&m_Circle.m_fRadius, &m_fVel);

	fLeft *= fAmpRatio;
	fTop *= fAmpRatio;
	fRight *= fAmpRatio;
	fBottom *= fAmpRatio;

	m_Fence.Create(fLeft, fTop, fRight, fBottom);

	return true;
}

void CMovingObj::Start() 
{
	m_ptStart = m_Fence.GetRandomPoint();
	m_ptTarget = m_ptStart;
	m_nStep = -1;
	m_Circle.m_ptCenter = m_ptStart;
	m_Circle0 = m_Circle;
}

void CMovingObj::SetTargetPoint()
{
	m_ptTarget = m_Fence.GetRandomPoint();
	m_ln.Create(m_ptStart, m_ptTarget);
	m_nStep = 0;
}

// ������һ��λ�õ�
void CMovingObj::Run()
{
	if (m_nStep < 0)
	{
		m_ptStart = m_ptTarget;
		SetTargetPoint();
	}

	float fProgress = (m_nStep++ * m_fVel) * CYCLE_TIME;
	if (fProgress > m_ln.Length())
	{
		m_nStep = -1;
		fProgress = m_ln.Length();
	}

	m_Circle.m_ptCenter = m_ln.TrajFun(fProgress);
}

//
//   ��������һ�γ�ʼ���ƹ��̡�
//
void CMovingObj::NewDraw()
{
	m_bShown = false;
}

#ifdef _MSC_VER
//
//   ����Ļ�ϻ��ƴ�Բ��
//
void CMovingObj::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
{
	pDC->SetROP2(R2_NOT);
	
	// ������״λ��ƣ�û�в���ԭ��ͼ����һ����
	if (!m_bShown)
		m_bShown = true;	
	
	// ���״λ��ƣ�Ӧ�Ȳ���ԭ����ͼ��
	else	
		m_Circle0.Draw(ScrnRef, pDC, crColor, nWidth);


	// ������λ�û�����ͼ��
	m_Circle.Draw(ScrnRef, pDC, crColor, nWidth);
	m_Circle0 = m_Circle;

	pDC->SetROP2(R2_COPYPEN);
}
#endif

///////////////////////////////////////////////////////////////////////////////


//
//   ���ļ���װ�������
//
bool CMovingObjSet::Load(FILE* fp, float fAmpRatio)
{
	if (fscanf(fp, "%d\n", &m_nCount) != 1)
		return false;

	for (int i = 0; i < m_nCount; i++)
		if (!m_Obj[i].Load(fp, fAmpRatio))
			return false;

	return true;
}

//
//   �����Զ��ƶ����̡�
//
void CMovingObjSet::Start()
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].Start();
}

//
//   ������һ��λ�õ㡣
//
void CMovingObjSet::Run()
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].Run();
}

//
//   ��������һ�γ�ʼ���ƹ��̡�
//
void CMovingObjSet::NewDraw()
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].NewDraw();
}

#ifdef _MSC_VER
//
//   ���Ƹ��ƶ�Ŀ�ꡣ
//
void CMovingObjSet::Draw(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, int nWidth)
{
	for (int i = 0; i < m_nCount; i++)
		m_Obj[i].Draw(ScrnRef, pDC, crColor, nWidth);
}
#endif
