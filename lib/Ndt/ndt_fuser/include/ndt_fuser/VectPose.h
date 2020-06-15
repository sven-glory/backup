#pragma once

#include <stdio.h>
#include <vector>
#include <Eigen/Eigen>
#include "Geometry.h"

using namespace std;

#ifdef _MSC_VER
class CDC;
#elif defined QT_VERSION
class QPainter;
class QColor;
#endif

class CScreenReference;
class CPnt;
class CPosture;

///////////////////////////////////////////////////////////////////////////////

class CStatusPosture : public CPosture
{
public:
	int m_nStatus;        // 0 - Odometry; 1 - scan matched

public:
	CStatusPosture(const CPosture p, int nStatus = 0)
	{
		SetPosture(p);
		m_nStatus = nStatus;
	}

	CStatusPosture(int nStatus = 0)
	{
		m_nStatus = nStatus;
	}

	CStatusPosture(const Eigen::Affine3d& affine);

	void operator = (const Eigen::Affine3d& affine);

	// ���ļ�װ��λ��״̬
	bool Load(FILE* fp);

	// ��λ��״̬д���ļ�
	bool Save(FILE* fp);
};

///////////////////////////////////////////////////////////////////////////////
//   ��ʷ��̬��¼
class CVectPose : public vector<CStatusPosture>
{
private:
	bool m_bShowPoses;
	bool m_bShowSelected;
	int  m_nSelected;

public:
	CVectPose() 
	{
		m_bShowPoses = false;
		m_bShowSelected = true;
		m_nSelected = -1;
	}

	// ѡ��ָ������̬
	void Select(int nIdx) { m_nSelected = nIdx; }

	// ������ʾѡ��
	void SetOption(bool bPosesOn, bool bSelectedOn)
	{
		m_bShowPoses = bPosesOn;
		m_bShowSelected = bSelectedOn;
	}

	// ȡ��ָ����ŵ���̬
	CPosture GetPosture(int nIdx);

	bool Load(FILE* fp);
	bool Save(FILE* fp);

	// �ж�ָ���ĵ��Ƿ�����ĳ��λ��
	int PointHit(const CPnt& pt, float fDistGate);

#ifdef _MFC_VER
	//   ��������λ�����ߺ���̬
	void Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrTraj, 
		unsigned long clrPoses, unsigned long clrSelected, unsigned long clrUnmatched);

#elif defined QT_VERSION
	//   ��������λ�����ߺ���̬
	void Plot(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrTraj, 
		QColor clrPoses, QColor clrSelected, QColor clrUnmatched);

#endif
};
