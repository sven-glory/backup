#include <stdafx.h>
#include "ndt_fuser/VectPose.h"
#include "AffinePosture.h"
#include "ScrnRef.h"

#ifdef QT_VERSION
#include <QPainter>
#include <QColor>
#endif

///////////////////////////////////////////////////////////////////////////////

CStatusPosture::CStatusPosture(const Eigen::Affine3d& affine)
{
	m_nStatus = 0;
	GetPostureObject() = AffineToPosture(affine);
}

void CStatusPosture::operator = (const Eigen::Affine3d& affine)
{
	m_nStatus = 0;
	GetPostureObject() = AffineToPosture(affine);
}

//
//   ���ļ�װ��λ��״̬��
//
bool CStatusPosture::Load(FILE* fp)
{
	float f[3];
	int n;

	if (fread(f, sizeof(float), 3, fp) != 3)
		return false;

	x = f[0];
	y = f[1];
	fThita = f[2];

	if (fread(&n, sizeof(int), 1, fp) != 1)
		return false;
	m_nStatus = n;

	return true;
}

//
//   ��λ��״̬д���ļ���
//
bool CStatusPosture::Save(FILE* fp)
{
	float f[3] = {x, y, fThita};

	if (fwrite(f, sizeof(float), 3, fp) != 3)
		return false;

	if (fwrite(&m_nStatus, sizeof(int), 1, fp) != 1)
		return false;

	return true;
}

///////////////////////////////////////////////////////////////////////////////

//
//   ȡ��ָ����ŵ���̬
//
CPosture CVectPose::GetPosture(int nIdx)
{
	return at(nIdx).GetPostureObject();
}

bool CVectPose::Load(FILE* fp)
{
	clear();

	for (size_t i = 0; i < size(); i++)
	{
		CStatusPosture pose;
		if (!pose.Load(fp))
			return false;
		push_back(pose);
	}

	return true;
}

bool CVectPose::Save(FILE* fp)
{
	for (size_t i = 0; i < size(); i++)
	{
		if (!at(i).Save(fp))
			return false;
	}

	return true;
}

//
//   �ж�ָ���ĵ��Ƿ�����ĳ��λ�ˡ�
//   ����ֵ��
//     -1 : û�д������κ�λ��
//    ��������������λ�˵ı��
//
int CVectPose::PointHit(const CPnt& pt, float fDistGate)
{
	CPnt ptPose;
	for (int i = 0; i < (int)size(); i++)
	{
		ptPose = at(i);
		if (ptPose.DistanceTo(pt) < fDistGate)
			return i;
	}

	return -1;
}

#ifdef _MFC_VER
//
//   ��������λ�����ߺ���̬��
//
void CVectPose::Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected, unsigned long clrUnmatched)
{
	// ��ʾλ������
	for (size_t i = 0; i < size(); i++)
	{
		// �������һ����֮�⣬��Ҫ�������һ����Ĺ켣������
		if (i + 1 != size())
		{
			CLine ln(at(i), at(i+1));
			ln.Draw(ScrnRef, pDc, clrTraj, 2);
		}
	}

	// ��ʾ����λ��
	for (size_t i = 0; i < size(); i++)
	{
		// �����Ҫ�Ļ�����ʾ����̬
		if (m_bShowPoses)
		{
			// �����Ҫ�Ļ�����ʾѡ����̬
			if (m_bShowSelected && i == m_nSelected)
				at(i).Draw(ScrnRef, pDc, clrSelected, 40, 150, 2);
			else if (at(i).m_nStatus == 1)
				at(i).Draw(ScrnRef, pDc, clrPoses, 40, 150, 1);
			else
				at(i).Draw(ScrnRef, pDc, clrUnmatched, 40, 150, 1);
		}
	}
}

#elif defined QT_VERSION
//
//   ��������λ�����ߺ���̬��
//
void CVectPose::Plot(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrTraj,
	QColor clrPoses, QColor clrSelected, QColor clrUnmatched)
{
	// ��ʾλ������
	for (size_t i = 0; i < size(); i++)
	{
		// �������һ����֮�⣬��Ҫ�������һ����Ĺ켣������
		if (i + 1 != size())
		{
			CLine ln(at(i), at(i + 1));
			ln.Draw(ScrnRef, pPainter, clrTraj, 2);
		}
	}

	// ��ʾ����λ��
	for (size_t i = 0; i < size(); i++)
	{
		// �����Ҫ�Ļ�����ʾ����̬
		if (m_bShowPoses)
		{
			// �����Ҫ�Ļ�����ʾѡ����̬
			if (m_bShowSelected && i == m_nSelected)
				at(i).Draw(ScrnRef, pPainter, clrSelected, 40, 150, 2);
			else if (at(i).m_nStatus == 1)
				at(i).Draw(ScrnRef, pPainter, clrPoses, 40, 150, 1);
			else
				at(i).Draw(ScrnRef, pPainter, clrUnmatched, 40, 150, 1);
		}
	}
}

#endif
