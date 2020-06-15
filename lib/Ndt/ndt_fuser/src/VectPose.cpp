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
//   从文件装入位姿状态。
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
//   将位姿状态写入文件。
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
//   取得指定序号的姿态
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
//   判断指定的点是否触碰到某个位姿。
//   返回值：
//     -1 : 没有触碰到任何位姿
//    其它：触碰到的位姿的编号
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
//   绘制整个位姿曲线和姿态。
//
void CVectPose::Plot(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected, unsigned long clrUnmatched)
{
	// 显示位姿曲线
	for (size_t i = 0; i < size(); i++)
	{
		// 除了最后一个点之外，需要画出与后一个点的轨迹连接线
		if (i + 1 != size())
		{
			CLine ln(at(i), at(i+1));
			ln.Draw(ScrnRef, pDc, clrTraj, 2);
		}
	}

	// 显示各个位姿
	for (size_t i = 0; i < size(); i++)
	{
		// 如果需要的话，显示此姿态
		if (m_bShowPoses)
		{
			// 如果需要的话，显示选中姿态
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
//   绘制整个位姿曲线和姿态。
//
void CVectPose::Plot(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrTraj,
	QColor clrPoses, QColor clrSelected, QColor clrUnmatched)
{
	// 显示位姿曲线
	for (size_t i = 0; i < size(); i++)
	{
		// 除了最后一个点之外，需要画出与后一个点的轨迹连接线
		if (i + 1 != size())
		{
			CLine ln(at(i), at(i + 1));
			ln.Draw(ScrnRef, pPainter, clrTraj, 2);
		}
	}

	// 显示各个位姿
	for (size_t i = 0; i < size(); i++)
	{
		// 如果需要的话，显示此姿态
		if (m_bShowPoses)
		{
			// 如果需要的话，显示选中姿态
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
