#include <stdafx.h>
#include "ndt_fuser/DatasetLocalization.h"
#include "AffinePosture.h"
#include "PclPointCloud.h"

///////////////////////////////////////////////////////////////////////////////

CDatasetLocalization::CDatasetLocalization(double map_reso) : CRobotLocalization(map_reso)
{
}

//
//   ����һ�����ݣ������ö�λ׼����
//
void CDatasetLocalization::collectStepData(const CSlamStepData& Step)
{
	unsigned int uLatestTime = Step.m_pstMoveEst.m_uTime;

	// ���ʱ���(ʱ�����m_pstMoveEstЯ��)
	CStampedAffine stampedOdomMove(PostureToAffine(Step.m_pstMoveEst), Step.m_pstMoveEst.m_uTime);

	// �ȵ�����̻ص�����
	OnReceiveOdometry(stampedOdomMove);

	// �����м������ĵ��ƽ��е���
	for (int i = 0; i < m_pScannerGroupParam->size(); i++)
	{
		if (m_pScannerGroupParam->at(i).m_bUseLocalize)
		{
			const CScan& scan = Step.m_scanLocal[i];
			CStampedPclCloud cloud(Scan2PclCloud(scan), scan.m_uTime);
			OnReceiveLaserScan(i, cloud);

			if (scan.m_uTime > uLatestTime)
				uLatestTime = scan.m_uTime;
		}
	}
	vel = PostureToAffine(Step.m_vel);

	// �������������ʱ�����Ϊ��λ��ʼ��ʱ�䣬������̬�岹�������ռ�
	CRobotLocalization::CollectOdomLaserData(uLatestTime);
}

//
//   ����CNdtLocalization::Localize()�������Ա����������̬��¼��
//
bool CDatasetLocalization::Localize(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry)
{
	// �ȵ���ԭ����
	bool bOk = CNdtLocalization::Localize(cloud_in, odometry);

	// ������̬��¼
	CStatusPosture ps = odometry;
	if (bOk)
		ps.m_nStatus = 1;

	// ��¼����̬
	correctedPoses.push_back(ps);
	return bOk;
}

//
//   �����첽�Ķ�λ��������̬�����
//
bool CDatasetLocalization::AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning)
{
	bool ret = CRobotLocalization::AsynLocalize(pose, realTimeRunning);
	CStatusPosture ps = pose;
	if (ret)
		ps.m_nStatus = 1;

	odomPoses.push_back(estTnow);
	filteredPoses.push_back(ps);

	return ret;
}

//
//   ������չ��λ��������̬�����
//
int CDatasetLocalization::LocalizeEx(Eigen::Affine3d& pose)
{
	pose = odomAdjusted;
	return CNdtLocalization::LocalizeEx(cloudAdjusted, pose);
}

//
//   ȡ�ö�Ӧ��ĳһ���ĺϳɵ��ơ�
//
CPclPointCloud CDatasetLocalization::GetStepPointCloud(const CSlamStepData& Step)
{
	collectStepData(Step);
	return cloudAdjusted;
}

#ifdef _MFC_VER

#define _RGB(r,g,b)          ((COLORREF)(((BYTE)(r)|((WORD)((BYTE)(g))<<8))|(((DWORD)(BYTE)(b))<<16)))

void CDatasetLocalization::PlotLocalization(CDC* pDC, CScreenReference& ScrnRef, bool bShowSource, bool bShowTarget, bool ShowStatusTexts)
{
#if 0
	if (nCurStep >= 0)
		Localization.matcher2D.ShowStatus(pDC, ScrnRef, nCurStep + 1, TestDataSet.size(), _RGB(0, 0, 0));
#endif

	// ��ʾ��ͼ��������λ������
	if (map != NULL)
	{
		map->Plot(pDC, ScrnRef, _RGB(0, 128, 192), _RGB(0, 0, 0), bShowTarget, _RGB(225, 110, 0));

		if (bShowSource)
			matcher2D.Plot(pDC, ScrnRef, _RGB(0, 180, 0), _RGB(0, 215, 0), _RGB(0, 128, 128), _RGB(0, 255, 255));

		// ��ʾλ������
		correctedPoses.SetOption(true, true);
		correctedPoses.Plot(pDC, ScrnRef, _RGB(0, 128, 0), _RGB(255, 0, 255), _RGB(0, 128, 64), _RGB(128, 128, 128));

#if 0
		odomPoses.SetOption(true, true);
		odomPoses.Plot(pDC, ScrnRef, _RGB(0, 128, 0), _RGB(0, 0, 255), _RGB(0, 128, 64), _RGB(128, 128, 128));

		for (int i = 0; i < odomPoses.size(); i++)
		{
			CPnt pt1 = odomPoses[i].GetPntObject();
			CPnt pt2 = correctedPoses[i].GetPntObject();
			CLine ln(pt1, pt2);
			ln.Draw(ScrnRef, pDC, _RGB(0, 0, 128));
		}
#endif
	}
}

#elif defined QT_VERSION
void CDatasetLocalization::PlotLocalization(QPainter* pPainter, CScreenReference& ScrnRef, bool bShowSource, bool bShowTarget, bool ShowStatusTexts)
{
}
#endif
