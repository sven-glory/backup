#include <stdafx.h>

#undef max
#undef min

#include "ndt_fuser/MapFuser.h"
#include "AffinePosture.h"
#include "time_patch.h"
#include "SlamDataSet.h"

#ifdef QT_VERSION
#include <QPainter>
#include <QColor>
#endif

#define MAX_TRANSLATION_DELTA            0.5
#define MAX_ROTATION_DELTA               0.5

//////////////////////////////////////////////////////////////////////////////

CMapFuser::CMapFuser(double map_size_x_, double map_size_y_, double map_reso) :
	CDatasetLocalization(map_reso)
{
	map_size_x = map_size_x_;
	map_size_y = map_size_y_;

	localMapSize << sensor_max_range, sensor_max_range, map_size_z;

	// ���û����˳�ʼ��̬
	pose_ = Eigen::Translation<double, 3>(0, 0, 0)*
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitZ());

	translation_fuse_delta = 0.0;
	rotation_fuse_delta = 0.0;
	isInit = false;

	m_nCurBuildStep = -1;
	count_clouds = 0;
	m_pDataset = NULL;
}

CMapFuser::~CMapFuser()
{
}

//
//   �������ݼ���
//
void CMapFuser::SetDataSet(CSlamDataSet* pDataset)
{
	m_nCurBuildStep = 0;
	m_pDataset = pDataset;
	SetScannerGroupParam(&(m_pDataset->m_ScannerParam));
	Eigen::Affine3d poseZero;
	poseZero.setIdentity();
	SetPose(poseZero);
}

//
// Ϊ��ͼ���ó�ʼ��̬����һ֡����
//
void CMapFuser::initialize(Eigen::Affine3d initPos, CPclPointCloud &cloud)
{
	perception_oru::transformPointCloudInPlace(initPos, cloud);
	Tlast_fuse = Tnow = initPos;

	// ���ɵ�ͼ����Ȼ������ʼ��
	map = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	map->initialize(Tnow.translation()(0), Tnow.translation()(1), Tnow.translation()(2), map_size_x, map_size_y, map_size_z);

	// �����һ֡����
	map->addPointCloud(Tnow.translation(), cloud, 0.1, 100.0, 0.1);

	// ����NDT��Ԫ
	map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);

	isInit = true;
}

//
//   �����µ�һ֡���ݡ�
//   ����ֵ��
//      ����ǵ�һ֡���Ǻ���֡���ɹ������˻���ƥ�䣬�򷵻�true
//      ������ǵ�һ֡�һ���ƥ��ʧ��,�������ƶ���̫С���򷵻�false
//
bool CMapFuser::processFrame(CPclPointCloud &cloud, Eigen::Affine3d& odometry)
{
	// ������ǵ�һ֡����Ҫ���⴦��
	if (count_clouds++ == 0)
	{
		initialize(pose_, cloud);

		// ��¼����̬
		CStatusPosture ps(AffineToPosture(pose_));
		correctedPoses.push_back(ps);
		return true;
	}

	// ������ǵ�һ֡
	else
	{
#if 0
		// �����̼�λ�˱仯̫С��������
		if ((Tmotion.translation().norm() < 0.01 && Tmotion.rotation().eulerAngles(0, 1, 2)(2) < 0.01))
			return false;
#endif

		// ���ڲ��������ݼ�ƴ�Ӽ�������̼Ʊ仯�ϴ�һ������֣����Խ�����Ĵ�����ʱ�ر�
#if 0
		// �����̼�λ�Ʊ仯���󣬻�ת�Ǳ仯���󣬺��Ը�֡
		if (Tmotion.translation().norm() > MAX_TRANSLATION_DELTA ||
			Tmotion.rotation().eulerAngles(0, 1, 2)(2) > MAX_ROTATION_DELTA)
			Tmotion.setIdentity();
#endif
		bool matched = false;
		pose_ = update(odometry, cloud, matched);

		return matched;
	}
}

//
//   ���ݻ�����λ�˱仯�ͽ��յ��ĵ��ƣ�������Ӧ�ĸ��´���
//
Eigen::Affine3d CMapFuser::update(Eigen::Affine3d& odometry, CPclPointCloud &cloud, bool& matched)
{
	matched = false;

	if (!isInit)
		return Tnow;

	// ������̬�仯���������̬ (we track this only for display purposes!)
	Eigen::Affine3d Tinit = odometry;

	// ���Խ��ֲ�ͼ��ģ�͵�ͼ����ƥ�䣬����ɹ���ƥ������̬������Tinit��
	if (Localize(cloud, Tinit)){
	}

	{
		// ���Ÿոյõ�����׼��̬
		matched = true;
		Tnow = Tinit;
		double x = Tnow.translation().transpose()(0);
		double y = Tnow.translation().transpose()(1);
		double theta = Tnow.rotation().eulerAngles(0, 1, 2)(2);

		perception_oru::transformPointCloudInPlace(Tnow, cloud);

		// �õ�����������̬
		Eigen::Affine3d spose = Tnow;

		// ������̬�ı仯��
		Eigen::Affine3d diff_fuse = Tlast_fuse.inverse() * Tnow;

		if (diff_fuse.translation().norm() > translation_fuse_delta ||
			diff_fuse.rotation().eulerAngles(0, 1, 2).norm() > rotation_fuse_delta)
		{
			// �Ѿֲ����ư����ֵ��ӵ�ͼ��
			map->addPointCloudMeanUpdate(spose.translation(), cloud, localMapSize, 1e5, 5, 2 * map_size_z, 0.06);

			/////////////FIXME: check if this works for error beams //////////////////
			// �ռ���������ļ����
			CPclPointCloud max_beams;
			for (int i = 0; i < cloud.points.size(); i++)
			{
				pcl::PointXYZ pt = cloud.points[i];
				double d = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
				if (d >= sensor_max_range)
					max_beams.points.push_back(pt);
			}

			map->addPointCloud(spose.translation(), max_beams, 0.06, 100, 0.25);
			Tlast_fuse = Tnow;
		}
	}

	return Tnow;
}

//
//   ���ļ���װ���ͼ��
//
bool CMapFuser::LoadMap(FILE* fp)
{
	if (CNdtLocalization::LoadMap(fp))
	{
		isInit = true;
		return true;
	}
	else
		return false;
}
//
//   ����ͼ���浽�ļ���
//
bool CMapFuser::SaveMap(FILE* fp)
{
	if (!isInit || map == NULL)
		return false;

	return (map->Save(fp) == 0);
}

//
//   ������ͼ(�����ͼ)���浽�ļ���
//
bool CMapFuser::SaveFeatureMap(FILE* fp)
{
	if (!isInit || map == NULL)
		return false;

	// ���淴�������
	if (!refPoints.SaveBinary(fp))
		return false;

	// ֱ������Ϊ�㣬Ҳ��Ҫ���
	int dummy = 0;
	if (fwrite(&dummy, sizeof(int), 1, fp) != 1)
		return false;

	return true;
}

///////////////////////////////////////////////////////////////////////////////

#include "PclPointCloud.h"
#include "PointFeatureSet.h"
#include "SlamDataSet.h"

HANDLE CMapFuser::m_hKillThread = NULL;
HANDLE CMapFuser::m_hThreadDead = NULL;

extern int  nCurSlamStep;
extern bool bRefresh;

//
//   �������ݼ����е�����ģ
//
bool CMapFuser::StepBuild()
{
	if (m_nCurBuildStep < 0)
		m_nCurBuildStep = 0;

	// �����ǰ��û�е����һ���������һ��
	if (m_nCurBuildStep < (int)m_pDataset->size() - 1)
	{
		// ȡ�õ�ǰ��
		const CSlamStepData& Step = m_pDataset->at(m_nCurBuildStep);
		
		// �ռ���ǰ��������
		collectStepData(Step);

		// ���������̬�ı仯��
		Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

		// ����У����������̬����̱仯����������µ������̬
		Eigen::Affine3d odometry = Tnow * Tm;

		bool ok = processFrame(cloudAdjusted, odometry);

		correctedPoses.Select(m_nCurBuildStep);

		if (ok)
		{
			CFeatureSet absFeatures = Step.m_featureLocal[0];
			absFeatures.InvTransform(correctedPoses[m_nCurBuildStep]);
			CPointFeatureSet* p = dynamic_cast<CPointFeatureSet*>(&absFeatures);
			highIntensityPoints += *p;
		}
		m_nCurBuildStep++;
		return true;
	}
	else
		return false;
}

// ����֧�ź���
bool CMapFuser::SupportRoutine()
{
	// �Զ�������ǰ��
	if (m_nCurBuildStep <= m_nSlamTargetStep)
	{
		m_crit.Lock();
		StepBuild();

		if ((m_nCurBuildStep % 5) == 0 && m_nCurBuildStep != m_nSlamTargetStep)
			bRefresh = true;
		
		m_crit.Unlock();
		return true;
	}
	else
	{
//		if (m_nCurBuildStep > 0)
//			m_nCurBuildStep--;

		return false;
	}
}

//
//   ͨѶ�������ĺ�̨�̡߳�
//
UINT CMapFuser::SupportProc(LPVOID pParam)
{
	CMapFuser* pFuser = (CMapFuser*)pParam;

	while (WaitForSingleObject(m_hKillThread, 0) != WAIT_OBJECT_0)
	{
		if (!pFuser->SupportRoutine())
		{
			pFuser->m_bRunning = false;
			break;
		}
		Sleep(2);
	}

	SetEvent(m_hThreadDead);
	return 0;
}

//
//   ����SLAM���̡�
//
bool CMapFuser::Start(int nTargetStep)
{
	m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hKillThread != NULL);

	m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hThreadDead != NULL);

	m_nSlamTargetStep = nTargetStep;
	m_bRunning = true;

	// ������̨�߳�
	AfxBeginThread(SupportProc, this, THREAD_PRIORITY_ABOVE_NORMAL);

	return true;
}

//
//   ����SLAM���̡�
//
void CMapFuser::Stop()
{
	SetEvent(m_hKillThread);
	WaitForSingleObject(m_hThreadDead, 5000);

	CloseHandle(m_hKillThread);
	CloseHandle(m_hThreadDead);
}

#ifdef _MFC_VER

//
//   ����ģ��ͼ��
//
void CMapFuser::PlotModelMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder,
	bool bShowMatched, unsigned long clrMatched)
{
	map->Plot(pDC, ScrnRef, clrCellFill, clrCellBorder, bShowMatched, clrMatched);
}

//
//   ����λ��ͼ��
//
void CMapFuser::PlotPoses(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected, unsigned clrUnmatched)
{
	correctedPoses.Plot(pDC, ScrnRef, clrTraj, clrPoses, clrSelected, clrUnmatched);
}

#elif defined QT_VERSION

//
//   ����ģ��ͼ��
//
void CMapFuser::PlotModelMap(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrCellFill, QColor clrCellBorder, 
	bool bShowMatched, QColor clrMatched)
{
	if (map != NULL)
		map->Plot(pPainter, ScrnRef, clrCellFill, clrCellBorder, bShowMatched, clrMatched);
}

//
//   ����λ��ͼ��
//
void CMapFuser::PlotPoses(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrTraj,
	QColor clrPoses, QColor clrSelected, QColor clrUnmatched)
{
	correctedPoses.Plot(pPainter, ScrnRef, clrTraj, clrPoses, clrSelected, clrUnmatched);
}

#endif
