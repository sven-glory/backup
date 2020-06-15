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

	// 设置机器人初始姿态
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
//   设置数据集。
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
// 为地图设置初始姿态及第一帧点云
//
void CMapFuser::initialize(Eigen::Affine3d initPos, CPclPointCloud &cloud)
{
	perception_oru::transformPointCloudInPlace(initPos, cloud);
	Tlast_fuse = Tnow = initPos;

	// 生成地图对象，然后对其初始化
	map = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	map->initialize(Tnow.translation()(0), Tnow.translation()(1), Tnow.translation()(2), map_size_x, map_size_y, map_size_z);

	// 加入第一帧点云
	map->addPointCloud(Tnow.translation(), cloud, 0.1, 100.0, 0.1);

	// 生成NDT单元
	map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);

	isInit = true;
}

//
//   处理新的一帧数据。
//   返回值：
//      如果是第一帧或是后续帧但成功进行了环境匹配，则返回true
//      如果不是第一帧且环境匹配失败,或者是移动量太小，则返回false
//
bool CMapFuser::processFrame(CPclPointCloud &cloud, Eigen::Affine3d& odometry)
{
	// 如果这是第一帧，需要特殊处理
	if (count_clouds++ == 0)
	{
		initialize(pose_, cloud);

		// 记录新姿态
		CStatusPosture ps(AffineToPosture(pose_));
		correctedPoses.push_back(ps);
		return true;
	}

	// 如果不是第一帧
	else
	{
#if 0
		// 如果里程计位姿变化太小，不处理
		if ((Tmotion.translation().norm() < 0.01 && Tmotion.rotation().eulerAngles(0, 1, 2)(2) < 0.01))
			return false;
#endif

		// 由于采用了数据集拼接技术，里程计变化较大一定会出现，所以将下面的代码暂时关闭
#if 0
		// 如果里程计位移变化过大，或转角变化过大，忽略该帧
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
//   根据机器人位姿变化和接收到的点云，进行相应的更新处理。
//
Eigen::Affine3d CMapFuser::update(Eigen::Affine3d& odometry, CPclPointCloud &cloud, bool& matched)
{
	matched = false;

	if (!isInit)
		return Tnow;

	// 根据姿态变化更新里程姿态 (we track this only for display purposes!)
	Eigen::Affine3d Tinit = odometry;

	// 测试将局部图与模型地图进行匹配，如果成功，匹配结果姿态保存于Tinit中
	if (Localize(cloud, Tinit)){
	}

	{
		// 采信刚刚得到的配准姿态
		matched = true;
		Tnow = Tinit;
		double x = Tnow.translation().transpose()(0);
		double y = Tnow.translation().transpose()(1);
		double theta = Tnow.rotation().eulerAngles(0, 1, 2)(2);

		perception_oru::transformPointCloudInPlace(Tnow, cloud);

		// 得到传感器的姿态
		Eigen::Affine3d spose = Tnow;

		// 计算姿态的变化量
		Eigen::Affine3d diff_fuse = Tlast_fuse.inverse() * Tnow;

		if (diff_fuse.translation().norm() > translation_fuse_delta ||
			diff_fuse.rotation().eulerAngles(0, 1, 2).norm() > rotation_fuse_delta)
		{
			// 把局部点云按其均值添加到图中
			map->addPointCloudMeanUpdate(spose.translation(), cloud, localMapSize, 1e5, 5, 2 * map_size_z, 0.06);

			/////////////FIXME: check if this works for error beams //////////////////
			// 收集超出距离的激光点
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
//   从文件中装入地图。
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
//   将地图保存到文件。
//
bool CMapFuser::SaveMap(FILE* fp)
{
	if (!isInit || map == NULL)
		return false;

	return (map->Save(fp) == 0);
}

//
//   将特征图(反光板图)保存到文件。
//
bool CMapFuser::SaveFeatureMap(FILE* fp)
{
	if (!isInit || map == NULL)
		return false;

	// 保存反光板数据
	if (!refPoints.SaveBinary(fp))
		return false;

	// 直线数量为零，也需要输出
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
//   根据数据集进行单步建模
//
bool CMapFuser::StepBuild()
{
	if (m_nCurBuildStep < 0)
		m_nCurBuildStep = 0;

	// 如果当前还没有到最后一步，则后移一步
	if (m_nCurBuildStep < (int)m_pDataset->size() - 1)
	{
		// 取得当前步
		const CSlamStepData& Step = m_pDataset->at(m_nCurBuildStep);
		
		// 收集当前步的数据
		collectStepData(Step);

		// 计算里程姿态的变化量
		Eigen::Affine3d Tm = getOdomMove(odomAdjusted);

		// 根据校正后的里程姿态和里程变化量，估算出新的里程姿态
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

// 运行支撑函数
bool CMapFuser::SupportRoutine()
{
	// 自动递增当前步
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
//   通讯管理程序的后台线程。
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
//   启动SLAM过程。
//
bool CMapFuser::Start(int nTargetStep)
{
	m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hKillThread != NULL);

	m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
	ASSERT(m_hThreadDead != NULL);

	m_nSlamTargetStep = nTargetStep;
	m_bRunning = true;

	// 启动后台线程
	AfxBeginThread(SupportProc, this, THREAD_PRIORITY_ABOVE_NORMAL);

	return true;
}

//
//   结束SLAM过程。
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
//   绘制模型图。
//
void CMapFuser::PlotModelMap(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder,
	bool bShowMatched, unsigned long clrMatched)
{
	map->Plot(pDC, ScrnRef, clrCellFill, clrCellBorder, bShowMatched, clrMatched);
}

//
//   绘制位姿图。
//
void CMapFuser::PlotPoses(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrTraj,
	unsigned long clrPoses, unsigned long clrSelected, unsigned clrUnmatched)
{
	correctedPoses.Plot(pDC, ScrnRef, clrTraj, clrPoses, clrSelected, clrUnmatched);
}

#elif defined QT_VERSION

//
//   绘制模型图。
//
void CMapFuser::PlotModelMap(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrCellFill, QColor clrCellBorder, 
	bool bShowMatched, QColor clrMatched)
{
	if (map != NULL)
		map->Plot(pPainter, ScrnRef, clrCellFill, clrCellBorder, bShowMatched, clrMatched);
}

//
//   绘制位姿图。
//
void CMapFuser::PlotPoses(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrTraj,
	QColor clrPoses, QColor clrSelected, QColor clrUnmatched)
{
	correctedPoses.Plot(pPainter, ScrnRef, clrTraj, clrPoses, clrSelected, clrUnmatched);
}

#endif
