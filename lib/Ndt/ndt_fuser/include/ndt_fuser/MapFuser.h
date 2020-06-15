#pragma once

#include "DatasetLocalization.h"
#include "PointFeatureSet.h"

#ifdef _MFC_VER
#include <afxmt.h>
class CDC;
class CScreenReference;
#elif defined QT_VERSION
#include <QColor>
#endif

class CSlamDataSet;

using namespace std;

class CMapFuser : public CDatasetLocalization
{
private:
	Eigen::Vector3d localMapSize;

	// 地图范围及分辨率
	double map_size_x;
	double map_size_y;

	Eigen::Affine3d pose_;

	double translation_fuse_delta;
	double rotation_fuse_delta;
	bool isInit;

	int    m_nSlamTargetStep;

public:
	bool   m_bRunning;

#ifdef _MFC_VER
	static HANDLE            m_hKillThread;
	static HANDLE            m_hThreadDead;

	CCriticalSection         m_crit;
#endif

public:
	CSlamDataSet* m_pDataset;
	int m_nCurBuildStep;
	CPointFeatureSet highIntensityPoints;    // 高亮点集合
	CPointFeatureSet refPoints;              // 反光板集合

	// 机器人里程姿态
	Eigen::Affine3d Tlast_fuse;

public:
	CMapFuser(double map_size_x = DEFAULT_MAP_SIZE_X, double map_size_y = DEFAULT_MAP_SIZE_Y, double map_reso = DEFAULT_MAP_RESO);

	~CMapFuser();

	// 设置数据集
	void SetDataSet(CSlamDataSet* pDataset);

	// 设置“仅定位”模式下所采用的定位方法
	void SetLocalizationMethod(int method);

	// 处理新的一帧数据
	bool processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d& odometry);

	// 为地图设置初始姿态及第一帧点云
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud);

	// 根据机器人位姿和接收到的点云，进行相应的更新处理
	Eigen::Affine3d update(Eigen::Affine3d& odometry, pcl::PointCloud<pcl::PointXYZ> &cloud, bool& matched);

	// 从文件中装入地图
	bool LoadMap(FILE* fp);

	// 将地图保存到文件
	bool SaveMap(FILE* fp);

	// 将特征图(反光板图)保存到文件
	bool SaveFeatureMap(FILE* fp);

	////////////////////////////////////////////////////////////////////////////

	// 根据数据集进行单步建模
	bool StepBuild();

	// 后台支持线程
	static UINT SupportProc(LPVOID pParam);

	// 运行支撑函数
	bool SupportRoutine();

	// 启动SLAM过程
	bool Start(int nTargetStep);

	// 结束SLAM过程
	void Stop();

#ifdef _MSC_VER
	void PlotModelMap(CDC* pDc, CScreenReference& ScrnRef, unsigned long clrCellFill, unsigned long clrCellBorder,
		bool bShowMatched = false, unsigned long clrMatched = 0);
	void PlotPoses(CDC* pDC, CScreenReference& ScrnRef, unsigned long clrTraj,
		unsigned long clrPoses, unsigned long clrSelected, unsigned clrUnmatched);

#elif defined QT_VERSION
	void PlotModelMap(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrCellFill, QColor clrCellBorder,
		bool bShowMatched = false, QColor clrMatched = Qt::black);
	void PlotPoses(QPainter* pPainter, CScreenReference& ScrnRef, QColor clrTraj,
		QColor clrPoses, QColor clrSelected, QColor clrUnmatched);

#endif

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
