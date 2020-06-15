#ifndef __CLocalizationDevel
#define __CLocalizationDevel

#include "ndt_fuser\RobotLocalization.h"
#include "SlamStepData.h"

///////////////////////////////////////////////////////////////////////////////
//   基于NDT的定位开发实验类。
class CDatasetLocalization : public CRobotLocalization
{
public:
	CVectPose odomPoses;                               // 未经校正的历史姿态
	CVectPose correctedPoses;                          // 校正后的历史姿态
	CVectPose filteredPoses;                           // 过平滑滤波后的历史姿态

protected:
	// 重载CNdtLocalization::Localize()函数，以便添加运行姿态记录
	virtual bool Localize(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

public:
	CDatasetLocalization(double map_reso = DEFAULT_MAP_RESO);

	// 接收一步数据，并做好定位准备
	virtual void collectStepData(const CSlamStepData& Step);

	// 进行异步的定位并返回姿态结果
	virtual bool AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning = true);

	// 进行扩展定位并返回姿态结果
	virtual int LocalizeEx(Eigen::Affine3d& pose);

	// 取得对应于某一步的合成点云
	CPclPointCloud GetStepPointCloud(const CSlamStepData& Step);

#ifdef _MFC_VER
	void PlotLocalization(CDC* pDc, CScreenReference& ScrnRef, bool bShowSource, bool bShowTarget, bool ShowStatusTexts);
#elif defined QT_VERSION
	void PlotLocalization(QPainter* pPainter, CScreenReference& ScrnRef, bool bShowSource, bool bShowTarget, bool ShowStatusTexts);
#endif
};
#endif
