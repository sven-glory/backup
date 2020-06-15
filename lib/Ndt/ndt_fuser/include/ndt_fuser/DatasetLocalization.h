#ifndef __CLocalizationDevel
#define __CLocalizationDevel

#include "ndt_fuser\RobotLocalization.h"
#include "SlamStepData.h"

///////////////////////////////////////////////////////////////////////////////
//   ����NDT�Ķ�λ����ʵ���ࡣ
class CDatasetLocalization : public CRobotLocalization
{
public:
	CVectPose odomPoses;                               // δ��У������ʷ��̬
	CVectPose correctedPoses;                          // У�������ʷ��̬
	CVectPose filteredPoses;                           // ��ƽ���˲������ʷ��̬

protected:
	// ����CNdtLocalization::Localize()�������Ա����������̬��¼
	virtual bool Localize(const CPclPointCloud &cloud_in, Eigen::Affine3d& odometry);

public:
	CDatasetLocalization(double map_reso = DEFAULT_MAP_RESO);

	// ����һ�����ݣ������ö�λ׼��
	virtual void collectStepData(const CSlamStepData& Step);

	// �����첽�Ķ�λ��������̬���
	virtual bool AsynLocalize(Eigen::Affine3d& pose, bool realTimeRunning = true);

	// ������չ��λ��������̬���
	virtual int LocalizeEx(Eigen::Affine3d& pose);

	// ȡ�ö�Ӧ��ĳһ���ĺϳɵ���
	CPclPointCloud GetStepPointCloud(const CSlamStepData& Step);

#ifdef _MFC_VER
	void PlotLocalization(CDC* pDc, CScreenReference& ScrnRef, bool bShowSource, bool bShowTarget, bool ShowStatusTexts);
#elif defined QT_VERSION
	void PlotLocalization(QPainter* pPainter, CScreenReference& ScrnRef, bool bShowSource, bool bShowTarget, bool ShowStatusTexts);
#endif
};
#endif
