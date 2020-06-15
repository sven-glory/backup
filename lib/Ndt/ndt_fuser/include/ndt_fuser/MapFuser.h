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

	// ��ͼ��Χ���ֱ���
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
	CPointFeatureSet highIntensityPoints;    // �����㼯��
	CPointFeatureSet refPoints;              // ����弯��

	// �����������̬
	Eigen::Affine3d Tlast_fuse;

public:
	CMapFuser(double map_size_x = DEFAULT_MAP_SIZE_X, double map_size_y = DEFAULT_MAP_SIZE_Y, double map_reso = DEFAULT_MAP_RESO);

	~CMapFuser();

	// �������ݼ�
	void SetDataSet(CSlamDataSet* pDataset);

	// ���á�����λ��ģʽ�������õĶ�λ����
	void SetLocalizationMethod(int method);

	// �����µ�һ֡����
	bool processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Affine3d& odometry);

	// Ϊ��ͼ���ó�ʼ��̬����һ֡����
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud);

	// ���ݻ�����λ�˺ͽ��յ��ĵ��ƣ�������Ӧ�ĸ��´���
	Eigen::Affine3d update(Eigen::Affine3d& odometry, pcl::PointCloud<pcl::PointXYZ> &cloud, bool& matched);

	// ���ļ���װ���ͼ
	bool LoadMap(FILE* fp);

	// ����ͼ���浽�ļ�
	bool SaveMap(FILE* fp);

	// ������ͼ(�����ͼ)���浽�ļ�
	bool SaveFeatureMap(FILE* fp);

	////////////////////////////////////////////////////////////////////////////

	// �������ݼ����е�����ģ
	bool StepBuild();

	// ��̨֧���߳�
	static UINT SupportProc(LPVOID pParam);

	// ����֧�ź���
	bool SupportRoutine();

	// ����SLAM����
	bool Start(int nTargetStep);

	// ����SLAM����
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
