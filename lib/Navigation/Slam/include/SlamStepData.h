#ifndef __CSlamStepData
#define __CSlamStepData

#include <vector>
#include "Geometry.h"
#include "Scan.h"
#include "FeatureSet.h"
#include "Frame.h"
#include "StampedPosture.h"

#ifdef QT_VERSION
#include <QColor>
#endif

using namespace std;

//////////////////////////////////////////////////////////////////////////////



typedef vector<CScan> CVectScan;
typedef vector<CFeatureSet> CVectFeatureSet;


///////////////////////////////////////////////////////////////////////////////
//   关于每个SLAM步的数据及相关处理类。
class CSlamStepData
{
public:
	CStampedPosture m_pstMoveEst;    // 机器人的估测姿态变化量
	CStampedPosture m_pst;           // 机器人的绝对姿态
	CPosture        m_vel;           // 速度向量，借用CPosture结构来表示
	CVectScan m_scanLocal;           // 相对于扫描姿态的局部点云
	CVectScan m_scanGlobal;          // 全局激光扫描数据
	CVectFeatureSet m_featureLocal;  // 特征集合(相对于激光器当前姿态)
	CVectFeatureSet m_featureGlobal; // 特征集合(相对于全局坐标系)

public:
	CSlamStepData() { Clear(); }

	// 拷贝构造函数
	CSlamStepData(const CSlamStepData& Obj);
	
	// 从扫描点云数据生成
	bool CreateFromScan(const CScan& Scan);

	// 二进制文件读取原始扫描点数据
	bool LoadRawScanBinary(FILE* fp, const CScannerGroupParam& ScannerParam, int nFileVersion, bool bFirstStep = false);

	// 将原始扫描点数据写入二进制文件
	bool SaveRawScanBinary(FILE* fp, int nFileVersion, bool bSaveGlobalPosture = false);

	// 清除所有数据
	void Clear();

	// 根据给定的原始坐标系，将数据转换到世界坐标系下
	void CreateGlobalData(CPosture& pstInit, const CScannerGroupParam& ScannerGroupParam);
	
	// 取得指定的世界散点
	CScanPoint* GetWorldRawPoint(int nScannerId, int nIdxPoint);

	// 将本步的数据进行指定的坐标变换
	void Transform(const CFrame& frame);

	// 将本步的数据进行指定的坐标逆变换
	void InvTransform(const CFrame& frame);

	// 启用关于扫描角度范围的约束
	void ApplyScanAngleRule(int nScannerId, float fMinAngle, float fMaxAngle);

	// 启用关于扫描距离的约束
	void ApplyScanDistRule(int nScannerId, float fMinDist, float fMaxDist);

	// 判断指定的屏幕点是否触本步数据中的某个原始点。
	int PointHitRawPoint(const CPnt& pt, float fDistGate);

	// 判断指定的屏幕点是否触本步数据中的机器人位姿
	int PointHitPose(const CPnt& pt, float fDistGate);

	// 对本步中的指定传感器扫描数据进行扫描匹配
	bool MatchScans(int nScanId1, int nScanId2, CPosture& result);

#ifdef _MFC_VER
	// 在屏幕上显示
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF clrRawPoint, COLORREF clrHightLightRawPoint,
		bool bShowScanner = false, bool bShowFeature = false, COLORREF clrFeature = 0);
	
#elif defined QT_VERSION
	// 在屏幕上显示
	virtual void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor clrRawPoint, QColor clrHightLightRawPoint,
		bool bShowScanner = false, bool bShowFeature = false, QColor clrFeature = Qt::black);
#endif

};
#endif
