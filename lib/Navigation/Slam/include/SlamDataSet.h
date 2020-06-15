#ifndef __CSlamDataSet
#define __CSlamDataSet

#include "SlamStepData.h"

// 对扫描数据进行滤波处理的规则
class CScanFilterRule
{
public:
	int   m_nScannerId;      // 激光器编号
	int   m_nType;           // 0-无规则; 1-角度规则; 2-距离规则
	int   m_nStartId;        // 开始启用的ID
	int   m_nEndId;          // 结束启用的ID
	float m_fParam[2];       // 规则参数

public:
	CScanFilterRule()
	{
		m_nScannerId = 0;
		m_nType = 0;
	}

	// 角度规则
	CScanFilterRule(int nScannerId, int nType, int nStartId, int nEndId, float fParam1, float fParam2)
	{
		m_nScannerId = nScannerId;
		m_nType = nType;
		m_nStartId = nStartId;
		m_nEndId = nEndId;
		m_fParam[0] = fParam1;
		m_fParam[1] = fParam2;
	}
};

typedef vector<CScanFilterRule> CScanFilterRules;

///////////////////////////////////////////////////////////////////////////////

class CSlamDataSet : public vector<CSlamStepData>
{
private:
	CPosture m_pstCur;
	int      m_nFileVersion;              // 数据文件的版本号
	int      m_nScannerCount;             // 激光扫描器的数量
	unsigned int m_uStartTime;            // 数据集的起始时间
	
public:
	CScannerGroupParam m_ScannerParam;    // 扫描器参数

public:
	CSlamDataSet() 
	{
		m_nFileVersion = 1;
		m_nScannerCount = 1;
		m_uStartTime = 0;
	}

	// 清除数据集
	void Clear();

	// 判断数据集是否为空
	bool IsEmpty() { return size() == 0; }

	// 装入二进制原始扫描点数据文件
	int LoadRawScanBinary(FILE* fp);

	// 将原始扫描点数据写入二进制文件
	bool SaveRawScanBinary(FILE* fp, int nFileVersion, int nFrom = 0, int nTo = -1, bool bReverseOrder = false,
		bool bSaveGlobalPosture = false);

	// 启用新的激光器参数
	bool SetScannerParam(const CScannerGroupParam& Param);

	// 取得指定的世界散点
	CScanPoint* GetWorldRawPoint(int nStepId, int nScannerId, int nIdxPoint);
	// 将两个数据进行合并
	void operator += (const CSlamDataSet& other);

	// 对数据集应用滤波规则
	void ApplyFilterRules(const CScanFilterRules& Rules);

	// 判断在某一步时，指定的屏幕点是否触及某个原始点
	int PointHitRawPoint(int nStepId, const CPnt& pt, float fDistGate);

	// 判断在某一步时，指定的屏幕点是否触及某个点特征
	int PointHitPointFeature(int nStepId, const CPnt& pt, float fDistGate);

	bool MatchScans(int nStepId, int nScanId1, int nScanId2, CPosture& result);

	// 根据局部数据生成全局数据
	void CreateGlobalData();

#ifdef _MFC_VER

	// 显示某一步
	virtual void Plot(int nStepId, CScreenReference& ScrnRef, CDC* pDC, COLORREF clrRawPoint, 
		bool bShowScanner = false, bool bShowFeature = false, COLORREF clrFeature = 0);

	// 显示全图

#elif defined QT_VERSION
	// 显示某一步
	virtual void Plot(int nStepId, CScreenReference& ScrnRef, QPainter* pPainter, QColor clrRawPoint,
		bool bShowScanner = false, bool bShowFeature = false, QColor clrFeature = Qt::black);
#endif
};
#endif
