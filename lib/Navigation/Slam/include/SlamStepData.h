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
//   ����ÿ��SLAM�������ݼ���ش����ࡣ
class CSlamStepData
{
public:
	CStampedPosture m_pstMoveEst;    // �����˵Ĺ�����̬�仯��
	CStampedPosture m_pst;           // �����˵ľ�����̬
	CPosture        m_vel;           // �ٶ�����������CPosture�ṹ����ʾ
	CVectScan m_scanLocal;           // �����ɨ����̬�ľֲ�����
	CVectScan m_scanGlobal;          // ȫ�ּ���ɨ������
	CVectFeatureSet m_featureLocal;  // ��������(����ڼ�������ǰ��̬)
	CVectFeatureSet m_featureGlobal; // ��������(�����ȫ������ϵ)

public:
	CSlamStepData() { Clear(); }

	// �������캯��
	CSlamStepData(const CSlamStepData& Obj);
	
	// ��ɨ�������������
	bool CreateFromScan(const CScan& Scan);

	// �������ļ���ȡԭʼɨ�������
	bool LoadRawScanBinary(FILE* fp, const CScannerGroupParam& ScannerParam, int nFileVersion, bool bFirstStep = false);

	// ��ԭʼɨ�������д��������ļ�
	bool SaveRawScanBinary(FILE* fp, int nFileVersion, bool bSaveGlobalPosture = false);

	// �����������
	void Clear();

	// ���ݸ�����ԭʼ����ϵ��������ת������������ϵ��
	void CreateGlobalData(CPosture& pstInit, const CScannerGroupParam& ScannerGroupParam);
	
	// ȡ��ָ��������ɢ��
	CScanPoint* GetWorldRawPoint(int nScannerId, int nIdxPoint);

	// �����������ݽ���ָ��������任
	void Transform(const CFrame& frame);

	// �����������ݽ���ָ����������任
	void InvTransform(const CFrame& frame);

	// ���ù���ɨ��Ƕȷ�Χ��Լ��
	void ApplyScanAngleRule(int nScannerId, float fMinAngle, float fMaxAngle);

	// ���ù���ɨ������Լ��
	void ApplyScanDistRule(int nScannerId, float fMinDist, float fMaxDist);

	// �ж�ָ������Ļ���Ƿ񴥱��������е�ĳ��ԭʼ�㡣
	int PointHitRawPoint(const CPnt& pt, float fDistGate);

	// �ж�ָ������Ļ���Ƿ񴥱��������еĻ�����λ��
	int PointHitPose(const CPnt& pt, float fDistGate);

	// �Ա����е�ָ��������ɨ�����ݽ���ɨ��ƥ��
	bool MatchScans(int nScanId1, int nScanId2, CPosture& result);

#ifdef _MFC_VER
	// ����Ļ����ʾ
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF clrRawPoint, COLORREF clrHightLightRawPoint,
		bool bShowScanner = false, bool bShowFeature = false, COLORREF clrFeature = 0);
	
#elif defined QT_VERSION
	// ����Ļ����ʾ
	virtual void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor clrRawPoint, QColor clrHightLightRawPoint,
		bool bShowScanner = false, bool bShowFeature = false, QColor clrFeature = Qt::black);
#endif

};
#endif
