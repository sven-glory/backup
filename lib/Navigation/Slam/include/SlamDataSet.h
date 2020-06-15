#ifndef __CSlamDataSet
#define __CSlamDataSet

#include "SlamStepData.h"

// ��ɨ�����ݽ����˲�����Ĺ���
class CScanFilterRule
{
public:
	int   m_nScannerId;      // ���������
	int   m_nType;           // 0-�޹���; 1-�Ƕȹ���; 2-�������
	int   m_nStartId;        // ��ʼ���õ�ID
	int   m_nEndId;          // �������õ�ID
	float m_fParam[2];       // �������

public:
	CScanFilterRule()
	{
		m_nScannerId = 0;
		m_nType = 0;
	}

	// �Ƕȹ���
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
	int      m_nFileVersion;              // �����ļ��İ汾��
	int      m_nScannerCount;             // ����ɨ����������
	unsigned int m_uStartTime;            // ���ݼ�����ʼʱ��
	
public:
	CScannerGroupParam m_ScannerParam;    // ɨ��������

public:
	CSlamDataSet() 
	{
		m_nFileVersion = 1;
		m_nScannerCount = 1;
		m_uStartTime = 0;
	}

	// ������ݼ�
	void Clear();

	// �ж����ݼ��Ƿ�Ϊ��
	bool IsEmpty() { return size() == 0; }

	// װ�������ԭʼɨ��������ļ�
	int LoadRawScanBinary(FILE* fp);

	// ��ԭʼɨ�������д��������ļ�
	bool SaveRawScanBinary(FILE* fp, int nFileVersion, int nFrom = 0, int nTo = -1, bool bReverseOrder = false,
		bool bSaveGlobalPosture = false);

	// �����µļ���������
	bool SetScannerParam(const CScannerGroupParam& Param);

	// ȡ��ָ��������ɢ��
	CScanPoint* GetWorldRawPoint(int nStepId, int nScannerId, int nIdxPoint);
	// ���������ݽ��кϲ�
	void operator += (const CSlamDataSet& other);

	// �����ݼ�Ӧ���˲�����
	void ApplyFilterRules(const CScanFilterRules& Rules);

	// �ж���ĳһ��ʱ��ָ������Ļ���Ƿ񴥼�ĳ��ԭʼ��
	int PointHitRawPoint(int nStepId, const CPnt& pt, float fDistGate);

	// �ж���ĳһ��ʱ��ָ������Ļ���Ƿ񴥼�ĳ��������
	int PointHitPointFeature(int nStepId, const CPnt& pt, float fDistGate);

	bool MatchScans(int nStepId, int nScanId1, int nScanId2, CPosture& result);

	// ���ݾֲ���������ȫ������
	void CreateGlobalData();

#ifdef _MFC_VER

	// ��ʾĳһ��
	virtual void Plot(int nStepId, CScreenReference& ScrnRef, CDC* pDC, COLORREF clrRawPoint, 
		bool bShowScanner = false, bool bShowFeature = false, COLORREF clrFeature = 0);

	// ��ʾȫͼ

#elif defined QT_VERSION
	// ��ʾĳһ��
	virtual void Plot(int nStepId, CScreenReference& ScrnRef, QPainter* pPainter, QColor clrRawPoint,
		bool bShowScanner = false, bool bShowFeature = false, QColor clrFeature = Qt::black);
#endif
};
#endif
