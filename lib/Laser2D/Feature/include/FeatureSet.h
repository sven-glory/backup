#ifndef __CFeatureSet
#define __CFeatureSet

#include "PointFeatureSet.h"
#include "FeatureCreationParam.h"
#include "Scan.h"

///////////////////////////////////////////////////////////////////////////////
//    �����ƶ������˵ġ��������ϡ���
class CFeatureSet : public CPointFeatureSet
{
public:
	CPosture         m_pstObserver;         // �۲������ڵ���̬

public:
	CFeatureSet() : m_pstObserver(0, 0, 0) {}

	CPointFeatureSet& GetPointFeatureSet();

	// ���ù۲�������̬
	virtual void SetObserverPosture(const CPosture& pst) { m_pstObserver = pst; }

	// �������
	void Clear();

	// ���ı��ļ���װ����������
	int LoadText(FILE* fp);

	// ���������ϴ����ı��ļ���
	int SaveText(FILE* fp);

	// �Ӷ������ļ���װ����������
	int LoadBinary(FILE* fp);

	// ���������ϴ���������ļ���
	int SaveBinary(FILE* fp);

	// ��ԭʼɨ�����������������
	bool CreateFromRawScan(const CScan& Scan, CFeatureCreationParam& Param);

	// ͨ���Ը�������ģ�͵Ĳ���������������������
	bool CreateFromFeatureMap(const CFeatureSet& FeatureMap, const CPosture& pstScanner);

	// �Ը����ĵ�Ϊ���ģ���ָ���ķ�ΧΪ�뾶����ȡ��һ�������Ӽ�
	bool GetSubset(CPnt& ptCenter, float fRange, CFeatureSet& Subset);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	// ����ȫ��ͼ
	void Plot(CScreenReference& ScrnRef, CDC* pDc, COLORREF clrPoint, bool bShowId = false);
#endif
};
#endif
