#ifndef __CPointFeatureSet
#define __CPointFeatureSet

#include <stdio.h>
#include <vector>
#include "PointFeature.h"
#include "FeatureCreationParam.h"

#ifdef QT_VERSION
#include <QColor>
#endif

using namespace std;

class CPointMatchList;
class CScan;

//////////////////////////////////////////////////////////////////////////////
//   ���塰���������ϡ��ࡣ
class CPointFeatureSet : public vector <CPointFeature*>
{
private:
	CRectangle m_rect;
	float** m_pDistCache;     // ���ڴ洢�����֮���������ݻ�����

private:
	// �������ṩ���������ͷ���ռ�
	CPointFeature* NewPointFeature(int nSubType);

	// ���±߽�ֵ
	void UpdateCoveringRect();

	// ����ڲ��ĸ���֮��ľ���ֵ
	void ClearDistanceCache();

public:
	CPointFeatureSet();
	~CPointFeatureSet();

	// �����������캯��
	CPointFeatureSet(const CPointFeatureSet& another);

	// ��ֵ
	CPointFeatureSet& operator = (const CPointFeatureSet& another);

	// �������
	virtual void Clear();

	// ȡ�ü����ڵ������
	int GetCount() { return (int)size(); }

	// �������е�ļ������������ǵĵϿ�������
	void UpdateCartisian();

	// �򼯺������һ���µ�
	CPointFeatureSet& operator += (CPointFeature* pNewFeature);

	// �򼯺������һ���µ�
	CPointFeatureSet& operator += (const CPointFeature& NewFeature);

	// ����һ�����ϲ��뱾������
	CPointFeatureSet& operator += (const CPointFeatureSet& another);

	// ɾ��ָ����ŵĵ�
	bool DeleteAt(int nIdx);

	// ����������������������ϵ�任���ֲ�����ϵ
	void Transform(CPosture& pstLocal);

	// �������������ɾֲ�����ϵ�任����������ϵ
	void InvTransform(CPosture& pstOrigin);

	// �Ը����ĵ�Ϊ���ģ���ָ���ķ�ΧΪ�뾶����ȡ��һ�������Ӽ�
	bool GetSubset(const CPnt& ptCenter, float fRange, CPointFeatureSet& Subset);

	// ����ָ�����λ��
	bool ModidfyPointPos(int nId, CPnt& pt);

	// ͨ����pstScanner���Ը����ĵ㼯��������ɨ��������������ɾֲ�ɨ��㼯
	int CreateFromSimuScan(const CPointFeatureSet& Model, const CPosture& pstScanner, 
		float fMaxScanDist, bool bAddNoise = false);

	// �Ӹ�����ɨ��������ɷ��������
	bool CreateFromScan(const CScan& Scan, CReflectorCreationParam* pParam);

	// ȡ�ø�������
	CRectangle GetCoveringRect() const { return m_rect; }

	// Ϊ����֮��ľ������洢�ռ�
	bool CreateDistanceCache();

	// ȡ��i, j����֮��ľ���
	float PointDistance(int i, int j);

	// �����еĵ����������Ǵ�С�����˳���������
	void SortByPolarAngle();

	// ���ص�����кϲ�
	void MergeOverlappedPoints(float fDistGate = 0.2f);

	// ���ļ���װ��������������
	virtual int LoadText(FILE* fp);

	// �������������ݱ��浽�ļ���
	virtual int SaveText(FILE* fp);

	// �Ӷ������ļ���װ��������������
	virtual int LoadBinary(FILE* fp);

	// �������������ݱ��浽�������ļ���
	virtual int SaveBinary(FILE* fp);

	// �Ӷ������ļ���װ���û��༭����
	bool LoadUserData(FILE* fp);

	// ���û��༭���ݱ��浽�������ļ���
	bool SaveUserData(FILE* fp);

	// ������һ��PointFeatureSet�������û�ʹ������
	bool CopyUserData(const CPointFeatureSet& anther);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

	// �ж�ָ����������Ƿ����������������е�ĳ����������
	int PointHit(const CPnt& pt, float fDistGate);

#ifdef _MFC_VER
	void Dump();

	// ����Ļ�ϻ��ƴ˵���������
	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF cr, COLORREF crSelected, int nLineWidth = 0, 
		bool bShowId = false);

#elif defined QT_VERSION
	// ����Ļ�ϻ��ƴ˵���������
	void Plot(CScreenReference& ScrnRef, QPainter* pPainter, QColor cr, QColor crSelected, , int nLineWidth = 0,
		bool bShowId = false);
#endif
};
#endif

