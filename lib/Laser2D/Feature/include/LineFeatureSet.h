#ifndef __CLineFeatureSet
#define __CLineFeatureSet

#include "ScanPoint.h"
#include "LineFeature.h"
#include "PointFeatureSet.h"
#include "LineFeatureCreateParam.h"

#include <vector>

using namespace std;

class CScan;

//
//   CLineFeatureSet����ɨ���е�ֱ�߶Ρ�
//
class CLineFeatureSet : public vector<CLineFeature*>
{
private:
	CRectangle m_rect;

private:
	void UpdateCoveringRect();

	void SplitPoints(CScanPoint *sp, long lStart, long lEnd);

	// ����Щ�������������߶κϲ�
	void LineScanMergeLines(CScan *scan, long *lineNum);

	// ɾ����Щɨ��ǲ��ѵ�ֱ������
	void RemoveBadLines(const CPosture& pstScanner, const CScan& scan);

public:
	CLineFeatureCreationParam m_Param;    // ֱ�����ɲ���
	CPosture      m_pstScanner;           // ����ͷ�ο���̬

private:
	bool PointTooCloseToSomeCorner(CPnt& pt, vector<CPnt>& ptCorners);

	// �������������й��ߵ���
	bool ColinearRectify();

	// ����ֱ���������ͷ���ռ�
	CLineFeature* NewLineFeature(int nSubType);

public:
	// ���캯��
	CLineFeatureSet(int nNum = 0);

	// �����������캯��
	CLineFeatureSet(const CLineFeatureSet& Obj, bool bFilterDisabled = false);

	~CLineFeatureSet();

	// ���ء�=��������
	void operator = (const CLineFeatureSet& Obj);

	// ����ֱ�����������ɲ���
	void SetCreationParam(CLineFeatureCreationParam* pParam);

	// �������ṩ��ɨ�赽��ֱ����������ֱ����������
	bool CreateFromLocalLines(int nNum, CLineFeature* pLineData);

	// ����ɨ��⵽��Щֱ������ʱ�ļ���ͷ��̬���Ա�������ֱ�������Ĺ۲ⷽ��
	void SetDetectPosture(const CPosture& pstDetect);

	// ��һ��ɨ�輯�г�ȡ��������ֱ�߶�
	bool CreateFromScan(const CScan& scan);

	// ���ݵ�ǰ��̬�����ɨ��뾶��ֱ��ģ��������ֱ����������
	bool CreateFromWorldLines(CPosture& pst, float fMaxRange, int nNumOfLines, CLine* pLine);

	// ������е�ֱ��ɨ��(CLineFeatureSet)
	void Clear();

	// ȡ��ֱ������������
	int GetCount() const {return (int)size();}

	// ȡ��ָ����ֱ������
	CLineFeature& GetLineFeature(int nIdx) { return *at(nIdx); }

	// �����ڴ沢���Ƶ�ǰ��������
	CLineFeatureSet *Duplicate();

	// ������һ��ֱ�߶μ������ֱ�߶μ���
	bool Merge(const CLineFeatureSet& LineScan);

	// ����һ��ֱ������
	bool Add(const CLineFeature& LineFeature);

	// ͨ���ϲ����ߵ��߶����򻯴�ֱ�߶μ���
	bool Simplify(float fMaxGapBetweenLines);

	// �ڼ������ҵ����й��ߵ������������з����¼
	int FindColinearGroups(float fMaxAngDiff, float fMaxDistDiff);

	// ��Թ��ߵ�������ͨ������ߵ�������ʽ���кϲ�
	bool ColinearSimplify(float fMaxAngDiff, float fMaxDistDiff);

	// �����Ż������������ֱ��������
	CLineFeatureSet* Optimize();

	// ɾ��ָ�����߶�
	bool DeleteAt(int nIdx);

	// returns field of view of linescan in rad.
	float FieldOfView(CScan *scan);

	// returns total length of all line segments
	float TotalLength();

	// ȥ�����г��ȶ���minLineLength��ֱ��
	void LengthFilter(float minLineLength);

	// �ж�ֱ��ɨ�輯�Ƿ����ָ���ĵ�
	bool ContainScanPoint(const CScanPoint& sp);

	// �Ƴ�λ��ָ�������ڵ��߶�
	void RemoveWithin(const CRectangle& r);

	// ����������ƽ��
	virtual void Move(float fX, float fY);

	// ������������ת
	virtual void Rotate(CAngle ang, CPnt ptCenter);

#if 0
	// �任���ֲ�����ϵ
	void TransformToLocal(CTransform& trans, CLineFeatureSet& setLocal);

	// �任��ȫ������ϵ
	void TransformToGlobal(CTransform& trans, CLineFeatureSet& setGlobal);
#endif

	void Select(int nIdx, bool bOn);

	// ���ı��ļ�װ��ֱ����������
	virtual int LoadText(FILE* fp);

	// ��ֱ���������ϴ浽�ı��ļ�
	virtual int SaveText(FILE* fp);

	// �Ӷ������ļ�װ��ֱ����������
	virtual int LoadBinary(FILE* fp);

	// ��ֱ���������ϴ浽�������ļ�
	virtual int SaveBinary(FILE* fp);

	// ȡ�������X����
	float LeftMost() { return m_rect.Left(); }

	// ȡ�����ҵ�X����
	float RightMost() { return m_rect.Right(); }

	// ȡ�����ϵ�Y����
	float TopMost() { return m_rect.Top(); }

	// ȡ�����µ�Y����
	float BottomMost() { return m_rect.Bottom(); }

	// ȡ������ֱ�߼��ϵ������ߴ�
	// ȡ�ø�������
	CRectangle GetCoveringRect() const { return m_rect; }

	// ����ֱ�������������ɽǵ㼯��
	int CreateCornerPoints(vector<CPointFeature>& ptCorners);

	// ��ֱ�߼���ת�����ֲ�����ϵ�У�ת�����ԭ����̬�䵽ָ������̬��
	void Transform(CPosture& pstLocal);

	// ��ֱ�߼���ת������������ϵ�У�ת����ԭ����ԭ����̬��Ҫ����ָ������̬
	void InvTransform(CPosture& pstOrigin);

	// �Ը����ĵ�Ϊ���ģ���ָ���ķ�ΧΪ�뾶����ȡ��һ�������Ӽ�
	bool GetSubset(CPnt& ptCenter, float fRange, CLineFeatureSet& Subset);

	// ��Ե�ǰ��ֱ���������ϣ���������Ϻõ�ֱ���������ϡ��͡��ǵ��������ϡ�
	bool SeperateFeatures(CLineFeatureSet& GoodLineFeatureSet, CPointFeatureSet& CornerFeatureSet);

	// �Ӷ������ļ���װ���û��༭����
	bool LoadUserData(FILE* fp);

	// ���û��༭���ݱ��浽�������ļ���
	bool SaveUserData(FILE* fp);

	// ������һ��LineFeatureSet�������û�ʹ������
	bool CopyUserData(const CLineFeatureSet& another);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MFC_VER
	void Dump();

	// ����ֱ����������
	void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, int nPointSize = 1,
		bool bShowActiveSide = false, bool bShowId = false, bool bShowRefPoint = false, 
		int nShowDisabled = DISABLED_FEATURE_UNDERTONE);

	// �ж�һ���������Ƿ�����ĳһ��ֱ������(������Ļ��ֱ�ߴ����ж�)
	int PointHit(const CPnt& pt, float fDistGate);
#endif
};
#endif
