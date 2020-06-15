#ifndef __CLineFeature
#define __CLineFeature

#include "MultiSegLine.h"
#include "Feature.h"

// ��������
#define GENERIC_LINE_FEATURE             0         // һ��ֱ������
#define SINGLE_SIDED_LINE_FEATURE        1         // ����ֱ������

///////////////////////////////////////////////////////////////////////////////
//   ���塰���������ࡣ

class CLineFeature : public CFeature, public CMultiSegLine
{
public:
	long  m_lStart;
	long  m_lEnd;              // point numbers in scan
	float m_fSigma2;           // ƥ�����

	int   m_nWhichSideToUse;   // ʹ��ֱ�ߵ���һ��(1-ǰ��;2-����;3-����)
	bool  m_bSelected;         // �������߶��Ƿ�ѡ��(������Ļ�༭����)

	bool     m_bCreateRef;     // �Ƿ������ɲο���
	CPnt m_ptRef;          // �ο����λ��
	CPnt m_ptProjectFoot;  // ͶӰ���λ��

public:
	CLineFeature(CPnt& ptStart, CPnt& ptEnd);
	CLineFeature();

	// ����һ������
	virtual CLineFeature* Duplicate() const;

	CLineFeature& GetLineFeatureObject() {return *this; }
	
	// ���ı��ļ���������
	virtual int LoadText(FILE* file);

	// ������д�뵽�ı��ļ�
	virtual int SaveText(FILE* file);

	// �Ӷ������ļ���������
	virtual int LoadBinary(FILE* file);

	// ������д�뵽�������ļ�
	virtual int SaveBinary(FILE* file);

	// �����߶�
	void Create(CPnt& ptStart, CPnt& ptEnd);

	// �����߶�
	void Create(CLine& ln);

	// ���ݶ������������
	void Create(CMultiSegLine& MultiSegLine);
	
	// ����ɨ��⵽��ֱ������ʱ�ļ���ͷ��̬���Ա������Ч�Ĺ۲⳯��
	void SetDetectPosture(const CPosture& pstDetect);

	// �жϱ��߶��Ƿ�����һ�߶����ص���
	bool IsOverlapWith(CLineFeature& Feature2);

	// ���˶����������һ�������(������ߵĻ�)���кϲ�
	bool ColinearMerge(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines);
	bool ColinearMerge1(CLineFeature& Feature2, float fMaxAngDiff, float fMaxDistDiff, float fMaxGapBetweenLines);

	// ����������ƽ��
	virtual void Move(float fX, float fY);

	// ������������ת
	virtual void Rotate(CAngle ang, CPnt ptCenter);

	// ѡ��/ȡ��ѡ�д��߶�
	void Select(bool bOn) {m_bSelected = bOn;}

	// ��pstScannerΪ�۲���̬���жϱ�ֱ�������Ƿ�����������һֱ������������׼
	bool RegisterWith(CLineFeature& another, CPosture& pstScanner, float fDistGate, float fAngGate);

	// ���㱾ֱ����������һֱ�������ı任����
	float TransformCostTo(const CLineFeature& another) const;

	// �жϸ�ֱ�������ܷ�������һ��ֱ����������һ�����ǵ㡱
	bool MakeCorner(const CLineFeature& another, float fMinAngle, float fMaxAngle, float fMaxGap, CPnt& ptCorner);

	// �����������任
	virtual void Transform(const CFrame& frame);

	// ����������任
	virtual void InvTransform(const CFrame& frame);

#ifdef _MSC_VER
	// (����Ļ������)����ָ���ĵ��Ƿ����߶���
	bool HitTest(CScreenReference& ScrnRef, CPoint point);

	// ����Ļ����ʾ��ֱ������
	virtual void Plot(CScreenReference& ScrnRef, CDC* pDC, COLORREF crColor, COLORREF crSelected, 
		int nSize, int nShowDisabled = DISABLED_FEATURE_UNDERTONE, bool bShowSuggested = true);
#endif
};
#endif
