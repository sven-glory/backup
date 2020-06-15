#ifndef __CLineMatchList
#define __CLineMatchList

// ע��:�ɴ洢���߶ζ���������ʵ�ʲ���ƥ�����������

#define MAX_LINE_MATCH_COUNT      10      // ���ɲ�����С����ƥ���ֱ�߶�����

#include <vector>
#include "LineMatchPair.h"
#include "CoorTransEquations.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   "CLineMatchList"��Ķ��塣
class CLineMatchList : public vector<CLineMatchPair>
{
private:
	static float m_fDistErrTab[5];
	static int m_nScoreTab[5];

public:
	CTransform   m_Trans;
	static CCoorTransEquations m_LineEquations;

public:
	CLineMatchList() {}

	// ȡ�ñ���ƥ��Ե�����
	int GetCount() {return (int)size();}

	// Ϊƥ�����������任��ϵ
	void SetTransform(const CTransform& _trans) { m_Trans = _trans; }

	// ȡ������任��ϵ
	CTransform& GetTransform() { return m_Trans; }

	// ��һ��ƥ��Լ��뵽ƥ�����
	bool Add(const CLineMatchPair& Pair);

	// ��һ��ƥ����в���һ��ƥ���
	int Search(const CLineMatchPair& Pair);

	// �������ṩ�ľֲ�ֱ�߶���ƥ����в������Ӧ��ƥ���
	int SearchByLocalLine(const CLine& ln);

	// �������ṩ������ֱ�߶���ƥ����в������Ӧ��ƥ���
	int SearchByWorldLine(const CLine& ln);

	// ��ƥ�������Щ�ǡ�һ��һ��ƥ���������Ƴ�
	void Filter(bool bDeleteParallel = false);

	void CreateAllOptions(vector<CLineMatchList>* pLineList);
	int FindBestMatchList(vector<CLineMatchList>* pLineList);
	int ApplicablePairCount();

	// �ж��Ƿ�ƥ����е�����ƥ��Զ���ƽ����
	bool AllParallel();

	// ���ɹ��������߶�ƥ��Ե���С��������
	bool CreateLeastSquareData(CCoorTransEquations* pLsm);

	// ����ƥ����������ֱ�߼���֮�������任��ϵ
	bool FindTransform();

	// ����ƥ��̶Ȳ���
	int EvaluateTransform(CTransform& Transform, float* fAverageErr, float* pMatchingScore);

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
