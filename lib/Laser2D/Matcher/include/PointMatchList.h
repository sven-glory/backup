#ifndef __CPointMatchList
#define __CPointMatchList

#define MAX_POINT_MATCH_COUNT     10      // ���ɲ�����С����ƥ���ֱ�߶�����

#include <vector>
#include "PointMatchPair.h"
#include "CoorTransEquations.h"

using namespace std;

#define DEFAULT_POINT_MATCH_ERR_GATE        0.3f      // ȱʡ�ĵ�ƥ���������

//
//   ƥ����������
//
struct CMatchEvaluation
{
	int   nMatchCount;             // ƥ������
	float fMatchRate;              // ƥ��ɹ��ĵ���ռ����(0%~100%)
	float fMeanErr;                // ƽ��ƥ�����(��λ:m)
	float fScore;                  // �ۺϵ÷�
};

///////////////////////////////////////////////////////////////////////////////
//   "CPointMatchList"��Ķ��塣
class CPointMatchList : public vector<CPointMatchPair>
{
private:
	static float m_fDistErrTab[5];
	static int   m_nScoreTab[5];
	float        m_fErrGate;          // ��ƥ��������ޣ����ڴ���������Ϊδƥ����

public:
	CTransform   m_Trans;
	CMatchEvaluation m_Evaluation;

	static CCoorTransEquations m_PointEquations;

public:
	CPointMatchList() { m_fErrGate = DEFAULT_POINT_MATCH_ERR_GATE; }

	// ���õ�ƥ���������
	void SetErrGate(float fErrGate) { m_fErrGate = fErrGate; }

	// �������ƥ���
	void Clear() { clear(); }

	// ȡ�ñ���ƥ��Ե�����
	int GetCount() {return (int)size();}

	// ȡ��ָ����ƥ���
	CPointMatchPair& GetPair(int nIdx) { return at(nIdx); }

	// Ϊƥ�����������任��ϵ
	void SetTransform(const CTransform& _trans) {m_Trans = _trans;}

	// ȡ������任��ϵ
	CTransform& GetTransform() { return m_Trans; }

	// ��һ��ƥ��Լ��뵽ƥ�����
	bool Add(const CPointMatchPair& Pair);

	// ��һ��ƥ��԰��ֲ��㼫����С�����˳����뵽ƥ�����
	bool AddInRadiusOrder(const CPointMatchPair& Pair);

	// ��һ��ƥ����в���һ��ƥ���
	int Search(const CPointMatchPair& Pair) const;

	// �������ṩ�ľֲ�����ƥ����в������Ӧ��ƥ���
	int SearchByLocalPoint(const CPnt& ptLocal) const;

	// �������ṩ���������ƥ����в������Ӧ��ƥ���
	int SearchByWorldPoint(const CPnt& ptWorld) const;

	// ����ʵ�����õ�ƥ��Ե�����
	void LimitMatchPairNum(int nLimit);

	// ��ƥ�������Щ�ǡ�һ��һ��ƥ���������Ƴ�
	void Filter();

	// ���ɹ������е�Ե���С��������
	bool CreateLeastSquareData(CLinearEquations* pLsm);

	// ����ƥ��������������֮�������任��ϵ
	bool FindTransform();

	// ������������֮���ƥ��̶Ȳ���
	void EvaluateTransform();

	// ����ƥ��̶Ȳ���
	int EvaluateTransform(CTransform& Transform, float* fAverageErr, float* pMatchingScore);

#ifdef _MFC_VER
	// �鿴��ƥ���
	void Dump();
#endif
};
#endif
