#include "stdafx.h"
#include <math.h>
#include "PointMatchList.h"
#include "CMatrix.h"
#include "CoorTransEquations.h"
#include "DebugTrace.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// ��ʼ��ƥ���ֱ�
float CPointMatchList::m_fDistErrTab[5] = { 0.05f,  0.2f,   0.3f,  0.5f,  0.8f };
int CPointMatchList::m_nScoreTab[5]     = {    10,     8,      6,     4,     2 };

CCoorTransEquations CPointMatchList::m_PointEquations((MAX_POINT_MATCH_COUNT+1) * 2, 4);

//////////////////////////////////////////////////////////////////////////////

//
//   ���������һ����ƥ��ԡ�
//
bool CPointMatchList::Add(const CPointMatchPair& Pair)
{
	push_back(Pair);
	return true;
}

//
//    ��һ��ƥ��԰��ֲ��㼫����С�����˳����뵽ƥ����С�
//
bool CPointMatchList::AddInRadiusOrder(const CPointMatchPair& Pair)
{
	// ���ҵ�����Ӧ�ڵĲ����i
	bool bFound = false;
	int i;
	for (i = 0; i < (int)size(); i++)
	{
		if (Pair.m_ptLocal.r < at(i).m_ptLocal.r)
		{
			bFound = true;
			break;
		}
	}

	// ���δ�ҵ�����㣬˵���˼������Ӧ���뵽��β
	if (!bFound)
		push_back(Pair);
	else
	{
		// ��λ��i����������
		insert(begin() + i, Pair);
	}

	return true;
}

//
//   ��һ��ƥ����в���һ��ƥ��ԡ�
//
//   ����ֵ��
//       ���ҵ����򷵻�����λ�õ������ţ����򷵻�-1��
//
int CPointMatchList::Search(const CPointMatchPair& Pair) const
{
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i).m_ptLocal.id == Pair.m_ptLocal.id && at(i).m_ptWorld.id == Pair.m_ptWorld.id)
			return i;
	}

	return -1;
}

//
//   �������ṩ�ľֲ�����ƥ����в������Ӧ��ƥ��ԡ�
//
//   ����ֵ��
//       ���ҵ����򷵻�����λ�õ������ţ����򷵻�-1��
//
int CPointMatchList::SearchByLocalPoint(const CPnt& ptLocal) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_ptLocal.id == ptLocal.id)
			return i;

	return -1;
}

//
//   �������ṩ���������ƥ����в������Ӧ��ƥ��ԡ�
//
//   ����ֵ��
//       ���ҵ����򷵻�����λ�õ������ţ����򷵻�-1��
//
int CPointMatchList::SearchByWorldPoint(const CPnt& ptWorld) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_ptWorld.id == ptWorld.id)
			return i;

	return -1;
}

//
//   ����ʵ�����õ�ƥ��Ե�������
//
void CPointMatchList::LimitMatchPairNum(int nLimit)
{
	if ((nLimit > 0) && (size() > nLimit))
		erase(begin() + nLimit, end());
}

//
//   ��ƥ�������Щ�ǡ�һ��һ��ƥ���������Ƴ���
//
void CPointMatchList::Filter()
{
	int nCount = size();

	// Ϊ��ɾ������Ƿ�����ʱ�ռ�
	bool* pRemove = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		pRemove[i] = false;

	for (int i = 0; i < nCount - 1; i++)
	{
		CPointMatchPair& pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			CPointMatchPair& pair2 = at(j);

			// ��Ƕ���ƥ����
			if (pair1.m_ptLocal.id == pair2.m_ptLocal.id ||
				 pair1.m_ptWorld.id == pair2.m_ptWorld.id)
			{
				pRemove[i] = true;
				pRemove[j] = true;
			}
		}
	}

	// ����Щ���Ϊ���Ƴ�����������Ƴ�
	for (int i = nCount - 1; i >= 0; i--)
	{
		if (pRemove[i])
			erase(begin() + i);
	}

	// �ͷš�ɾ�������
	delete []pRemove;
}

//
//   ���ɹ������е�Ե���С�������ݡ�
//
bool CPointMatchList::CreateLeastSquareData(CLinearEquations* pLsm)
{
	float a[2][4], b[2];

	int nMaxCount = size();
	if (nMaxCount > MAX_POINT_MATCH_COUNT)
		nMaxCount = MAX_POINT_MATCH_COUNT;

	for (int i = 0; i < nMaxCount; i++)
	{
		// ���ݵ���������о�������
		at(i).MakeLeastSquareData(a, b);
	
		// �����������ݼ�����С�����������
		if (!pLsm->AddRow(a[0], b[0]))
			return false;

		if (!pLsm->AddRow(a[1], b[1]))
			return false;
	}

	return true;
}

//
//   ����ƥ��������������֮�������任��ϵ��
//
bool CPointMatchList::FindTransform()
{
	// �������þ�������
	m_PointEquations.Start();

	float a[2][4], b[2];
	for (int i = 0; i <(int)size(); i++)
	{
		// ���ݵ���������о�������
		at(i).MakeLeastSquareData(a, b);
	
		// �����������ݼ�����С�����������
		if (!m_PointEquations.AddRow(a[0], b[0]))
			return false;

		if (!m_PointEquations.AddRow(a[1], b[1]))
			return false;
	}

	// �����С���˷�
	float f[4];
	if (m_PointEquations.LeastSquareSolve(f, 4))
	{
		float sin = f[0];
		float cos = f[1];
		float angle = (float)atan2(sin, cos);

		float x = f[2];
		float y = f[3];
		m_Trans.Create(x, y, angle);

		// �����������
		EvaluateTransform();
		return true;
	}
	else
		return false;
}

//
//   ������������֮���ƥ��̶Ȳ�����
//
void CPointMatchList::EvaluateTransform()
{
	int nCount = size();
	if (nCount == 0)
		return;

	// ͳ��ƥ����֮���ƽ������
	int nCountMatched = 0;
	int nTotalScore = 0;
	float fTotalDist = 0;

	for (int i = 0; i < nCount; i++)
	{
		CPnt& pnt1 = at(i).m_ptLocal;
		CPnt& pnt2 = at(i).m_ptWorld;
		at(i).m_ptLocalToWorld = m_Trans.GetWorldPoint(pnt1);

		// ����õ��ƥ��������
		float fDist = at(i).m_ptLocalToWorld.DistanceTo(pnt2);

		// ����õ�����С�ڹ涨������
		if (fDist < m_fErrGate)
		{
			nCountMatched++;
			fTotalDist += fDist;
		}

		// ���ݾ��������
		int nScore = 0;
		for (int j = 0; j < 5; j++)
		{
			// �����
			if (fDist <= m_fDistErrTab[j])
			{
				nScore = m_nScoreTab[j];
				break;
			}
		}
		nTotalScore += nScore;
	}

	// ͳ���������
	m_Evaluation.fMeanErr = fTotalDist / nCount;
	m_Evaluation.nMatchCount = nCountMatched;
	m_Evaluation.fMatchRate = (float)nCountMatched / (float)nCount;
	m_Evaluation.fScore = (float)nTotalScore;
}

//
//   ���������㼯֮���ƥ��̶Ȳ�����
//
//   ����ֵ��ƥ��ɹ��ĵ��������
//
int CPointMatchList::EvaluateTransform(CTransform& Transform, float* pAveErr, float* pScore)
{
	int nCount = size();
	if (nCount == 0)
		return 0;

	// ͳ��ƥ����֮���ƽ������
	int nCountMatched = 0;
	int nTotalScore = 0;
	float fTotalDist = 0;

	for (int i = 0; i < nCount; i++)
	{
		CPnt& pnt1 = at(i).m_ptLocal;
		CPnt& pnt2 = at(i).m_ptWorld;
		at(i).m_ptLocalToWorld = Transform.GetWorldPoint(pnt1);

		// ����õ��ƥ��������
		float fDist = at(i).m_ptLocalToWorld.DistanceTo(pnt2);

		// ����õ�����С�ڹ涨������
		if (fDist < m_fErrGate)
		{
			nCountMatched++;
			fTotalDist += fDist;
		}

		// ���ݾ��������
		int nScore = 0;
		for (int j = 0; j < 5; j++)
		{
			// �����
			if (fDist <= m_fDistErrTab[j])
			{
				nScore = m_nScoreTab[j];
				break;
			}
		}
		nTotalScore += nScore;
	}

	// ����ƽ���������
	float fAveDist = fTotalDist / nCount;

	// ����ƽ�����
	if (pAveErr != NULL)
		*pAveErr = fAveDist;

	// �����ܵ÷�
	if (pScore != NULL)
		*pScore = nTotalScore;

	// ����ƥ������
	return nCountMatched;
}

#ifdef _MSC_VER

//
//   �鿴��ƥ���
//
void CPointMatchList::Dump()
{
	for (int i = 0; i < (int)size(); i++)
	{
		CPointMatchPair& Pair = at(i);
		
		CPnt& ptLocal = Pair.m_ptLocal;
		CPnt& ptWorld = Pair.m_ptWorld;

		DebugTrace(_T("Local: (%.2f, %.2f) --------  World: (%.2f, %.2f)\n"), 
			ptLocal.x, ptLocal.y, ptWorld.x, ptWorld.y);
	}
}
#endif
