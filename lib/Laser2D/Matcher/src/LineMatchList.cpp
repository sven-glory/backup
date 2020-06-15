#include "stdafx.h"
#include <math.h>
#include "LineMatchList.h"
#include "LineFeature.h"
#include "DebugTrace.h"
#include "Combination.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define MAX_LINE_MATCH_ERROR              0.5f

// ��ʼ��ƥ���ֱ�
float CLineMatchList::m_fDistErrTab[5] = { 0.1f, 0.3f, 0.5f, 0.8f, 1.2f };
int CLineMatchList::m_nScoreTab[5] = { 10, 8, 6, 4, 2 };

// ����ֱ��ƥ���ı任����
CCoorTransEquations CLineMatchList::m_LineEquations((MAX_LINE_MATCH_COUNT+1) * 3, 4);

///////////////////////////////////////////////////////////////////////////////
// for CLineMatchList member function

//
//   ���������һ���߶�ƥ��ԡ�
//
bool CLineMatchList::Add(const CLineMatchPair& Pair)
{
	push_back(Pair);
	return true;
}

//
//   ��һ��ƥ����в���һ��ƥ��ԡ�
//
//   ����ֵ��
//       ���ҵ����򷵻�����λ�õ������ţ����򷵻�-1��
//
int CLineMatchList::Search(const CLineMatchPair& Pair)
{
	for (int i = 0; i < (int)size(); i++)
	{
		if (at(i).m_lnLocal.m_nId == Pair.m_lnLocal.m_nId &&
			at(i).m_lnWorld.m_nId == Pair.m_lnWorld.m_nId)
			return i;
	}

	return -1;
}

//
//   �������ṩ�ľֲ�ֱ�߶���ƥ����в������Ӧ��ƥ��ԡ�
//
//   ����ֵ��
//       ���ҵ����򷵻�����λ�õ������ţ����򷵻�-1��
//
int CLineMatchList::SearchByLocalLine(const CLine& ln)
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_lnLocal.m_nId == ln.m_nId)
			return i;

	return -1;
}

//
//   �������ṩ������ֱ�߶���ƥ����в������Ӧ��ƥ��ԡ�
//
//   ����ֵ��
//       ���ҵ����򷵻�����λ�õ������ţ����򷵻�-1��
//
int CLineMatchList::SearchByWorldLine(const CLine& ln)
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).m_lnWorld.m_nId == ln.m_nId)
			return i;

	return -1;
}

//
//   ��ƥ�������Щ�ǡ�һ��һ��ƥ���������Ƴ���
//
void CLineMatchList::Filter(bool bDeleteParallel)
{
	int nCount = size();
	bool* pRemove = new bool[nCount];
	for (int i = 0; i < nCount; i++)
		pRemove[i] = false;

	for (int i = 0; i < nCount - 1; i++)
	{
		// ������Щ�ѱ��Ϊ��ɾ��������
		if (pRemove[i])
			continue;

		CLineMatchPair& Pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			if (pRemove[j])
				continue;

			CLineMatchPair& Pair = at(j);

			// ��Ƕ���ƥ����
			if (Pair1.m_lnLocal.m_nId == Pair.m_lnLocal.m_nId)
			{
				// ����������߶ι��ߣ���ɾ������
				if (Pair.m_lnLocal.IsColinearWith(Pair1.m_lnLocal, 5*PI/180, 0.080f) || 
					 (bDeleteParallel && Pair.m_lnLocal.IsParallelTo(Pair1.m_lnLocal)))
				{
					pRemove[j] = true;
				}

#if 0
				// ����������ɾ��
				else
				{
					pRemove[i] = true;
					pRemove[j] = true;
				}
#endif
			}
			else if (Pair1.m_lnWorld.m_nId == Pair.m_lnWorld.m_nId)
			{
				// ����������߶ι��ߣ���ɾ������֮һ
				if (Pair.m_lnWorld.IsColinearWith(Pair1.m_lnWorld, 5*PI/180, 0.080f) ||
					(bDeleteParallel && Pair.m_lnWorld.IsParallelTo(Pair1.m_lnWorld)))
				{
					pRemove[j] = true;
				}
#if 0
				// ����������ɾ��
				else
				{
					pRemove[i] = true;
					pRemove[j] = true;
				}
#endif
			}
		}
	}

	for (int i = 0; i < nCount - 1; i++)
	{
		// ������Щ�ѱ��Ϊ��ɾ��������
		if (pRemove[i])
			continue;

		CLineMatchPair& Pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			if (pRemove[j])
				continue;

			CLineMatchPair& Pair = at(j);

			if (Pair.m_lnWorld.IsColinearWith(Pair1.m_lnWorld, 5 * PI / 180, 0.080f) ||
				(bDeleteParallel && Pair.m_lnWorld.IsParallelTo(Pair1.m_lnWorld)))
			{
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

	delete[]pRemove;
}

//
//   ���ݵ�ǰ��(���ܺ��С�һ�Զࡱƥ���)ֱ��ƥ����������п��ܵġ�һ��һ��ƥ�䷽�����ϡ�
//
void CLineMatchList::CreateAllOptions(vector<CLineMatchList>* pLineList)
{
	int nCount = size();
	int* pNext = new int[nCount];
	for (int i = 0; i < nCount; i++)
		pNext[i] = -1;

	// ��local�߶���ͬ������Ա�ǣ��Ա�һ�����ȡ
	for (int i = 0; i < nCount - 1; i++)
	{
		CLineMatchPair& Pair1 = at(i);

		for (int j = i + 1; j < nCount; j++)
		{
			CLineMatchPair& Pair = at(j);

			// ��Ƕ���ƥ����
			if (Pair1.m_lnLocal.m_nId == Pair.m_lnLocal.m_nId)
			{
				pNext[i] = j;
			}
#if 0
			else if (Pair1.m_lnWorld.m_nId == Pair.m_lnWorld.m_nId)
			{
			}
#endif
		}
	}

	// ׼���������еġ�һ��һ��ƥ�䷽���ļ���
	vector<vector<CLineMatchPair>> vvPairs;

	// ���ҵ����о��ж���ƥ���ϵ�ġ�ƥ��ԡ���������Щ��ƥ��ԡ�����vvPair������
	int j = 0;
	for (int i = 0; i < nCount; i++)
	{
		// ����˶���ƥ�����ѱ��������������
		if (pNext[i] == -2)
			continue;

		// �����δ������Ķ���ƥ���
		else if (pNext[i] >= 0)
		{
			vector<CLineMatchPair> vPairs;

			vPairs.push_back(at(i));
			int nNext = pNext[i];

			pNext[i] = -2;

			// �����Localֱ����ͬ������ƥ����ռ���vPairs������
			while (nNext >= 0)
			{
				vPairs.push_back(at(nNext));
				int nNext = pNext[nNext];
				pNext[nNext] = -2;
			}

			// �����о��ж���ƥ���ƥ��Լ��϶����뵽vvPairs������
			vvPairs.push_back(vPairs);
		}
	}

	CLineMatchList MinList = *this;

	// ����Щ���Ϊ���Ƴ�����������Ƴ�
	for (int i = MinList.size() - 1; i >= 0; i--)
	{
		if (pNext[i] == -2)
			MinList.erase(MinList.begin() + i);
	}

	delete[]pNext;


	// ����׼�����������
	int nvvCount = vvPairs.size();
	if (nvvCount > 0)
	{
		int* pSize = new int[nvvCount];
		int* pOutput = new int[nvvCount];

		CCombination Combination;
		Combination.Create(nvvCount, pSize);

		// ����������������С�ֱ��ƥ�伯�ϡ��ļ���
		bool bDone = false;
		while (!bDone)
		{
			bDone = !Combination.Generate(pOutput);
			if (!bDone)
			{
				CLineMatchList newList = MinList;
				for (int i = 0; i < nvvCount; i++)
					newList.push_back(vvPairs[i][pOutput[i]]);

				pLineList->push_back(newList);
			}
		}
		delete[]pSize;
		delete[]pOutput;
	}
	else
	{
		pLineList->push_back(MinList);
	}
}

int CLineMatchList::ApplicablePairCount()
{
	// �ж��Ƿ����㹻���ж�λ��ֱ��ƥ���
	int nCount = 0;
	for (int i = 0; i < GetCount() - 1; i++)
	{
		CLine& Line1 = at(i).m_lnLocal;
		for (int j = i + 1; j < GetCount(); j++)
		{
			CLine& Line2 = at(j).m_lnLocal;
			CAngle angDiff = Line1.AngleToUndirectionalLine(Line2);

			// �����ֱ�߼нǴ���30�ȣ�������ڶ�λ
			if (angDiff > CAngle(30.0f, IN_DEGREE))
				nCount++;
		}
	}

	// ���ؿ��õ�ƥ��ֱ�߶�����
	return nCount;
}

int CLineMatchList::FindBestMatchList(vector<CLineMatchList>* pLineList)
{
	int nBest = -1;
	float fMaxScore = 0;

	CCoorTransEquations lsm((MAX_LINE_MATCH_COUNT + 1)* 3, 4);

	for (int i = 0; i < (int)pLineList->size(); i++)
	{
		lsm.Start();

		CLineMatchList& LineMatchList = pLineList->at(i);
		if (LineMatchList.ApplicablePairCount() < 2)
			continue;

		LineMatchList.CreateLeastSquareData(&lsm);

		CTransform trans;
		float f[4];
		if (lsm.LeastSquareSolve(f, 4))
		{
			float sin = f[0];
			float cos = f[1];
			float angle = (float)atan2(sin, cos);

			float x = f[2];
			float y = f[3];
			trans.Create(x, y, angle);

			// �ڴ���Ҫ�����Ľ�������жϣ�������̫������Ϊ���ʧ��
			float fScore2 = 0, fAveErr1 = 0, fAveErr2 = 0;
			int nCountLineMatched = LineMatchList.EvaluateTransform(trans, &fAveErr2, &fScore2);

			if (fScore2 > fMaxScore)
			{
				fMaxScore = fScore2;
				nBest = i;
			}
		}
	}

	return nBest;
}

//
//   �ж��Ƿ�ƥ����е�����ƥ��Զ���ƽ���ߡ�
//
bool CLineMatchList::AllParallel()
{
	// �������һ��ֱ��ƥ��ԣ��򷵻�true
	if (size() == 1)
		return true;

	// ȡ�õ�һ��ֱ��
	CLineMatchPair& Pair1 = at(0);
	CLine& lnWorld1 =at(0).m_lnWorld;

	// ������ֱ���������һ�����жԱȣ����Ƿ���֮ƽ��
	for (int i = 1; i < (int)size(); i++)
	{
		CLineMatchPair& Pair = at(i);

		// �����ƽ�У�ֱ�ӷ���false
		if (!Pair.m_lnWorld.IsParallelTo(lnWorld1))
			return false;
	}

	return true;
}

//
//   ���ɹ�������ֱ�߶ζԵ���С�������ݡ�
//
bool CLineMatchList::CreateLeastSquareData(CCoorTransEquations* pLsm)
{
	float a[3][4], b[3];

	int nMaxCount = size();
	if (nMaxCount > MAX_LINE_MATCH_COUNT)
		nMaxCount = MAX_LINE_MATCH_COUNT;

	for (int i = 0; i < nMaxCount; i++)
	{
		// ����ֱ�߶ζ��������о�������
		at(i).MakeLeastSquareData(a, b);
	
		// �����������ݼ�����С�����������
		for (int j = 0; j < 3; j++)
			if (!pLsm->AddRow(a[j], b[j]))
				return false;
	}

	return true;
}

//
//   ����ƥ����������ֱ�߼���֮�������任��ϵ��
//
bool CLineMatchList::FindTransform()
{
	// �������þ�������
	m_LineEquations.Start();

	if (!CreateLeastSquareData(&m_LineEquations))
		return false;
	
	m_LineEquations.Dump();

	float f[4];
	if (m_LineEquations.LeastSquareSolve(f, 4))
	{
		float sin = f[0];
		float cos = f[1];
		float angle = (float)atan2(sin, cos);

		float x = f[2];
		float y = f[3];
		m_Trans.Create(x, y, angle);

		// �ڴ���Ҫ�����Ľ�������жϣ�������̫������Ϊ���ʧ��
		float fScore2 = 0, fAveErr1 = 0, fAveErr2 = 0;
		int nCountLineMatched = EvaluateTransform(m_Trans, &fAveErr2, &fScore2);
#if 0
		if (nCountLineMatched >= 2)
			return true;
		else
			return false;
#endif

		return true;
	}
}

//
//   ����ƥ��̶Ȳ�����
//
int CLineMatchList::EvaluateTransform(CTransform& Transform, float* pAveErr, float* pScore)
{
	if (size() == 0)
		return 0;

	// ͳ��ƥ���߶�֮���ƽ���ۺϾ���
	int nCountMatched = 0;
	float fTotalDist = 0;
	int nTotalScore = 0;

	for (int i = 0; i < (int)size(); i++)
	{
		CLineMatchPair& Pair = at(i);

		// ���㱾��ֱ�߶α任����������ϵ�е�λ��
		CLine ln1 = Transform.GetWorldLine(Pair.m_lnLocal);

		// ��������ƥ���߶ε��ۺϾ���
		CLineFeature Feature1, Feature2;
		Feature1.Create(ln1);
		Feature2.Create(Pair.m_lnWorld);
		float fDist = Feature1.TransformCostTo(Feature2);

		if (fDist < MAX_LINE_MATCH_ERROR)
			nCountMatched++;

		fTotalDist += fDist;

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
	float fAveDist = fTotalDist / size();

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

void CLineMatchList::Dump()
{
	for (int i = 0; i < (int)size(); i++)
	{
		CLineMatchPair& Pair = at(i);
		
		CLine& lnLocal = Pair.m_lnLocal;
		CLine& lnWorld = Pair.m_lnWorld;

//		DebugTrace(_T("Local:[%d, %.3f, %.3f, %.3f] (%.2f, %.2f)-(%.2f, %.2f) --------  World: [%d, %.3f, %.3f, %.3f](%.2f, %.2f)-(%.2f, %.2f)\n"), 
		DebugTrace(_T("[\t%d\t%.3f\t%.3f\t%.3f\t]\t%.2f\t%.2f\t%.2f\t%.2f\t\t[\t%d\t%.3f\t%.3f\t%.3f\t]\t%.2f\t%.2f\t%.2f\t%.2f\n"), 
			lnLocal.m_nId, lnLocal.a, lnLocal.b, lnLocal.c,
			lnLocal.m_ptStart.x, lnLocal.m_ptStart.y, lnLocal.m_ptEnd.x, lnLocal.m_ptEnd.y,

			lnWorld.m_nId, lnWorld.a, lnWorld.b, lnWorld.c,
			lnWorld.m_ptStart.x, lnWorld.m_ptStart.y, lnWorld.m_ptEnd.x, lnWorld.m_ptEnd.y);
	}
}
#endif
