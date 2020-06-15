#include "stdafx.h"
#include "PointMatchListSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

///////////////////////////////////////////////////////////////////////////////

//
//   ��һ��ƥ������м���һ���µ�ƥ���
//   ����ֵ��
//      true  - �����ɹ�
//      false - ����ʧ��
//
bool CPointMatchListSet::Add(const CPointMatchList& lst)
{
	push_back(lst);
	return true;
}

//
//   ��һ��ƥ������в���һ��ƥ���ʹ������ָ��������ƥ��ԡ�
//   ����ֵ��
//       >= 0: �ɹ��ҵ���ƥ�������
//       < 0 : δ�ҵ�
//
int CPointMatchListSet::Search(const CPointMatchPair& Pair1, const CPointMatchPair& Pair2) const
{
	for (int i = 0; i < size(); i++)
		if (at(i).Search(Pair1) >= 0 && at(i).Search(Pair2) >= 0)
			return i;

	return -1;
}

//
//   Ѱ������ƥ���
//
int CPointMatchListSet::FindBestMatch()
{
	int nBest = -1;      // ��ǰ����ƥ�������
	float fMaxScore = 0;

	for (int i = 0; i < size(); i++)
	{
		// ȡ��ǰƥ���ĵ÷�
		float fScore = at(i).m_Evaluation.fScore;

		// �統ǰ��ǰƥ���ĵ�������ڼ�¼��"�������"
		if (fScore > fMaxScore)
		{
			fMaxScore = fScore;        // ��¼�µ�������
			nBest = i;                 // ��¼��ǰ���ű�����
		}
	}

	// ���û�ҵ�
	if (nBest < 0)
		return -1;

	// ���ű�ĵ������С��3����Ϊ���ɹ�
	else if (at(nBest).m_Evaluation.nMatchCount < 3)
		return -1;
	else
		return nBest;
}
