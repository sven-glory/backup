#ifndef __CPointMatchListSet
#define __CPointMatchListSet

#include <vector>
#include "PointMatchList.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//   ���塰��ƥ����ϡ��ࡣ
class CPointMatchListSet : private vector<CPointMatchList>
{
public:
	CPointMatchListSet() {}

	// ����һ��ƥ�����
	void Clear() { clear(); }

	// ȡ��һ��ƥ�����������ƥ��������
	int GetCount() { return (int)size(); }

	// ��һ��ƥ�������ȡ��һ��ƥ���
	CPointMatchList& GetList(int i) { return at(i); }

	// ��һ��ƥ������м���һ���µ�ƥ���
	bool Add(const CPointMatchList& lst);

	// ��ƥ������в���һ��ƥ���ʹ������ָ��������ƥ���
	int Search(const CPointMatchPair& Pair1, const CPointMatchPair& Pair2) const;

	// Ѱ������ƥ���
	int FindBestMatch();
};
#endif
