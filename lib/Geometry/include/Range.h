#ifndef __CRange
#define __CRange

#include <vector>

using namespace std;

/////////////////////////////////////////////////////////////////////
// ���ڷ�Χ��������
class CRange
{
public:
	float fFrom;     // ��ʼ��
	float fTo;       // ��ֹ��

public:
	CRange(float _from, float _to)
	{
		fFrom = _from;
		fTo = _to;
	}

	CRange() 
	{
		fFrom = 0;
		fTo = 0;
	}

	CRange& CRangeObject()
	{
		return *this;
	}

	// �жϴ˷�Χ�Ƿ�Ϸ�
	bool IsValid() { return fFrom >= fTo; }

	// �жϴ˷�Χ�Ƿ������һ��
	bool operator > (const CRange& another)
	{
		return (fFrom > another.fTo);
	}

	// �жϴ˷�Χ�Ƿ�С����һ��
	bool operator < (const CRange& another)
	{
		return (fTo < another.fFrom);
	}

	// �ж�һ�����Ƿ��ڸ÷�Χ��
	bool Contain(float f) const
	{
		return (f >= fFrom && f <= fTo);
	}

	// �жϴ˷�Χ�Ƿ�����һ����Χ���ص���
	bool IsOverlap(const CRange& another)
	{
		if (*this > another || *this < another)
			return false;
		else
			return true;
	}

	// ���Խ�������Χ���кϲ�
	bool Merge(const CRange& another)
	{
		if (!IsOverlap(another))
			return false;

		if (another.fFrom < fFrom)
			fFrom = another.fFrom;

		if (another.fTo > fTo)
			fTo = another.fTo;

		return true;
	}
};

/////////////////////////////////////////////////////////////////////
// ���塰��Χ�εļ��ϡ���

class CRangeSet : public vector<CRange>
{
private:

public:
	CRangeSet() {}

	// ֱ�Ӱ���һ���Ϲ���
	CRangeSet(const CRangeSet& another);

	// ���һ����Χ
	void operator +=(const CRange& range);

	// ���һ����Χ����
	void operator +=(const CRangeSet& another);

	// ʵ�ּ�����Ԫ�صļӷ�
	CRangeSet operator + (const CRange& range);

	// ʵ�ּ����뼯�ϵļӷ�
	CRangeSet operator + (const CRangeSet& another);

	// ��ֵ
	void operator = (const vector<CRange>& another);

	// �ж�һ�����Ƿ��ڸ÷�Χ������
	bool Contain(float f) const;

	// ��һ������Χ���ϡ����б�׼��
	void Normalize();
};
#endif
