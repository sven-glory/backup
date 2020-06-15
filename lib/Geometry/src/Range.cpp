#include <stdafx.h>
#include "Range.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

//////////////////////////////////////////////////////////////////////////////
//   ʵ�֡���Χ���ϡ��ࡣ

CRangeSet::CRangeSet(const CRangeSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
		push_back(another[i]);
}

//
//   �򼯺������һ���µķ�Χ(�ݹ�ʵ��)��
//
void CRangeSet::operator +=(const CRange& range)
{
#if 0
	switch (size())
	{
	case 0:
		// ���ԭ���ļ���Ϊ�գ���ֱ�ӽ��·�Χ����(��ΪΨһԪ��)
		push_back(range);
		break;

	case 1:
		// ���ԭ���ļ��Ͻ���һ����Ա�����Խ��кϲ�
		if (!(*this)[0].Merge(range))
		{
			push_back(range);       // ���ܺϲ���ֱ�Ӽ���ڶ���Ԫ��
		}
		break;

	default:
	{
		// ������ʱ���ϣ�ֻ����ԭ�����е����һ��Ԫ��
		CRangeSet temp;
		temp.push_back(back());

		// ��������Ԫ�غϲ�
		temp += range;

		// ɾ��ԭ���������һ��Ԫ��
		pop_back();

		// �������ɵ���ʱ���Ͻ��кϲ�
		for (int i = 0; i < temp.size(); i++)
			*this += temp[i];
	}
	break;
	}
#endif

	push_back(range);

	Normalize();
}

//
//   ���һ����Χ���ϡ�
//
void CRangeSet::operator +=(const CRangeSet& another)
{
	for (int i = 0; i < (int)another.size(); i++)
		*this += another[i];
}

//
//   ʵ�ּ�����Ԫ�صļӷ���
//
CRangeSet CRangeSet::operator + (const CRange& range)
{
	CRangeSet temp(*this);
	temp += range;

	return temp;
}

//
//   ʵ�ּ����뼯�ϵļӷ���
//
CRangeSet CRangeSet::operator + (const CRangeSet& another)
{
	CRangeSet temp(*this);
	temp += another;
	return temp;
}

//
//   �ж�һ�����Ƿ��ڸ÷�Χ�����С�
//
bool CRangeSet::Contain(float f) const
{
	for (int i = 0; i < (int)size(); i++)
		if (at(i).Contain(f))
			return true;

	return false;
}

//
//   ��һ������Χ���ϡ����б�׼����
//
void CRangeSet::Normalize()
{
	int nSize = size();
	if (nSize < 2)
		return;

	// ������ʱ�ռ�
	bool* bDelete = new bool[nSize];

	// ��ʼʱ�������������ɾ��
	for (int i = 0; i < nSize; i++)
		bDelete[i] = false;

	// ������кϲ�
	for (int i = 0; i < nSize; i++)
	{
		// ������ɾ������
		if (bDelete[i])
			continue;

		for (int j = 0; j < nSize; j++)
		{
			// ������ɾ������
			if (bDelete[j] || (i == j))
				continue;

			// �������ɺϲ��������j��ɾ��
			if (at(i).Merge(at(j)))
			{
				bDelete[j] = true;
			}
		}
	}

	// ʵ��ɾ�������ѱ�ǡ�delete������
	for (int i = size() - 1; i >= 0; i--)
	{
		if (bDelete[i])
			erase(begin() + i);
	}
	delete[]bDelete;

	// ��������
	for (int i = 0; i < (int)size()-1; i++)
	{
		for (int j = i + 1; j < (int)size(); j++)
		{
			if (at(i) > at(j))
			{
				CRange temp = at(i);
				at(i) = at(j);
				at(j) = temp;
			}
		}
	}
}

//
//   ��ֵ��
//
void CRangeSet::operator = (const vector<CRange>& another)
{
	clear();

	for (int i = 0; i < (int)another.size(); i++)
		push_back(another[i]);
}

