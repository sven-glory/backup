#ifndef __CCorrList
#define __CCorrList

class CCsmScan;

// �������ƵĶ�Ӧ��ϵ��
class CCorrList
{
public:
	int  m_nCount;
	int* m_pIdx;

public:
	CCorrList()
	{
		m_nCount = 0;
		m_pIdx = NULL;
	}

	CCorrList(const CCorrList& another)
	{
		m_pIdx = NULL;
		Create(another.m_nCount);

		for (int i = 0; i < m_nCount; i++)
			m_pIdx[i] = another.m_pIdx[i];
	}

	CCorrList& operator = (const CCorrList& another)
	{
		Create(another.m_nCount);

		for (int i = 0; i < m_nCount; i++)
			m_pIdx[i] = another.m_pIdx[i];

		return *this;
	}

	~CCorrList()
	{
		Clear();
	}

	void Clear()
	{
		if (m_pIdx != NULL)
		{
			delete[]m_pIdx;
			m_pIdx = NULL;
		}
		m_nCount = 0;
	}

	bool Create(int nCount)
	{
		Clear();

		m_pIdx = new int[nCount];
		if (m_pIdx != NULL)
		{
			m_nCount = nCount;
			for (int i = 0; i < nCount; i++)
				m_pIdx[i] = -1;

			return true;
		}
		else
			return false;
	}

	// ����ɨ����������ƥ���Ӧ��
	bool CreateFromScan(const CCsmScan& scan);

	// �˶�����С�εĵ��Ƿ������Ϊ���ص���
	bool CheckSmallSegOverlap(int nSeg1Start, int nSeg1End, int nSeg2Start, int nSeg2End);

	// �˶���������(��ֱ��)�ĵ��Ƿ������Ϊ�����൱�������ص���
	bool CheckLineSegOverlap(int nSeg1Start, int nSeg1End, int nSeg2Start, int nSeg2End);
};
#endif
