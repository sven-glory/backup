#ifndef __CUnorderedPair
#define __CUnorderedPair

///////////////////////////////////////////////////////////////////////////////
//   ���塰�������ݶԡ��ࡣ
class CUnorderedPair
{
public:
	int m_nId1;              // ��һ������
	int m_nId2;              // �ڶ�������

public:
	CUnorderedPair(int nId1, int nId2)
	{
		m_nId1 = nId1;
		m_nId2 = nId2;
	}

	CUnorderedPair(const CUnorderedPair& pair)
	{
		m_nId1 = pair.m_nId1;
		m_nId2 = pair.m_nId2;
	}

	CUnorderedPair()
	{
		m_nId1 = m_nId2 = 0;
	}

	// �ж����������ݶԡ��Ƿ����
	bool operator == (CUnorderedPair& pair)
	{
		if ((m_nId1 == pair.m_nId1 && m_nId2 == pair.m_nId2) ||
			(m_nId1 == pair.m_nId2 && m_nId2 == pair.m_nId1))
			return true;
		else
			return false;
	}

	// �ж����������ݶԡ��Ƿ����
	bool operator != (CUnorderedPair& pair)
	{
		if (*this == pair)
			return false;
		else
			return true;
	}
};
#endif
