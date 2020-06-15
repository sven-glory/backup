#ifndef __CTimeStamp
#define __CTimeStamp

//
//   ���塰ʱ�������
//
class CTimeStamp
{
public:
	unsigned int m_uTime;              // ʱ�������λ"ms"

public:
	CTimeStamp(unsigned int uTime)
	{
		m_uTime = uTime;
	}

	// ȱʡ���캯����ȡ��ǰʱ��
	CTimeStamp()
	{
		m_uTime = (unsigned int)GetTickCount();
	}
	
	CTimeStamp(const CTimeStamp& other)
	{
		m_uTime = other.m_uTime;
	}
	
	void Stamp(unsigned int uTime)
	{
		m_uTime = uTime;
	}

	// ��������ʱ��Ĳ��
	unsigned int TimeDiff(const CTimeStamp& other)
	{
		return (m_uTime - other.m_uTime);
	}
};
#endif
