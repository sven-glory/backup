#ifndef __CTimeStamp
#define __CTimeStamp

//
//   定义“时间戳”。
//
class CTimeStamp
{
public:
	unsigned int m_uTime;              // 时间戳，单位"ms"

public:
	CTimeStamp(unsigned int uTime)
	{
		m_uTime = uTime;
	}

	// 缺省构造函数，取当前时间
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

	// 计算两个时间的差距
	unsigned int TimeDiff(const CTimeStamp& other)
	{
		return (m_uTime - other.m_uTime);
	}
};
#endif
